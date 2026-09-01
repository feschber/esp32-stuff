//! Driver for the Good Display GDEQ0426T82 4.26" monochrome e-paper panel
//! (SSD1677 controller, 4-wire SPI).
//!
//! Panel geometry, as the controller sees it: 800 source lines along x, 480
//! gate lines along y. Held in portrait, x runs along the long edge and y
//! along the short one, which is the transpose of what [`crate::ft6336`]
//! reports.
//!
//! Waveforms come from the controller's OTP, selected by its built-in
//! temperature sensor. The panel also has a "fast" mode that forces the
//! temperature register high to pick a shorter waveform; it is deliberately not
//! implemented, because on this panel it measured *slower* than a normal full
//! refresh (1949ms against 1677ms) while leaving the image visibly under-driven.

use embedded_graphics::{pixelcolor::BinaryColor, prelude::*, primitives::Rectangle};
use esp_idf_svc::hal::{
    delay::{Ets, FreeRtos},
    gpio::*,
    peripheral::Peripheral,
    prelude::*,
    spi::{config::DriverConfig, SpiAnyPins, SpiConfig, SpiDeviceDriver, SpiDriver},
};
use esp_idf_sys::EspError;

/// Panel geometry as the controller addresses it: 800 source lines along x,
/// 480 gate lines along y.
pub const WIDTH: u16 = 800;
pub const HEIGHT: u16 = 480;
const ROW_BYTES: usize = WIDTH as usize / 8;
pub const IMAGE_BYTES: usize = ROW_BYTES * HEIGHT as usize;

/// Geometry as seen when the panel is held the way its artwork reads: portrait,
/// with the controller's y axis running left to right and its x axis running top
/// to bottom. [`FrameBuffer`] draws in these coordinates, and they are what
/// [`crate::ft6336`] reports, so touch maps to the screen one to one.
pub const SCREEN_WIDTH: u16 = HEIGHT;
pub const SCREEN_HEIGHT: u16 = WIDTH;

/// The SSD1677 is specified up to 20MHz; 10MHz leaves margin for the ribbon.
const SPI_HZ: u32 = 10_000_000;

/// How long to wait on BUSY before giving up. The panel needs ~1.7s for a full
/// refresh and ~0.6s for a partial one.
const RESET_TIMEOUT_MS: u32 = 1_000;
const FULL_REFRESH_TIMEOUT_MS: u32 = 5_000;
const PARTIAL_REFRESH_TIMEOUT_MS: u32 = 2_000;

#[repr(u8)]
#[derive(Clone, Copy)]
enum Cmd {
    DriverOutputControl = 0x01,
    GateDrivingVoltage = 0x03,
    SourceDrivingVoltage = 0x04,
    BoosterSoftStart = 0x0C,
    DeepSleep = 0x10,
    DataEntryMode = 0x11,
    SwReset = 0x12,
    TemperatureSensor = 0x18,
    MasterActivation = 0x20,
    UpdateControl1 = 0x21,
    UpdateControl2 = 0x22,
    WriteRamCurrent = 0x24,
    WriteRamPrevious = 0x26,
    WriteLut = 0x32,
    WriteVcom = 0x2C,
    BorderWaveform = 0x3C,
    RamXRange = 0x44,
    RamYRange = 0x45,
    RamXCounter = 0x4E,
    RamYCounter = 0x4F,
}

/// Steps the controller runs on [`Cmd::MasterActivation`], as selected by
/// [`Cmd::UpdateControl2`]. Mode 1 is the flashing full waveform, mode 2 the
/// differential one used for partial refreshes.
mod seq {
    pub const ENABLE_CLOCK: u8 = 0x80;
    pub const ENABLE_ANALOG: u8 = 0x40;
    pub const LOAD_TEMPERATURE: u8 = 0x20;
    pub const LOAD_LUT: u8 = 0x10;
    /// Modifier, not a step of its own: it switches whichever of the load-LUT
    /// and display steps are present from Display Mode 1 to the differential
    /// Display Mode 2. The datasheet's own table for 0x22 spells this out --
    /// 0xC7 is "display with Mode 1", 0xCF the same with Mode 2.
    pub const MODE_2: u8 = 0x08;
    pub const DISPLAY: u8 = 0x04;
    pub const DISABLE_ANALOG: u8 = 0x02;
    pub const DISABLE_CLOCK: u8 = 0x01;
}

/// Which of the controller's two images a write lands in.
///
/// A partial refresh draws the difference between them, so anything put on
/// screen by a full refresh has to go into [`Bank::Both`]; leaving the previous
/// image stale makes the next partial refresh drive from the wrong state and do
/// nothing visible.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Bank {
    Current,
    /// The previous image on its own. Only useful for setting up a known state
    /// to probe what the differential update actually keys on.
    Previous,
    Both,
}

/// Phases pack two bits each into a waveform LUT byte, phase A in the top pair.
/// Levels: 00 VSS, 01 VSH1, 10 VSL, 11 VSH2.
///
/// VSH1 drives a pixel black and VSL drives it white. The vendor's tables look
/// like the opposite at a glance because a full-refresh waveform opens by
/// pushing the pixel to the far extreme and only settles on the target in its
/// last phases -- their "driving Black" LUT starts on VSL but ends on VSH1.
///
/// Measured: the panel runs the LUT at
/// 19.65ms per frame, so a refresh costs `frames * 19.65ms`, plus 218ms if the
/// rails have to be raised and dropped around it.
const VSS: u8 = 0b00;
const VSH1: u8 = 0b01;
const VSL: u8 = 0b10;

/// Packs the four phases of one group into a LUT byte, phase A in the top pair.
const fn phases(a: u8, b: u8, c: u8, d: u8) -> u8 {
    a << 6 | b << 4 | c << 2 | d
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Refresh {
    /// Flashes the whole panel. Slow, but the only way to clear the ghosting
    /// left behind by partial refreshes.
    Full,
    /// Redraws only the pixels that differ from the previous image, using the
    /// waveform from OTP. Slower than [`Refresh::PartialFast`], but its
    /// oscillating waveform is DC balanced, so residue stays clearable.
    Partial,
    /// As [`Refresh::Partial`], but never reloads the waveform, so it uses
    /// whatever [`Epd::load_fast_lut`] last wrote instead of the one in OTP.
    #[allow(dead_code)]
    PartialFast,
    /// As [`Refresh::PartialFast`], but leaves the previous image live so that
    /// the four LUT slots of [`Epd::load_hybrid_lut`] each apply to their own
    /// class of pixel.
    PartialHybrid,
}

pub struct Epd<'d> {
    spi: SpiDeviceDriver<'d, SpiDriver<'d>>,
    cs: PinDriver<'d, AnyOutputPin, Output>,
    dc: PinDriver<'d, AnyOutputPin, Output>,
    rst: PinDriver<'d, AnyOutputPin, Output>,
    busy: PinDriver<'d, AnyInputPin, Input>,
    /// Set while the panel is powered up and still holding the differential
    /// waveform, which lets the next partial refresh skip straight to the
    /// display phase. Raising the analog rails again costs ~81ms, and loading
    /// the waveform another ~1ms, so a burst of updates should only pay it once.
    primed: bool,
}

impl<'d> Epd<'d> {
    /// The panel is write-only, so the bus has no MISO. CS is driven by hand
    /// rather than by the SPI peripheral, so that it can stay asserted across a
    /// whole image transfer.
    pub fn new<SPI: SpiAnyPins>(
        spi: impl Peripheral<P = SPI> + 'd,
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        mosi: impl Peripheral<P = impl OutputPin> + 'd,
        cs: impl OutputPin,
        dc: impl OutputPin,
        rst: impl OutputPin,
        busy: impl InputPin,
    ) -> Result<Self, EspError> {
        let driver = SpiDriver::new(
            spi,
            sclk,
            mosi,
            None::<AnyIOPin>,
            // No DMA: the driver would then refuse any buffer outside internal
            // RAM, and the command bytes below are slice literals that Rust
            // promotes into flash. Transfers here are at most one row, so the
            // FIFO path costs little.
            &DriverConfig::new(),
        )?;
        let spi = SpiDeviceDriver::new(
            driver,
            None::<AnyOutputPin>,
            &SpiConfig::new().baudrate(SPI_HZ.Hz()).write_only(true),
        )?;

        let mut epd = Self {
            spi,
            cs: PinDriver::output(cs.downgrade_output())?,
            dc: PinDriver::output(dc.downgrade_output())?,
            rst: PinDriver::output(rst.downgrade_output())?,
            busy: PinDriver::input(busy.downgrade_input())?,
            primed: false,
        };
        epd.cs.set_high()?;
        epd.rst.set_high()?;
        Ok(epd)
    }

    /// Resets the panel and runs the power-up sequence. Required before the
    /// first refresh, and after [`Epd::sleep`].
    pub fn init(&mut self) -> Result<(), EspError> {
        self.primed = false;
        // Busy-wait rather than yielding: the FreeRTOS tick is 100Hz here, so
        // a one-tick sleep can return in well under the 10ms the panel needs.
        self.rst.set_low()?;
        Ets::delay_us(10_000); // datasheet: at least 10ms
        self.rst.set_high()?;
        Ets::delay_us(10_000);
        self.wait_while_busy("reset", RESET_TIMEOUT_MS)?;

        self.command(Cmd::SwReset)?;
        self.wait_while_busy("software reset", RESET_TIMEOUT_MS)?;

        self.write(Cmd::TemperatureSensor, &[0x80])?; // use the internal sensor
        self.write(Cmd::BoosterSoftStart, &[0xAE, 0xC7, 0xC3, 0xC0, 0x80])?;

        // Number of gate lines to scan, plus interlaced scan mode.
        let gates = HEIGHT - 1;
        self.write(
            Cmd::DriverOutputControl,
            &[gates as u8, (gates >> 8) as u8, 0x02],
        )?;

        // Panel drive voltages. Loading a waveform from OTP is meant to bring
        // its own, but the vendor's tables write these explicitly for every
        // temperature band, with the same values each time.
        self.write(Cmd::GateDrivingVoltage, &[0x17])?;
        self.write(Cmd::SourceDrivingVoltage, &[0x41, 0xA8, 0x32])?;

        // Border waveform (0x3C): bits 7:6 pick the VBD source (00 GS
        // transition, 01 fix level, 10 VCOM, 11 HiZ) and bits 5:4 the level for
        // a fixed one (00 VSS, 01 VSH1, 10 VSL, 11 VSH2). The vendor's 0x01
        // makes the border follow LUT1 as a transition, which is why it drifts
        // to grey; pinning it to VSH1 -- the level the waveform tables use to
        // drive white -- keeps it paper-coloured.
        self.write(Cmd::BorderWaveform, &[0x50])?;
        self.set_window(0, 0, WIDTH, HEIGHT)
    }

    /// Writes a bitmap into controller memory without refreshing. `x` and `w`
    /// are rounded down to whole bytes; the bitmap is `w / 8` bytes per row,
    /// `h` rows, MSB leftmost, 1 = white.
    pub fn draw(
        &mut self,
        bitmap: &[u8],
        x: u16,
        y: u16,
        w: u16,
        h: u16,
        bank: Bank,
    ) -> Result<(), EspError> {
        if bank != Bank::Previous {
            self.write_ram(Cmd::WriteRamCurrent, bitmap, x, y, w, h)?;
        }
        if bank != Bank::Current {
            self.write_ram(Cmd::WriteRamPrevious, bitmap, x, y, w, h)?;
        }
        Ok(())
    }

    /// Fills both images with a repeating byte pattern (0xFF white, 0x00 black).
    pub fn fill(&mut self, pattern: u8) -> Result<(), EspError> {
        for cmd in [Cmd::WriteRamCurrent, Cmd::WriteRamPrevious] {
            self.set_window(0, 0, WIDTH, HEIGHT)?;
            self.command(cmd)?;
            let row = [pattern; ROW_BYTES];
            self.dc.set_high()?;
            self.cs.set_low()?;
            let result = (0..HEIGHT).try_for_each(|_| self.spi.write(&row));
            self.cs.set_high()?;
            result?;
        }
        Ok(())
    }

    /// Drives the panel from controller memory. Blocks until the panel is idle,
    /// and returns how long that took.
    /// Drives the panel from controller memory, over whatever RAM window is
    /// currently set. Blocks until the panel is idle, and returns how long that
    /// took.
    ///
    /// Measured on this panel: a partial refresh takes ~614ms whether the
    /// window covers the full 800x480 or only 128x64. The cost is the
    /// waveform's frame count, not the number of gate lines scanned, so
    /// restricting the window buys nothing.
    pub fn refresh(&mut self, mode: Refresh) -> Result<u32, EspError> {
        // Update control 1: red channel handling and single/dual chip. This
        // panel has no red channel; a full refresh bypasses it so that the
        // previous image cannot affect the flashing phases.
        let (red, sequence, timeout_ms) = match mode {
            Refresh::Full => (
                [0x40, 0x00],
                seq::ENABLE_CLOCK
                    | seq::ENABLE_ANALOG
                    | seq::LOAD_TEMPERATURE
                    | seq::LOAD_LUT
                    | seq::DISPLAY
                    | seq::DISABLE_ANALOG
                    | seq::DISABLE_CLOCK,
                FULL_REFRESH_TIMEOUT_MS,
            ),
            // Mid-burst: the rails are up and the waveform is loaded, so all
            // that is left to do is drive the panel. 392ms against 476ms.
            // Never loads a waveform, so the one from load_fast_lut stays in
            // place. Bypasses the previous image, so the current RAM bit alone
            // selects the LUT however the display mode happens to be set.
            Refresh::PartialHybrid if self.primed => (
                [0x00, 0x00],
                seq::MODE_2 | seq::DISPLAY,
                PARTIAL_REFRESH_TIMEOUT_MS,
            ),
            Refresh::PartialHybrid => (
                [0x00, 0x00],
                seq::ENABLE_CLOCK | seq::ENABLE_ANALOG | seq::MODE_2 | seq::DISPLAY,
                PARTIAL_REFRESH_TIMEOUT_MS,
            ),
            Refresh::PartialFast if self.primed => (
                [0x40, 0x00],
                seq::DISPLAY,
                PARTIAL_REFRESH_TIMEOUT_MS,
            ),
            Refresh::PartialFast => (
                [0x40, 0x00],
                seq::ENABLE_CLOCK | seq::ENABLE_ANALOG | seq::DISPLAY,
                PARTIAL_REFRESH_TIMEOUT_MS,
            ),
            Refresh::Partial if self.primed => (
                [0x00, 0x00],
                seq::MODE_2 | seq::DISPLAY,
                PARTIAL_REFRESH_TIMEOUT_MS,
            ),
            // First of a burst. Leaves the supplies up afterwards: powering
            // them down here and back up next time costs more than the display
            // phase. Call [`Epd::power_off`] once the updates stop, or the
            // standing rails will slowly fade the image.
            Refresh::Partial => (
                [0x00, 0x00],
                seq::ENABLE_CLOCK
                    | seq::ENABLE_ANALOG
                    | seq::LOAD_TEMPERATURE
                    | seq::LOAD_LUT
                    | seq::MODE_2
                    | seq::DISPLAY,
                PARTIAL_REFRESH_TIMEOUT_MS,
            ),
        };

        self.write(Cmd::UpdateControl1, &red)?;
        self.write(Cmd::WriteVcom, &[0x48])?;
        self.write(Cmd::UpdateControl2, &[sequence])?;

        let started = unsafe { esp_idf_sys::esp_timer_get_time() };
        self.command(Cmd::MasterActivation)?;
        self.wait_while_busy("refresh", timeout_ms)?;
        // A full refresh powers down on its way out and leaves the mode 1
        // waveform behind, so only a partial refresh leaves us primed.
        self.primed = mode != Refresh::Full;
        Ok(((unsafe { esp_idf_sys::esp_timer_get_time() } - started) / 1000) as u32)
    }

    /// Replaces the waveform from OTP with a minimal differential one: pixels
    /// that are not changing are held, and each of the two transitions gets a
    /// single phase of `frames`.
    ///
    /// Fewer frames means a faster refresh and a weaker transition; too few and
    /// pixels stop part way, which reads as grey rather than black or white.
    /// Only [`Refresh::PartialFast`] uses this -- any other refresh loads the
    /// waveform from OTP again, so this has to be called afresh afterwards.
    /// Probe kept for re-confirming the LUT slot mapping: gives each of the four
    /// slots a different amount of drive
    /// towards black -- LUT0 four phases, LUT1 three, LUT2 two, LUT3 one -- then
    /// displays in Mode 2 with the previous image live. Whatever pattern of
    /// greys comes out tells us what the hardware keys the LUT choice on.
    #[allow(dead_code)]
    pub fn display_lut_probe(&mut self) -> Result<u32, EspError> {
        let mut ws = [0u8; 105];
        // Phase A..D of group 0, two bits each, 01 = VSH1 = towards black.
        ws[0] = phases(VSH1, VSH1, VSH1, VSH1); // LUT0: 4 phases
        ws[10] = phases(VSH1, VSH1, VSH1, VSS); // LUT1: 3 phases
        ws[20] = phases(VSH1, VSH1, VSS, VSS); // LUT2: 2 phases
        ws[30] = phases(VSH1, VSS, VSS, VSS); // LUT3: 1 phase
        ws[50..55].copy_from_slice(&[5, 5, 5, 5, 0]); // TP[A..D], RP
        ws[100..105].fill(0x22);

        self.write(Cmd::WriteLut, &ws)?;
        self.wait_while_busy("load probe lut", RESET_TIMEOUT_MS)?;
        self.write(Cmd::GateDrivingVoltage, &[0x17])?;
        self.write(Cmd::SourceDrivingVoltage, &[0x41, 0xA8, 0x32])?;
        self.write(Cmd::WriteVcom, &[0x48])?;

        // Previous image live (not bypassed), Mode 2, no LUT load from OTP.
        self.write(Cmd::UpdateControl1, &[0x00, 0x00])?;
        self.write(
            Cmd::UpdateControl2,
            &[seq::ENABLE_CLOCK
                | seq::ENABLE_ANALOG
                | seq::MODE_2
                | seq::DISPLAY
                | seq::DISABLE_ANALOG
                | seq::DISABLE_CLOCK],
        )?;
        let started = unsafe { esp_idf_sys::esp_timer_get_time() };
        self.command(Cmd::MasterActivation)?;
        self.wait_while_busy("probe", FULL_REFRESH_TIMEOUT_MS)?;
        self.primed = false;
        Ok(((unsafe { esp_idf_sys::esp_timer_get_time() } - started) / 1000) as u32)
    }

    /// A differential waveform that drives only the pixels presented as
    /// transitions and holds everything else at exactly zero.
    ///
    /// Confirmed by probing the hardware: the LUT slot is chosen by
    /// `(previous << 1) | current`, where a set bit is white.
    ///
    /// | slot | (prev, cur) | gets |
    /// |---|---|---|
    /// | LUT0 | black, black | nothing -- held |
    /// | LUT1 | black, white | `frames` towards white |
    /// | LUT2 | white, black | `frames` towards black |
    /// | LUT3 | white, white | nothing -- held |
    ///
    /// Holding both unchanged classes is what keeps this usable. A differential
    /// waveform does not need to be self-balanced the way a full refresh does:
    /// charge balances over a pixel's transition history, a drive to black being
    /// cancelled by a later drive to white. That only works while pixels that
    /// are not transitioning contribute nothing, which is why any per-refresh
    /// top-up of standing state -- however small -- accumulates without bound
    /// and cannot be cancelled.
    ///
    /// Use [`Epd::flush_with_drive`] to choose which pixels count as
    /// transitions.
    #[allow(dead_code)] // superseded by load_graded_lut; kept for comparison
    pub fn load_hybrid_lut(&mut self, frames: u8) -> Result<(), EspError> {
        let mut ws = [0u8; 105];
        ws[0] = phases(VSS, VSS, VSS, VSS); // LUT0: held
        ws[10] = phases(VSL, VSS, VSS, VSS); // LUT1: driven white
        ws[20] = phases(VSH1, VSS, VSS, VSS); // LUT2: driven black
        ws[30] = phases(VSS, VSS, VSS, VSS); // LUT3: held
        ws[50..55].copy_from_slice(&[frames, 0, 0, 0, 0]);
        ws[100..105].fill(0x22);

        self.write(Cmd::WriteLut, &ws)?;
        self.wait_while_busy("load lut", RESET_TIMEOUT_MS)?;
        self.write(Cmd::GateDrivingVoltage, &[0x17])?;
        self.write(Cmd::SourceDrivingVoltage, &[0x41, 0xA8, 0x32])?;
        self.write(Cmd::WriteVcom, &[0x48])
    }

    /// Writes both image banks so that each pixel lands in the LUT slot we
    /// want: `target` becomes the current image and `target ^ drive` the
    /// previous one.
    ///
    /// A set bit in `drive` therefore presents that pixel as a transition, so
    /// [`Epd::load_hybrid_lut`] drives it towards whatever `target` says; a
    /// clear bit presents it as unchanged and it is held at zero. Several drive
    /// masks are OR-ed together, which lets a caller keep one mask per recent
    /// pass and give a pixel a bounded number of drives without needing a
    /// per-pixel counter.
    #[allow(dead_code)] // the area variant covers the whole screen if asked
    pub fn flush_with_drive(&mut self, target: &[u8], drive: &[&[u8]]) -> Result<(), EspError> {
        self.write_ram(Cmd::WriteRamCurrent, target, 0, 0, WIDTH, HEIGHT)?;

        self.set_window(0, 0, WIDTH, HEIGHT)?;
        self.command(Cmd::WriteRamPrevious)?;
        self.dc.set_high()?;
        self.cs.set_low()?;
        let mut row = [0u8; ROW_BYTES];
        let result = (0..HEIGHT as usize).try_for_each(|line| {
            let at = line * ROW_BYTES;
            for (i, byte) in row.iter_mut().enumerate() {
                let driven = drive.iter().fold(0u8, |acc, mask| acc | mask[at + i]);
                *byte = target[at + i] ^ driven;
            }
            self.spi.write(&row)
        });
        self.cs.set_high()?;
        result
    }

    /// As [`Epd::flush_with_drive`], but writes only `area` of each bank.
    ///
    /// `area` is in screen coordinates and is rounded outward to whole bytes in
    /// y, because the screen's y is the panel's native x and RAM is addressed in
    /// bytes along it. Since the buffers are full-width, each row of the region
    /// is a slice out of the middle of a framebuffer row -- hence the explicit
    /// stride rather than reusing [`Epd::draw`].
    #[allow(dead_code)] // the graded variant is what the demo uses
    pub fn flush_with_drive_area(
        &mut self,
        target: &[u8],
        drive: &[&[u8]],
        area: Rectangle,
    ) -> Result<(), EspError> {
        let Some(corner) = area.bottom_right() else {
            return Ok(());
        };
        let x0 = area.top_left.x.clamp(0, SCREEN_WIDTH as i32 - 1) as u16;
        let x1 = (corner.x + 1).clamp(0, SCREEN_WIDTH as i32) as u16;
        let y0 = (area.top_left.y.clamp(0, SCREEN_HEIGHT as i32 - 1) as u16) & !7;
        let y1 = (((corner.y + 1).clamp(0, SCREEN_HEIGHT as i32) as u16) + 7) & !7;
        if x1 <= x0 || y1 <= y0 {
            return Ok(());
        }

        let first = (y0 / 8) as usize;
        let len = ((y1 - y0) / 8) as usize;

        for (cmd, xor) in [(Cmd::WriteRamCurrent, false), (Cmd::WriteRamPrevious, true)] {
            // Native x is the screen's y, native y the screen's x.
            self.set_window(y0, x0, y1 - y0, x1 - x0)?;
            self.command(cmd)?;
            self.dc.set_high()?;
            self.cs.set_low()?;
            let mut row = [0u8; ROW_BYTES];
            let result = (x0..x1).try_for_each(|screen_x| {
                let at = screen_x as usize * ROW_BYTES + first;
                let line = &target[at..at + len];
                if !xor {
                    self.spi.write(line)
                } else {
                    for (i, byte) in row[..len].iter_mut().enumerate() {
                        let driven = drive.iter().fold(0u8, |acc, mask| acc | mask[at + i]);
                        *byte = line[i] ^ driven;
                    }
                    self.spi.write(&row[..len])
                }
            });
            self.cs.set_high()?;
            result?;
        }
        Ok(())
    }

    /// A waveform with two strengths of drive towards black, so a stroke can be
    /// taken most of the way in the pass it is drawn and only topped up after.
    ///
    /// | slot | (prev, cur) | gets |
    /// |---|---|---|
    /// | LUT0 | black, black | nothing -- held |
    /// | LUT1 | black, white | `heal_frames` towards black |
    /// | LUT2 | white, black | `fresh_frames` towards black |
    /// | LUT3 | white, white | nothing -- held |
    ///
    /// LUT1 would normally erase towards white. Spending it on a weak black
    /// drive instead is only possible because erasing happens through a full
    /// refresh, never a partial one -- so no pixel ever needs driving white here.
    ///
    /// Note the refresh costs `fresh_frames` regardless of how many pixels are
    /// healing: the phase lengths are shared, and healing rides inside phase A
    /// which the fresh pixels are being driven for anyway.
    pub fn load_graded_lut(&mut self, heal_frames: u8, fresh_frames: u8) -> Result<(), EspError> {
        let extra = fresh_frames.saturating_sub(heal_frames);
        let mut ws = [0u8; 105];
        ws[0] = phases(VSS, VSS, VSS, VSS); // LUT0: held
        ws[10] = phases(VSH1, VSS, VSS, VSS); // LUT1: weak, phase A only
        ws[20] = phases(VSH1, VSH1, VSS, VSS); // LUT2: strong, both phases
        ws[30] = phases(VSS, VSS, VSS, VSS); // LUT3: held
        ws[50..55].copy_from_slice(&[heal_frames, extra, 0, 0, 0]);
        ws[100..105].fill(0x22);

        self.write(Cmd::WriteLut, &ws)?;
        self.wait_while_busy("load lut", RESET_TIMEOUT_MS)?;
        self.write(Cmd::GateDrivingVoltage, &[0x17])?;
        self.write(Cmd::SourceDrivingVoltage, &[0x41, 0xA8, 0x32])?;
        self.write(Cmd::WriteVcom, &[0x48])
    }

    /// Writes `area` of both banks so that `fresh` pixels land in LUT2, `older`
    /// ones in LUT1, and everything else is held.
    ///
    /// The encoding falls out symmetrically: flipping a pixel in the current
    /// bank presents it as `(black, white)`, and flipping it in the previous
    /// bank presents it as `(white, black)`. So
    ///
    /// ```text
    /// current  = target ^ healing
    /// previous = target ^ fresh
    /// ```
    ///
    /// A pixel drawn again while still healing must count as fresh only --
    /// flipping both banks would land it back on a held slot.
    pub fn flush_graded_area(
        &mut self,
        target: &[u8],
        fresh: &[u8],
        older: &[&[u8]],
        area: Rectangle,
    ) -> Result<(), EspError> {
        let Some(corner) = area.bottom_right() else {
            return Ok(());
        };
        let x0 = area.top_left.x.clamp(0, SCREEN_WIDTH as i32 - 1) as u16;
        let x1 = (corner.x + 1).clamp(0, SCREEN_WIDTH as i32) as u16;
        let y0 = (area.top_left.y.clamp(0, SCREEN_HEIGHT as i32 - 1) as u16) & !7;
        let y1 = (((corner.y + 1).clamp(0, SCREEN_HEIGHT as i32) as u16) + 7) & !7;
        if x1 <= x0 || y1 <= y0 {
            return Ok(());
        }
        let first = (y0 / 8) as usize;
        let len = ((y1 - y0) / 8) as usize;

        for (cmd, use_fresh) in [(Cmd::WriteRamCurrent, false), (Cmd::WriteRamPrevious, true)] {
            self.set_window(y0, x0, y1 - y0, x1 - x0)?;
            self.command(cmd)?;
            self.dc.set_high()?;
            self.cs.set_low()?;
            let mut row = [0u8; ROW_BYTES];
            let result = (x0..x1).try_for_each(|screen_x| {
                let at = screen_x as usize * ROW_BYTES + first;
                for (i, byte) in row[..len].iter_mut().enumerate() {
                    let f = fresh[at + i];
                    let flip = if use_fresh {
                        f
                    } else {
                        older.iter().fold(0u8, |acc, m| acc | m[at + i]) & !f
                    };
                    *byte = target[at + i] ^ flip;
                }
                self.spi.write(&row[..len])
            });
            self.cs.set_high()?;
            result?;
        }
        Ok(())
    }

    /// Sets the border waveform register (0x3C). Takes effect on the next
    /// refresh; [`Epd::init`] already picks a fixed white.
    #[allow(dead_code)]
    pub fn set_border(&mut self, value: u8) -> Result<(), EspError> {
        self.write(Cmd::BorderWaveform, &[value])
    }

    #[allow(dead_code)]
    pub fn load_fast_lut(&mut self, frames: u8) -> Result<(), EspError> {
        let mut ws = [0u8; 105];
        // Bytes 0..49 hold LUT0..LUT4, ten bytes each. Only phase A of group 0
        // is used, so only the first byte of each LUT is non-zero.
        //
        // Every LUT drives its pixel straight at the target level rather than
        // holding the ones that have not changed: with Display Update Control 1
        // bypassing the previous image, the current RAM bit alone picks the LUT
        // (Table 6-5), so LUT0/LUT2 are the black cases and LUT1/LUT3 white. An
        // earlier attempt held LUT0 at zero on the assumption that Mode 2 would
        // be indexing on (previous, current); black was then driven by nothing
        // at all, and the screen stayed blank.
        ws[0] = phases(VSH1, VSS, VSS, VSS); // -> black
        ws[10] = phases(VSL, VSS, VSS, VSS); // -> white
        ws[20] = phases(VSH1, VSS, VSS, VSS); // LUT2 = LUT0
        ws[30] = phases(VSL, VSS, VSS, VSS); // LUT3 = LUT1
        // Bytes 50..99 are ten groups of TP[A], TP[B], TP[C], TP[D], RP. A
        // phase length of zero skips that phase, so the rest stay silent.
        ws[50] = frames;
        // Bytes 100..104 set the frame rate; every vendor table uses this value.
        ws[100..105].fill(0x22);

        self.write(Cmd::WriteLut, &ws)?;
        self.wait_while_busy("load lut", RESET_TIMEOUT_MS)?;
        // A table from OTP carries these in its tail (bytes 105..109); a
        // waveform written by hand has to set them separately.
        self.write(Cmd::GateDrivingVoltage, &[0x17])?;
        self.write(Cmd::SourceDrivingVoltage, &[0x41, 0xA8, 0x32])?;
        self.write(Cmd::WriteVcom, &[0x48])
    }

    /// Drops the panel's driving voltages without touching the controller. A
    /// [`Refresh::Partial`] leaves them up so that a burst of updates does not
    /// pay to raise them each time; this puts them back down once the burst is
    /// over, before the standing rails start fading the image.
    pub fn power_off(&mut self) -> Result<(), EspError> {
        self.write(
            Cmd::UpdateControl2,
            &[seq::ENABLE_CLOCK | seq::DISABLE_ANALOG | seq::DISABLE_CLOCK],
        )?;
        self.command(Cmd::MasterActivation)?;
        self.primed = false;
        self.wait_while_busy("power off", PARTIAL_REFRESH_TIMEOUT_MS)
    }

    /// Cuts the panel's internal supplies. Draws ~1uA, but only [`Epd::init`]
    /// brings the controller back.
    #[allow(dead_code)] // not exercised by the demo, but the panel supports it
    pub fn sleep(&mut self) -> Result<(), EspError> {
        self.write(Cmd::DeepSleep, &[0x01])?;
        Ets::delay_us(10_000);
        Ok(())
    }

    /// Selects the rectangle the next RAM write lands in, and points the
    /// address counter at its top left corner. Data entry runs along x first,
    /// with both axes counting up, which is the order bitmaps are stored in.
    fn set_window(&mut self, x: u16, y: u16, w: u16, h: u16) -> Result<(), EspError> {
        let (x_end, y_end) = (x + w - 1, y + h - 1);
        self.write(Cmd::DataEntryMode, &[0x03])?; // x increment, y increment
        self.write(
            Cmd::RamXRange,
            &[x as u8, (x >> 8) as u8, x_end as u8, (x_end >> 8) as u8],
        )?;
        self.write(
            Cmd::RamYRange,
            &[y as u8, (y >> 8) as u8, y_end as u8, (y_end >> 8) as u8],
        )?;
        self.write(Cmd::RamXCounter, &[x as u8, (x >> 8) as u8])?;
        self.write(Cmd::RamYCounter, &[y as u8, (y >> 8) as u8])
    }

    fn write_ram(
        &mut self,
        cmd: Cmd,
        bitmap: &[u8],
        x: u16,
        y: u16,
        w: u16,
        h: u16,
    ) -> Result<(), EspError> {
        // The RAM is addressed in bytes along x, so a window can only start and
        // end on a byte boundary.
        let x = x - x % 8;
        let w = w - w % 8;
        if x >= WIDTH || y >= HEIGHT || w == 0 || h == 0 {
            return Ok(());
        }
        // Clip to the panel, but keep stepping through the bitmap at its own
        // width.
        let source_stride = w as usize / 8;
        let w = w.min(WIDTH - x);
        let h = h.min(HEIGHT - y);
        let row_bytes = w as usize / 8;

        self.set_window(x, y, w, h)?;
        self.command(cmd)?;
        self.dc.set_high()?;
        self.cs.set_low()?;
        // One row per transfer, stepping through the bitmap at its own width so
        // that a clipped window still reads the right bytes.
        let result = (0..h as usize).try_for_each(|line| {
            let start = line * source_stride;
            self.spi.write(&bitmap[start..start + row_bytes])
        });
        self.cs.set_high()?;
        result
    }

    fn command(&mut self, cmd: Cmd) -> Result<(), EspError> {
        self.dc.set_low()?;
        self.cs.set_low()?;
        let result = self.spi.write(&[cmd as u8]);
        self.cs.set_high()?;
        result
    }

    fn write(&mut self, cmd: Cmd, data: &[u8]) -> Result<(), EspError> {
        self.command(cmd)?;
        self.dc.set_high()?;
        self.cs.set_low()?;
        let result = self.spi.write(data);
        self.cs.set_high()?;
        result
    }

    fn wait_while_busy(&mut self, what: &str, timeout_ms: u32) -> Result<(), EspError> {
        let started = unsafe { esp_idf_sys::esp_timer_get_time() };
        while self.busy.is_high() {
            if unsafe { esp_idf_sys::esp_timer_get_time() } - started > (timeout_ms as i64) * 1000 {
                log::warn!("epd: {what} timed out after {timeout_ms}ms");
                return Ok(());
            }
            FreeRtos::delay_ms(1);
        }
        Ok(())
    }
}

/// An in-RAM image of the whole panel, drawable through `embedded-graphics` and
/// pushed to the controller with [`FrameBuffer::flush`].
///
/// Drawing happens in portrait screen coordinates ([`SCREEN_WIDTH`] by
/// [`SCREEN_HEIGHT`]); the transpose into the controller's landscape layout
/// happens per drawn pixel, so [`FrameBuffer::flush`] stays a straight linear
/// write. [`BinaryColor::On`] is black ink.
pub struct FrameBuffer {
    data: Box<[u8]>,
}

impl FrameBuffer {
    pub fn new() -> Self {
        Self {
            data: vec![0xFF; IMAGE_BYTES].into_boxed_slice(),
        }
    }

    pub fn clear_white(&mut self) {
        self.data.fill(0xFF);
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.data
    }

    /// Writes the whole buffer into controller memory. Follow with
    /// [`Epd::refresh`].
    pub fn flush(&self, epd: &mut Epd<'_>, bank: Bank) -> Result<(), EspError> {
        epd.draw(&self.data, 0, 0, WIDTH, HEIGHT, bank)
    }

    /// `x`, `y` are portrait screen coordinates; the controller's x axis is the
    /// screen's y and vice versa.
    fn set_pixel(&mut self, x: u16, y: u16, black: bool) {
        if x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT {
            return;
        }
        let index = x as usize * ROW_BYTES + y as usize / 8;
        let mask = 0x80 >> (y % 8);
        if black {
            self.data[index] &= !mask;
        } else {
            self.data[index] |= mask;
        }
    }
}

/// A set of pixels to drive on the next refresh, in the same layout as
/// [`FrameBuffer`] so it can be handed straight to [`Epd::flush_with_drive`].
/// Drawing [`BinaryColor::On`] marks a pixel for driving.
pub struct Mask {
    data: Box<[u8]>,
}

impl Mask {
    pub fn new() -> Self {
        Self {
            data: vec![0x00; IMAGE_BYTES].into_boxed_slice(),
        }
    }

    pub fn clear(&mut self) {
        self.data.fill(0x00);
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.data
    }

    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.data.iter().all(|byte| *byte == 0)
    }
}

impl Default for Mask {
    fn default() -> Self {
        Self::new()
    }
}

impl OriginDimensions for Mask {
    fn size(&self) -> Size {
        Size::new(SCREEN_WIDTH as u32, SCREEN_HEIGHT as u32)
    }
}

impl DrawTarget for Mask {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            if !color.is_on() {
                continue;
            }
            if let (Ok(x), Ok(y)) = (u16::try_from(point.x), u16::try_from(point.y)) {
                if x < SCREEN_WIDTH && y < SCREEN_HEIGHT {
                    self.data[x as usize * ROW_BYTES + y as usize / 8] |= 0x80 >> (y % 8);
                }
            }
        }
        Ok(())
    }
}

impl Default for FrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl OriginDimensions for FrameBuffer {
    fn size(&self) -> Size {
        Size::new(SCREEN_WIDTH as u32, SCREEN_HEIGHT as u32)
    }
}

impl DrawTarget for FrameBuffer {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            if let (Ok(x), Ok(y)) = (u16::try_from(point.x), u16::try_from(point.y)) {
                self.set_pixel(x, y, color.is_on());
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        for point in area.points() {
            if let (Ok(x), Ok(y)) = (u16::try_from(point.x), u16::try_from(point.y)) {
                self.set_pixel(x, y, color.is_on());
            }
        }
        Ok(())
    }
}
