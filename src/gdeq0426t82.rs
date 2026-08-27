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
    pub const LOAD_LUT_MODE_1: u8 = 0x10;
    pub const LOAD_LUT_MODE_2: u8 = 0x08;
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
    Both,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Refresh {
    /// Flashes the whole panel. Slow, but the only way to clear the ghosting
    /// left behind by partial refreshes.
    Full,
    /// Redraws only the pixels that differ from the previous image, without
    /// flashing. Leaves ghosting behind, so run a full refresh now and then.
    Partial,
}

pub struct Epd<'d> {
    spi: SpiDeviceDriver<'d, SpiDriver<'d>>,
    cs: PinDriver<'d, AnyOutputPin, Output>,
    dc: PinDriver<'d, AnyOutputPin, Output>,
    rst: PinDriver<'d, AnyOutputPin, Output>,
    busy: PinDriver<'d, AnyInputPin, Input>,
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
        };
        epd.cs.set_high()?;
        epd.rst.set_high()?;
        Ok(epd)
    }

    /// Resets the panel and runs the power-up sequence. Required before the
    /// first refresh, and after [`Epd::sleep`].
    pub fn init(&mut self) -> Result<(), EspError> {
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

        self.write(Cmd::BorderWaveform, &[0x01])?;
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
        self.write_ram(Cmd::WriteRamCurrent, bitmap, x, y, w, h)?;
        if bank == Bank::Both {
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
                    | seq::LOAD_LUT_MODE_1
                    | seq::DISPLAY
                    | seq::DISABLE_ANALOG
                    | seq::DISABLE_CLOCK,
                FULL_REFRESH_TIMEOUT_MS,
            ),
            // Powering down between refreshes costs ~100ms on the next one, but
            // leaving the panel's supplies up makes the image fade while idle.
            Refresh::Partial => (
                [0x00, 0x00],
                seq::ENABLE_CLOCK
                    | seq::ENABLE_ANALOG
                    | seq::LOAD_TEMPERATURE
                    | seq::LOAD_LUT_MODE_1
                    | seq::LOAD_LUT_MODE_2
                    | seq::DISPLAY
                    | seq::DISABLE_ANALOG
                    | seq::DISABLE_CLOCK,
                PARTIAL_REFRESH_TIMEOUT_MS,
            ),
        };

        self.write(Cmd::UpdateControl1, &red)?;
        self.write(Cmd::WriteVcom, &[0x48])?;
        self.write(Cmd::UpdateControl2, &[sequence])?;

        let started = unsafe { esp_idf_sys::esp_timer_get_time() };
        self.command(Cmd::MasterActivation)?;
        self.wait_while_busy("refresh", timeout_ms)?;
        Ok(((unsafe { esp_idf_sys::esp_timer_get_time() } - started) / 1000) as u32)
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
