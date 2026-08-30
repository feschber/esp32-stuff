//! Driver for the FT6336U capacitive touch controller bonded to the
//! GDEQ0426T82 e-paper panel.
//!
//! Coordinates are reported for the panel held in portrait: `x` across the
//! 480-pixel short edge, `y` along the 800-pixel long edge. Those are the
//! display's `y` and `x` respectively -- see [`crate::gdeq0426t82`].
//!
//! The controller's reset line is wired to GPIO39 on this board, which is
//! input-only on the ESP32 and so cannot be driven. It is not used here; the
//! controller resets on power-up anyway.

use esp_idf_svc::hal::{
    delay::TickType, gpio::*, i2c::*, peripheral::Peripheral, units::FromValueType,
};
use esp_idf_sys::EspError;

const ADDR: u8 = 0x38;
const I2C_TIMEOUT_MS: u64 = 20;

/// The FT6336U tracks at most two fingers.
pub const MAX_TOUCHES: usize = 2;

/// Bytes per touch point: x high/low, y high/low, weight, area.
const POINT_BYTES: usize = 6;

/// Expected value of [`Reg::ChipId`] for the FT6336U.
const CHIP_ID_FT6336U: u8 = 0x64;

/// How hard a touch has to be, and how often the controller scans while a
/// finger is down (in ms). Both are the values the vendor driver writes.
const TOUCH_THRESHOLD: u8 = 22;
const SCAN_PERIOD_MS: u8 = 5;

#[repr(u8)]
#[derive(Clone, Copy)]
enum Reg {
    /// 0 for normal reporting, non-zero selects the factory test modes.
    DeviceMode = 0x00,
    /// Low nibble holds the number of points currently down.
    NumTouches = 0x02,
    /// First of [`MAX_TOUCHES`] blocks of [`POINT_BYTES`] bytes.
    Touch1 = 0x03,
    Threshold = 0x80,
    ScanPeriod = 0x88,
    ChipId = 0xA3,
    /// 0 = the interrupt line stays low while a finger is down, 1 = it pulses
    /// once per new touch.
    InterruptMode = 0xA4,
    FirmwareVersion = 0xA6,
}

/// The transition a point reports, from the top two bits of its x high byte.
/// A point that is lifting or absent carries stale coordinates.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Event {
    PressDown,
    LiftUp,
    Contact,
    None,
}

impl Event {
    fn from_x_high(byte: u8) -> Self {
        match byte >> 6 {
            0 => Event::PressDown,
            1 => Event::LiftUp,
            2 => Event::Contact,
            _ => Event::None,
        }
    }

    fn is_down(self) -> bool {
        matches!(self, Event::PressDown | Event::Contact)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Touch {
    /// 0..479, across the short edge of the panel.
    pub x: u16,
    /// 0..799, along the long edge.
    pub y: u16,
    /// Finger id, stable while that finger stays down.
    pub id: u8,
}

/// The burst read in [`Ft6336::read`] assumes the point blocks sit directly
/// behind the status byte.
const _: () = assert!(Reg::Touch1 as u8 == Reg::NumTouches as u8 + 1);

pub struct Ft6336<'d, INT: InputPin> {
    i2c: I2cDriver<'d>,
    /// Held to keep the pin reserved even when nothing reads it; see
    /// [`Ft6336::is_touched`].
    #[allow(dead_code)]
    int: PinDriver<'d, INT, Input>,
}

impl<'d, INT: InputPin> Ft6336<'d, INT> {
    /// `int` is pulled low by the controller while a finger is down. GPIO36 has
    /// no internal pull-up, so the board has to provide one.
    pub fn new(
        i2c: impl Peripheral<P = impl I2c> + 'd,
        sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        int: impl Peripheral<P = INT> + 'd,
    ) -> Result<Self, EspError> {
        let i2c = I2cDriver::new(i2c, sda, scl, &I2cConfig::new().baudrate(400.kHz().into()))?;
        let int = PinDriver::input(int)?;
        let mut touch = Self { i2c, int };

        let chip_id = touch.read_reg(Reg::ChipId)?;
        if chip_id != CHIP_ID_FT6336U {
            log::warn!("ft6336: unexpected chip id {chip_id:#04x}, continuing anyway");
        }

        touch.write_reg(Reg::DeviceMode, 0)?;
        touch.write_reg(Reg::Threshold, TOUCH_THRESHOLD)?;
        touch.write_reg(Reg::ScanPeriod, SCAN_PERIOD_MS)?;
        // Hold the interrupt line low for as long as a finger is down. The
        // controller powers up in trigger mode, where it emits a pulse far too
        // short to catch by polling the pin's level.
        touch.write_reg(Reg::InterruptMode, 0)?;

        log::info!(
            "ft6336: chip {:#04x} fw {:#04x}, mode {}, threshold {}, scan {}ms, int mode {}, status {:#04x}",
            chip_id,
            touch.read_reg(Reg::FirmwareVersion)?,
            touch.read_reg(Reg::DeviceMode)?,
            touch.read_reg(Reg::Threshold)?,
            touch.read_reg(Reg::ScanPeriod)?,
            touch.read_reg(Reg::InterruptMode)?,
            touch.read_reg(Reg::NumTouches)?,
        );
        Ok(touch)
    }

    /// True while the interrupt line is asserted, i.e. there is something to
    /// read. Only meaningful because [`Ft6336::new`] puts the controller in
    /// polling mode; in its power-on trigger mode the line only pulses.
    ///
    /// Callers that poll over I2C anyway do not need this, and are better off
    /// not depending on how INT is wired -- reading the registers works even if
    /// the line does not.
    #[allow(dead_code)]
    pub fn is_touched(&self) -> bool {
        self.int.is_low()
    }

    /// Reads the points that are currently down, skipping any that are lifting
    /// off. Returns empty once the last finger is gone.
    pub fn read(&mut self) -> Result<Vec<Touch>, EspError> {
        // One burst covers the status byte and every point block behind it.
        let mut buf = [0u8; 1 + MAX_TOUCHES * POINT_BYTES];
        self.read_regs(Reg::NumTouches, &mut buf)?;
        let reported = usize::from(buf[0] & 0x0F).min(MAX_TOUCHES);

        let mut touches = Vec::with_capacity(reported);
        for point in buf[1..].chunks_exact(POINT_BYTES).take(reported) {
            if !Event::from_x_high(point[0]).is_down() {
                continue;
            }
            // The high bytes carry the event and the finger id in their top
            // bits, so only the low nibble belongs to the coordinate.
            touches.push(Touch {
                x: u16::from(point[0] & 0x0F) << 8 | u16::from(point[1]),
                y: u16::from(point[2] & 0x0F) << 8 | u16::from(point[3]),
                id: point[2] >> 4,
            });
        }
        Ok(touches)
    }

    fn read_reg(&mut self, reg: Reg) -> Result<u8, EspError> {
        let mut buf = [0u8; 1];
        self.read_regs(reg, &mut buf)?;
        Ok(buf[0])
    }

    fn read_regs(&mut self, reg: Reg, buf: &mut [u8]) -> Result<(), EspError> {
        self.i2c.write_read(
            ADDR,
            &[reg as u8],
            buf,
            TickType::new_millis(I2C_TIMEOUT_MS).into(),
        )
    }

    fn write_reg(&mut self, reg: Reg, value: u8) -> Result<(), EspError> {
        self.i2c.write(
            ADDR,
            &[reg as u8, value],
            TickType::new_millis(I2C_TIMEOUT_MS).into(),
        )
    }
}
