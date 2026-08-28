use anyhow::Result;

use esp_idf_svc::hal::{gpio::*, i2c::*, units::FromValueType};

use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

pub(crate) fn setup_display<'a>(
    sda: &'a mut Gpio5,
    scl: &'a mut Gpio6,
    i2c0: &'a mut I2C0,
) -> Result<
    Ssd1306<
        I2CInterface<esp_idf_svc::hal::i2c::I2cDriver<'a>>,
        ssd1306::prelude::DisplaySize128x64,
        BufferedGraphicsMode<ssd1306::prelude::DisplaySize128x64>,
    >,
    anyhow::Error,
> {
    let i2c = I2cDriver::new(i2c0, sda, scl, &I2cConfig::new().baudrate(400.kHz().into()))?;

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap();

    display.flush().unwrap();
    Ok(display)
}
