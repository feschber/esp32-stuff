use anyhow::Result;

use esp_idf_svc::hal::{gpio::*, i2c::*, units::FromValueType};

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
};

use qrcode::{self, QrCode};
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

pub(crate) fn render_qr_code<'a>(
    data: &str,
    display: &mut Ssd1306<
        I2CInterface<esp_idf_svc::hal::i2c::I2cDriver<'a>>,
        ssd1306::prelude::DisplaySize128x64,
        BufferedGraphicsMode<ssd1306::prelude::DisplaySize128x64>,
    >,
) -> Result<()> {
    let code =
        QrCode::with_error_correction_level(data.as_bytes(), qrcode::EcLevel::L).expect("qr code");
    let width = code.width();
    let colors = code.into_colors();
    let scale = 2i32;
    let offset_x = (128 - width as i32 * scale) / 2;
    let offset_y = (64 - width as i32 * scale) / 2;

    for (i, col) in colors.iter().enumerate() {
        let x = (i % width) as i32;
        let y = (i / width) as i32;
        let (x, y) = (offset_x + x * scale, offset_y + y * scale);
        let col = col.select(BinaryColor::On, BinaryColor::Off);
        Rectangle::new(Point::new(x, y), Size::new(scale as u32, scale as u32))
            .into_styled(PrimitiveStyleBuilder::new().fill_color(col).build())
            .draw(display)
            .map_err(|e| anyhow::anyhow!("{e:?}"))?;
    }
    display.flush().map_err(|e| anyhow::anyhow!("{e:?}"))?;
    Ok(())
}
