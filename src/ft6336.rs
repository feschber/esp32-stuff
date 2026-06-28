use esp_idf_hal::{delay::TickType, peripheral::PeripheralRef};
use esp_idf_svc::hal::{gpio::*, i2c::*, units::FromValueType};
use esp_idf_sys::EspError;

pub struct Ft6336<INT: InputPin> {
    addr: u8,
    i2c: I2cDriver<'_>,
    int: PinDriver<'_, INT, Input>,
    // rst: PinDriver<'static, Gpio39, Output>,
}

const FT6X36_ADDR: u8 = 0x38;
const FT6X36_PMODE_ACTIVE: u8 = 0x00;
const FT6X36_PMODE_MONITOR: u8 = 0x01;
const FT6X36_PMODE_STANDBY: u8 = 0x02;
const FT6X36_PMODE_HIBERNATE: u8 = 0x03;

const FT6X36_VENDID: u8 = 0x11;
const FT6206_CHIPID: u8 = 0x06;
const FT6236_CHIPID: u8 = 0x36;
const FT6336_CHIPID: u8 = 0x64;

const FT6X36_DEFAULT_THRESHOLD: u8 = 22;

#[repr(u8)]
pub enum Ft6336Regs {
    DeviceMode = 0x00,
    GestureId = 0x01,
    NumTouches = 0x02,
    P1Xh = 0x03,
    P1Xl = 0x04,
    P1Yh = 0x05,
    P1Yl = 0x06,
    P1Weight = 0x07,
    P1Misc = 0x08,
    P2Xh = 0x09,
    P2Xl = 0x0A,
    P2Yh = 0x0B,
    P2Yl = 0x0C,
    P2Weight = 0x0D,
    P2Misc = 0x0E,
    Threshhold = 0x80,
    FilterCoef = 0x85,
    Ctrl = 0x86,
    TimeEnterMonitor = 0x87,
    TouchrateActive = 0x88,
    TouchrateMonitor = 0x89, // value in ms
    RadianValue = 0x91,
    OffsetLeftRight = 0x92,
    OffsetUpDown = 0x93,
    DistanceLeftRight = 0x94,
    DistanceUpDown = 0x95,
    DistanceZoom = 0x96,
    LibVersionH = 0xA1,
    LibVersionL = 0xA2,
    Chipid = 0xA3,
    InterruptMode = 0xA4,
    PowerMode = 0xA5,
    FirmwareVersion = 0xA6,
    PanelId = 0xA8,
    State = 0xBC,
}

const MAX_TOUCHES: usize = 10;

#[derive(Clone, Copy, Default)]
pub struct Touch {
    x: u16,
    y: u16,
}

impl Ft6336<Gpio36> {
    pub fn new(
        i2c0: PeripheralRef<'static, I2C0>,
        sda: PeripheralRef<'static, Gpio32>,
        scl: PeripheralRef<'static, Gpio33>,
        int: PeripheralRef<'static, Gpio36>,
        // rst: PeripheralRef<'static, Gpio39>,
    ) -> Result<Self, EspError> {
        let i2c = I2cDriver::new(i2c0, sda, scl, &I2cConfig::new().baudrate(400.kHz().into()))?;
        let mut int = PinDriver::input(int)?;
        int.set_interrupt_type(InterruptType::NegEdge)?;
        // let mut rst = PinDriver::output(rst)?; // TODO Check if this is actually reset
        let driver = Self {
            addr: FT6X36_ADDR,
            i2c,
            int,
        };
        let chipid = driver.read_reg(Ft6336Regs::Chipid)?;
        if chipid != FT6336_CHIPID {
            panic!("unsupported ft chip");
        }

        // rst.set_low()?;
        // FreeRtos::delay_ms(10);

        Ok(driver)
    }
}

impl<INT: InputPin> Ft6336<INT> {
    fn read_reg(&self, reg: Ft6336Regs) -> Result<u8, EspError> {
        self.read_byte(reg as u8)
    }

    fn read_byte(&self, addr: u8) -> Result<u8, EspError> {
        let mut buf = [0u8];
        self.i2c.write_read(
            self.addr,
            &[addr as u8],
            &mut buf,
            TickType::new_millis(20).into(),
        );
        Ok(buf[0])
    }

    pub fn read_touch(&mut self, idx: u8) -> Result<Option<Touch>, EspError> {
        let touches = self.read_reg(Ft6336Regs::NumTouches)?;
        if idx >= touches {
            return Ok(None);
        }

        const STRIDE: u8 = Ft6336Regs::P2Xh as u8 - Ft6336Regs::P1Xh as u8;
        let xh = self.read_byte(Ft6336Regs::P1Xh as u8 + STRIDE * idx)?;
        let xl = self.read_byte(Ft6336Regs::P1Xl as u8 + STRIDE * idx)?;
        let yh = self.read_byte(Ft6336Regs::P1Yh as u8 + STRIDE * idx)?;
        let yl = self.read_byte(Ft6336Regs::P1Yl as u8 + STRIDE * idx)?;
        let x = (xh as u16) << 8 | xl as u16;
        let y = (yh as u16) << 8 | yl as u16;
        Ok(Some(Touch { x, y }))
    }
}
