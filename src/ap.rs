use std::{
    ffi::CStr,
    i8,
    net::Ipv4Addr,
    sync::{mpsc, Arc, Mutex},
    thread::sleep,
    time::Duration,
};

use embedded_graphics::{
    framebuffer,
    geometry::{Point, Size},
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::BinaryColor,
    primitives::Rectangle,
    text::{Alignment, LineHeight, Text, TextStyleBuilder},
    Drawable,
};
use esp_idf_hal::{cpu::Core::Core1, delay::FreeRtos, peripheral::Peripheral};
use esp_idf_svc::{
    hal::prelude::Peripherals,
    nvs::EspDefaultNvsPartition,
    wifi::{
        self, AccessPointConfiguration, AccessPointInfo, AuthMethod::WPA2Personal, BlockingWifi,
        ClientConfiguration, EspWifi, WifiDriver,
    },
};
use esp_idf_sys::EspError;

use crate::{
    dns,
    ft6336::{self, Ft6336},
    gdeq0426t82::{self, Bank, Epd, FrameBuffer, Refresh, SCREEN_HEIGHT, SCREEN_WIDTH},
    nvs::save_wifi_credentials,
    oled, qr,
};

#[cfg(feature = "oled")]
use crate::oled;

const SSID: &'static str = "magic-esp-wifi";
const WIFI_PW: &'static str = "magic-esp-wifi-pw";
const CAPTIVE_PORTAL_URI: &'static [u8] = b"http://192.168.71.1/portal\0";

pub(crate) fn provisioning_mode(nvs: EspDefaultNvsPartition) {
    if let Err(e) = try_run(nvs) {
        log::error!("paper: {e:?}");
    }
}

fn try_run(nvs: EspDefaultNvsPartition) -> anyhow::Result<()> {
    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;
    let touch = Ft6336::new(peripherals.i2c0, pins.gpio32, pins.gpio33, pins.gpio36)?;

    #[cfg(feature = "oled")]
    let mut display = oled::setup_display(
        &mut peripherals.pins.gpio5,
        &mut peripherals.pins.gpio6,
        &mut peripherals.i2c0,
    )
    .expect("display");

    let mut epd = Epd::new(
        peripherals.spi3,
        pins.gpio18, // SCLK
        pins.gpio23, // MOSI
        pins.gpio27, // CS
        pins.gpio14, // DC
        pins.gpio12, // RST
        pins.gpio13, // BUSY
    )?;

    epd.init()?;
    epd.fill(0xFF)?;
    log::info!("cleared in {}ms", epd.refresh(Refresh::Full)?);

    let mut framebuffer = FrameBuffer::new();
    draw_ui(&mut framebuffer)?;

    framebuffer.flush(&mut epd, Bank::Both)?;
    let time = epd.refresh(Refresh::Full)?;
    log::info!("base image in {time}ms");
    epd.sleep()?;

    let modem = peripherals.modem.into_ref();
    let event_loop = esp_idf_svc::eventloop::EspSystemEventLoop::take().expect("event loop");
    // The driver keeps this handle for as long as it runs, so hand it a clone
    // and keep ours for saving the credentials below.
    let (mut wifi, ap_ip) = crate::wifi::access_point(modem, event_loop.clone(), Some(nvs.clone()))
        .expect("failed to create wifi");
    start_wifi(&mut wifi)?;
    let wifi_aps = Arc::new(Mutex::new(Vec::new()));
    wifi_scan(&mut wifi, Arc::clone(&wifi_aps));

    #[derive(Clone, Copy)]
    struct DnsCtx {
        ap_ip: Ipv4Addr,
    }

    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: Some(b"dns-task\0"),
        stack_size: 8192,
        priority: 20,
        pin_to_core: Some(Core1),
        ..Default::default()
    }
    .set()
    .expect("thread spawn config");
    std::thread::spawn(move || {
        dns::dns_server(ap_ip).expect("dns failed");
    });
    std::thread::spawn(move || loop {
        // touch.poll();
        FreeRtos::delay_ms(10);
    });

    let (credentials_tx, credentials_rx) = mpsc::channel();
    let _server = crate::http::host_server(Arc::clone(&wifi_aps), credentials_tx)?;
    let (ssid, password) = credentials_rx.recv()?;
    wifi.stop()?;
    save_wifi_credentials(&nvs, &ssid, &password)?;
    unsafe { esp_idf_sys::esp_restart() }
}

fn start_wifi(wifi: &mut BlockingWifi<EspWifi<'_>>) -> anyhow::Result<()> {
    wifi.set_configuration(&wifi::Configuration::Mixed(
        ClientConfiguration {
            // channel: Some(6),
            ..Default::default()
        },
        AccessPointConfiguration {
            auth_method: WPA2Personal,
            ssid: SSID.try_into().expect("ssid string"),
            password: WIFI_PW.try_into().expect("pw string"),
            // channel: 6,
            max_connections: 2,
            ..Default::default()
        },
    ))?;
    wifi.start()?;
    Ok(())
}

fn wifi_scan(wifi: &mut BlockingWifi<EspWifi<'_>>, aps: Arc<Mutex<Vec<AccessPointInfo>>>) {
    loop {
        log::info!("scanning wifi networks");
        let res = match wifi.scan() {
            Ok(v) => v,
            Err(e) => {
                log::warn!("wifi scan failed: {e}");
                sleep(Duration::from_secs(10));
                continue;
            }
        };
        for ap in &res {
            let ssid = ap.ssid.as_str();
            let signal_strength = ap.signal_strength;
            let bars = match ap.signal_strength {
                -50..=i8::MAX => "****",
                -60..-50 => "***.",
                -70..-60 => "**..",
                -80..-70 => "*...",
                _ => "....",
            };
            let protos = ap
                .protocols
                .iter()
                .map(|p| p.to_string())
                .collect::<Vec<_>>();
            let protos = protos.join(",");
            let chan = ap.channel;
            let sec = ap
                .auth_method
                .map(|a| a.to_string())
                .unwrap_or("open".into());
            log::info!("{ssid:<30} chan{chan:>2} {bars} ({signal_strength}dbm) {protos} {sec}");
        }
        *aps.lock().unwrap() = res;
        break;
    }
}

// fn handle_event(event: EspEvent) {
//     match event {}
// }

fn draw_ui(framebuffer: &mut FrameBuffer) -> anyhow::Result<()> {
    let qr_code = qr::QrImage::fit(
        format!("WIFI:T:WPA;S:{SSID};P:{WIFI_PW};;").as_str(),
        Rectangle::new(
            Point::zero(),
            Size::new(SCREEN_WIDTH as u32, SCREEN_HEIGHT as u32),
        ),
    )?;
    let mono_text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    let text_style = TextStyleBuilder::new()
        .line_height(LineHeight::Pixels(50))
        .alignment(Alignment::Center)
        .build();
    let _ = Text::with_text_style(
        "Scan to Setup",
        Point::new(SCREEN_WIDTH as i32 / 2, 100),
        mono_text_style,
        text_style,
    )
    .draw(framebuffer);
    qr_code.draw(framebuffer);
    Ok(())
}
