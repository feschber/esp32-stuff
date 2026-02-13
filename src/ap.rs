use std::{
    ffi::CStr,
    i8,
    net::Ipv4Addr,
    sync::{Arc, Mutex},
    thread::sleep,
    time::Duration,
};

use esp_idf_svc::{
    hal::prelude::Peripherals,
    handle::RawHandle,
    ipv4::{self, Mask, RouterConfiguration, Subnet},
    netif::{EspNetif, NetifConfiguration},
    nvs::EspDefaultNvsPartition,
    wifi::{
        self, AccessPointConfiguration, AccessPointInfo, BlockingWifi, ClientConfiguration,
        EspWifi, WifiDriver,
    },
};
use esp_idf_sys::{
    esp, esp_netif_dhcp_option_id_t_ESP_NETIF_CAPTIVEPORTAL_URI,
    esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET, esp_netif_dhcps_option, esp_wifi_set_ps,
    wifi_ps_type_t_WIFI_PS_NONE,
};

use crate::{dns, oled};

const SSID: &'static str = "magic-esp-wifi";
const WIFI_PW: &'static str = "magic-esp-wifi-pw";
const CAPTIVE_PORTAL_URI: &'static [u8] = b"http://192.168.71.1/portal\0";

pub(crate) fn provisioning_mode() {
    let mut peripherals = Peripherals::take().expect("peripherals");
    let mut display = oled::setup_display(
        &mut peripherals.pins.gpio5,
        &mut peripherals.pins.gpio6,
        &mut peripherals.i2c0,
    )
    .expect("display");
    oled::render_qr_code(
        format!("WIFI:T:WPA;S:{SSID};P:{WIFI_PW};;").as_str(),
        &mut display,
    )
    .expect("render qr code");
    let modem = &mut peripherals.modem;
    let event_loop = esp_idf_svc::eventloop::EspSystemEventLoop::take().expect("event loop");
    let nvs = EspDefaultNvsPartition::take().expect("failed to load nvs partition");
    let wifi = WifiDriver::new(modem, event_loop.clone(), Some(nvs)).expect("wifi driver");
    let sta_netif =
        EspNetif::new_with_conf(&NetifConfiguration::wifi_default_client()).expect("netif");
    let ap_netif = EspNetif::new_with_conf(&NetifConfiguration {
        ip_configuration: Some(ipv4::Configuration::Router(RouterConfiguration {
            subnet: Subnet {
                gateway: Ipv4Addr::new(192, 168, 71, 1),
                mask: Mask(24),
            },
            dhcp_enabled: true,
            dns: Some(Ipv4Addr::new(192, 168, 71, 1)),
            secondary_dns: None,
        })),
        ..NetifConfiguration::wifi_default_router()
    })
    .expect("netif");
    assert!(!ap_netif.is_up().unwrap());
    esp!(unsafe {
        let captive_portal_uri = CStr::from_bytes_with_nul(CAPTIVE_PORTAL_URI).unwrap();
        esp_netif_dhcps_option(
            ap_netif.handle(),
            esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET,
            esp_netif_dhcp_option_id_t_ESP_NETIF_CAPTIVEPORTAL_URI,
            captive_portal_uri.as_ptr() as *mut _,
            CAPTIVE_PORTAL_URI.len() as u32 - 1,
        )
    })
    .expect("set dhcps option");
    let ap_ip = ap_netif.get_ip_info().expect("ip info").ip;
    let wifi = EspWifi::wrap_all(wifi, sta_netif, ap_netif).expect("EspWifi");
    let mut wifi = BlockingWifi::wrap(wifi, event_loop).expect("blocking wifi");
    log::info!("disabling wifi power save");
    unsafe { esp!(esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE)) }
        .expect("failed to set powersaving");
    start_wifi(&mut wifi);
    let wifi_aps = Arc::new(Mutex::new(Vec::new()));
    wifi_scan(&mut wifi, Arc::clone(&wifi_aps));
    std::thread::spawn(move || {
        dns::dns_server(ap_ip).expect("dns failed");
    });
    let server = crate::http::host_server(Arc::clone(&wifi_aps)).expect("http server");
    // event_loop.subscribe(handle_event);
    // FIXME
    core::mem::forget(wifi);
    core::mem::forget(server);
    // unsafe { esp_idf_sys::esp_restart() }
}

fn start_wifi(wifi: &mut BlockingWifi<EspWifi<'_>>) {
    wifi.set_configuration(&wifi::Configuration::Mixed(
        ClientConfiguration {
            // channel: Some(6),
            ..Default::default()
        },
        AccessPointConfiguration {
            auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
            ssid: SSID.try_into().expect("ssid string"),
            password: WIFI_PW.try_into().expect("pw string"),
            // channel: 6,
            max_connections: 2,
            ..Default::default()
        },
    ))
    .expect("wifi configuration");
    wifi.start().expect("failed to start wifi");
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
                .map(|p| match p {
                    wifi::Protocol::P802D11B => "b",
                    wifi::Protocol::P802D11BG => "bg",
                    wifi::Protocol::P802D11BGN => "bgn",
                    wifi::Protocol::P802D11BGNLR => "bgnlr",
                    wifi::Protocol::P802D11LR => "lr",
                })
                .collect::<Vec<_>>();
            let protos = protos.join(",");
            let chan = ap.channel;
            let sec = match ap.auth_method {
                Some(auth) => match auth {
                    wifi::AuthMethod::None => "none",
                    wifi::AuthMethod::WEP => "WEP",
                    wifi::AuthMethod::WPA => "WPA",
                    wifi::AuthMethod::WPA2Personal => "WPA2",
                    wifi::AuthMethod::WPAWPA2Personal => "WPA / WPA2",
                    wifi::AuthMethod::WPA2Enterprise => "WPA2 Enterprise",
                    wifi::AuthMethod::WPA3Personal => "WPA3",
                    wifi::AuthMethod::WPA2WPA3Personal => "WPA2 / WPA3",
                    wifi::AuthMethod::WAPIPersonal => "WAPI",
                },
                None => "open",
            };
            log::info!("{ssid:<30} chan{chan:>2} {bars} ({signal_strength}dbm) {protos} {sec}");
        }
        *aps.lock().unwrap() = res;
        break;
    }
}

// fn handle_event(event: EspEvent) {
//     match event {}
// }
