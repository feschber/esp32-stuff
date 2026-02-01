use std::{
    ffi::{c_str, CStr},
    net::Ipv4Addr,
    sync::{Arc, Mutex},
    thread::sleep,
    time::Duration,
};

use esp_idf_svc::{
    hal::{adc::continuous, prelude::Peripherals},
    handle::RawHandle,
    ipv4::{self, Configuration, Mask, RouterConfiguration, Subnet},
    netif::{EspNetif, NetifConfiguration, NetifStack},
    nvs::EspDefaultNvsPartition,
    wifi::{
        self, AccessPointConfiguration, AccessPointInfo, BlockingWifi, ClientConfiguration,
        EspWifi, WifiDriver,
    },
};
use esp_idf_sys::{
    self, esp, esp_netif_dhcp_option_id_t_ESP_NETIF_CAPTIVEPORTAL_URI,
    esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET, esp_netif_dhcps_option, ESP_OK,
};

use crate::dns;

const SSID: &'static str = "magic-esp-wifi";
const WIFI_PW: &'static str = "magic-esp-wifi-pw";
const CAPTIVE_PORTAL_URI: &'static str = "http://192.168.71.1/portal\0";

pub(crate) fn provisioning_mode() {
    let modem = Peripherals::take().expect("peripherals").modem;
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
        let captive_portal_uri = CStr::from_bytes_with_nul(CAPTIVE_PORTAL_URI.as_bytes()).unwrap();
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
    host_ap(&mut wifi);
    let wifi_aps = Arc::new(Mutex::new(Vec::new()));
    std::thread::spawn(move || {
        dns::dns_server(ap_ip).expect("dns failed");
    });
    let server = crate::http::host_server(Arc::clone(&wifi_aps)).expect("http server");
    wifi_scan(&mut wifi, Arc::clone(&wifi_aps));
    // event_loop.subscribe(handle_event);
    // FIXME
    core::mem::forget(wifi);
    core::mem::forget(server);
    // unsafe { esp_idf_sys::esp_restart() }
}

fn host_ap(wifi: &mut BlockingWifi<EspWifi<'_>>) {
    wifi.set_configuration(&wifi::Configuration::Mixed(
        ClientConfiguration::default(),
        AccessPointConfiguration {
            auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
            ssid: SSID.try_into().expect("ssid string"),
            password: WIFI_PW.try_into().expect("pw string"),
            channel: 1,
            max_connections: 4,
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
                continue;
            }
        };
        for ap in &res {
            log::info!("{ap:?}");
        }
        *aps.lock().unwrap() = res;
        sleep(Duration::from_secs(10));
    }
}

// fn handle_event(event: EspEvent) {
//     match event {}
// }
