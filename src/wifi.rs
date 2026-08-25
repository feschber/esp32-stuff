use std::{ffi::CStr, net::Ipv4Addr};

use embedded_svc::ipv4;
use esp_idf_hal::{modem::Modem, peripheral::PeripheralRef};
use esp_idf_svc::{
    eventloop::{EspEventLoop, System},
    handle::RawHandle,
    ipv4::{Mask, RouterConfiguration, Subnet},
    netif::{EspNetif, NetifConfiguration},
    nvs::EspDefaultNvsPartition,
    wifi::{BlockingWifi, EspWifi, WifiDriver},
};
use esp_idf_sys::{
    esp, esp_netif_dhcp_option_id_t_ESP_NETIF_CAPTIVEPORTAL_URI,
    esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET, esp_netif_dhcps_option, esp_wifi_set_ps,
    wifi_ps_type_t_WIFI_PS_NONE, EspError,
};

const CAPTIVE_PORTAL_URI: &'static [u8] = b"http://192.168.71.1/portal\0";
pub struct AccessPoint {}

pub fn access_point(
    modem: PeripheralRef<'static, Modem>,
    event_loop: EspEventLoop<System>,
    nvs: Option<EspDefaultNvsPartition>,
) -> Result<(BlockingWifi<EspWifi<'_>>, Ipv4Addr), EspError> {
    let wifi = WifiDriver::new(modem, event_loop.clone(), nvs)?;
    let sta_netif = EspNetif::new_with_conf(&NetifConfiguration::wifi_default_client())?;
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
    assert!(!ap_netif.is_up().unwrap());
    let ap_ip = ap_netif.get_ip_info().expect("ip info").ip;
    let wifi = EspWifi::wrap_all(wifi, sta_netif, ap_netif).expect("EspWifi");
    let wifi = BlockingWifi::wrap(wifi, event_loop).expect("blocking wifi");
    log::info!("disabling wifi power save");
    unsafe { esp!(esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE)) }?;
    Ok((wifi, ap_ip))
}

pub fn start_wifi(wifi: &mut BlockingWifi<EspWifi>) -> Result<(), EspError> {
    wifi.start()?;
    Ok(())
}
