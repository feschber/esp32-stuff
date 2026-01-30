use esp_idf_svc::{
    hal::prelude::Peripherals,
    nvs::EspDefaultNvsPartition,
    wifi::{AccessPointConfiguration, BlockingWifi, Configuration, EspWifi},
};
const SSID: &'static str = "magic-esp-wifi";
const WIFI_PW: &'static str = "magic-esp-wifi-pw";

pub(crate) fn provisioning_mode() {
    let modem = Peripherals::take().expect("peripherals").modem;
    let event_loop = esp_idf_svc::eventloop::EspSystemEventLoop::take().expect("event loop");
    let nvs = EspDefaultNvsPartition::take().expect("failed to load nvs partition");
    let mut wifi = EspWifi::new(modem, event_loop.clone(), Some(nvs)).expect("wifi station");
    wifi.set_configuration(&Configuration::AccessPoint(AccessPointConfiguration {
        auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
        ssid: SSID.try_into().expect("ssid string"),
        password: WIFI_PW.try_into().expect("pw string"),
        channel: 1,
        max_connections: 255,
        ..Default::default()
    }))
    .expect("wifi configuration");
    let mut wifi = BlockingWifi::wrap(wifi, event_loop).expect("blocking wifi");
    wifi.start().expect("failed to start wifi");
    wifi.wait_netif_up()
        .expect("failed to setup network interface");
    // FIXME
    core::mem::forget(wifi);
    // unsafe { esp_idf_sys::esp_restart() }
}
