use crate::ap::provisioning_mode;

mod ap;
mod nvs;

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    match nvs::load_wifi_credentials() {
        Ok(Some(cred)) => {
            let (ssid, pw) = cred;
            log::info!("loaded credentials: ssid={ssid}, pw={pw}");
        }
        Ok(None) => {
            log::warn!("wifi credentials not found!");
        }
        Err(e) => {
            log::warn!("failed to load wifi credentials: {e}");
        }
    };
    provisioning_mode();
}
