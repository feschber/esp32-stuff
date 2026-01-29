mod nvs;

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    match nvs::load_wifi_credentials() {
        Ok(cred) => match cred {
            Some((ssid, pw)) => log::info!("loaded credentials: ssid={ssid}, pw={pw}"),
            None => todo!(),
        },
        Err(e) => log::warn!("failed to load wifi credentials: {e}"),
    }

    log::info!("Hello, world!");
}
