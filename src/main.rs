use esp_idf_svc::nvs::EspDefaultNvsPartition;

mod ap;
mod dns;
mod ft6336;
mod gdeq0426t82;
mod http;
mod nvs;
mod oled;
mod paper;
mod qr;
mod wifi;

enum Mode {
    Provisioning,
    Main,
}

impl Mode {
    fn run(self, nvs: EspDefaultNvsPartition) {
        match self {
            Mode::Provisioning => ap::provisioning_mode(nvs),
            Mode::Main => paper::run(),
        }
    }
}

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // Taken once here and passed down: a second `take()` anywhere while this is
    // alive fails with ESP_ERR_INVALID_STATE.
    let partition = EspDefaultNvsPartition::take().expect("nvs partition");

    let mode = match nvs::load_wifi_credentials(&partition) {
        Ok(Some(cred)) => {
            let (ssid, pw) = cred;
            log::info!("loaded credentials: ssid={ssid}, pw={pw}");
            Mode::Main
        }
        Ok(None) => {
            log::warn!("wifi credentials not found!");
            Mode::Provisioning
        }
        Err(e) => {
            log::warn!("failed to load wifi credentials: {e}");
            Mode::Provisioning
        }
    };
    mode.run(partition);
}
