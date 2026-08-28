#![feature(ip_as_octets)]

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
    fn run(self) {
        match self {
            Mode::Provisioning => ap::provisioning_mode(),
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

    let mode = match nvs::load_wifi_credentials() {
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
    paper::run();
    // mode.run();
}
