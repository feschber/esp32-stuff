use std::sync::{Arc, Mutex};

use embedded_svc::{
    http::{Headers, Method},
    io::{Read, Write},
};
use esp_idf_svc::{http::server::EspHttpServer, wifi::AccessPointInfo};

use serde::{Deserialize, Serialize};

const STACK_SIZE: usize = 10240;
const INDEX_HTML: &str = include_str!("asdf.html");

#[derive(Deserialize)]
struct WifiFormData<'a> {
    ssid: &'a str,
    password: &'a str,
}

#[derive(Serialize)]
struct WifiAdvert<'a> {
    ssid: &'a str,
    signal_strength: i8,
}

const MAX_REQ_LEN: usize = 128;

pub(crate) fn host_server<'a>(
    wifi_aps: Arc<Mutex<Vec<AccessPointInfo>>>,
) -> anyhow::Result<EspHttpServer<'a>> {
    let mut server = create_server()?;
    server.fn_handler::<anyhow::Error, _>("/wifi_credentials", Method::Post, |mut req| {
        let len = req.content_len().unwrap_or(0) as usize;

        if len > MAX_REQ_LEN {
            req.into_status_response(413)?
                .write_all("Request too big".as_bytes())?;
            return Ok(());
        }

        let mut buf = vec![0; len];
        req.read_exact(&mut buf)?;
        let mut resp = req.into_ok_response()?;

        if let Ok(form) = serde_json::from_slice::<WifiFormData>(&buf) {
            write!(resp, "WIFI: {}, {}", form.ssid, form.password)?;
        } else {
            resp.write_all("JSON error".as_bytes())?;
        }

        Ok(())
    })?;
    server.fn_handler::<anyhow::Error, _>("/wifi-networks", Method::Get, {
        let wifi_aps = wifi_aps.clone();
        move |mut req| {
            let len = req.content_len().unwrap_or(0) as usize;

            if len > MAX_REQ_LEN {
                req.into_status_response(413)?
                    .write_all("Request too big".as_bytes())?;
                return Ok(());
            }

            let mut buf = vec![0; len];
            req.read_exact(&mut buf)?;
            let mut resp = req.into_ok_response()?;

            write!(resp, "[")?;
            let aps = wifi_aps.lock().unwrap().clone();
            let len = aps.len();
            for (i, ap) in aps.into_iter().enumerate() {
                if let Ok(json) = serde_json::to_string(&WifiAdvert {
                    ssid: ap.ssid.as_str(),
                    signal_strength: ap.signal_strength,
                }) {
                    write!(resp, "{json}")?;
                    if i < len - 1 {
                        write!(resp, ", ")?;
                    }
                } else {
                    resp.write_all("JSON error".as_bytes())?;
                }
            }
            write!(resp, "]")?;

            Ok(())
        }
    })?;
    server.fn_handler("/*", Method::Get, |req| {
        req.into_ok_response()?
            .write_all(INDEX_HTML.as_bytes())
            .map(|_| ())
    })?;
    Ok(server)
}

fn create_server() -> anyhow::Result<EspHttpServer<'static>> {
    let config = esp_idf_svc::http::server::Configuration {
        stack_size: STACK_SIZE,
        uri_match_wildcard: true,
        ..Default::default()
    };
    Ok(EspHttpServer::new(&config)?)
}
