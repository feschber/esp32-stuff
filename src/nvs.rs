//! Credential storage in the default NVS partition.
//!
//! The partition is a process-wide singleton: `EspDefaultNvsPartition::take()`
//! returns `ESP_ERR_INVALID_STATE` while a handle is alive, and the WiFi driver
//! holds one for as long as it runs. So these take a borrow of the caller's
//! handle and clone it (it is an `Arc` inside) instead of taking their own.

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsPartitionId};

const WIFI_NS: &'static str = "wifi";
const SSID_KEY: &'static str = "ssid";
const PW_KEY: &'static str = "password";

pub(crate) fn save_wifi_credentials(
    partition: &EspDefaultNvsPartition,
    ssid: &str,
    password: &str,
) -> anyhow::Result<()> {
    let mut namespace = EspNvs::new(partition.clone(), WIFI_NS, true)?;
    namespace.set_raw(SSID_KEY, ssid.as_bytes())?;
    namespace.set_raw(PW_KEY, password.as_bytes())?;
    Ok(())
}

pub(crate) fn clear_wifi_credentials(partition: &EspDefaultNvsPartition) -> anyhow::Result<()> {
    let mut namespace = EspNvs::new(partition.clone(), WIFI_NS, true)?;
    namespace.remove(SSID_KEY)?;
    namespace.remove(PW_KEY)?;
    Ok(())
}

pub(crate) fn load_wifi_credentials(
    partition: &EspDefaultNvsPartition,
) -> anyhow::Result<Option<(String, String)>> {
    let namespace = EspNvs::new(partition.clone(), WIFI_NS, false)?;
    let ssid = read_to_string(&namespace, SSID_KEY)?;
    let password = read_to_string(&namespace, PW_KEY)?;
    match (ssid, password) {
        (Some(ssid), Some(password)) => Ok(Some((ssid, password))),
        _ => Ok(None),
    }
}

fn read_to_string<T: NvsPartitionId>(
    namespace: &EspNvs<T>,
    key: &str,
) -> anyhow::Result<Option<String>> {
    let mut buf = [0u8; 128];
    match namespace.get_raw(key, &mut buf)? {
        Some(val) => Ok(Some(str::from_utf8(val)?.to_string())),
        None => Ok(None),
    }
}
