//! Wi-Fi macro editor and storage. Handler callbacks perform file I/O outside
//! the macro engine lock so BLE report generation can keep running.
use std::{
    fs::{self, File},
    io::{ErrorKind, Read as _, Write as _},
    path::{Path, PathBuf},
    sync::{Arc, atomic::Ordering::Relaxed},
};

use anyhow::{Context as _, Result, bail};
use controller_core::macros::{MAX_JSON_BYTES, MacroDocument, valid_id};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::modem::WifiModemPeripheral,
    http::server::{Configuration, EspHttpConnection, EspHttpServer, Method, Request},
    io::Write as _,
    nvs::EspDefaultNvsPartition,
    sys::{self, EspError},
    wifi::{AccessPointConfiguration, AuthMethod, Configuration as WifiConfiguration, EspWifi},
};
use serde::{Deserialize, Serialize, de::IgnoredAny};
use serde_json::{Value, json};

use crate::State;

const MOUNT: &str = "/macros";
const MAX_MACROS: usize = 64;
const EDITOR: &str = include_str!("../assets/macros.html");
pub const SSID: &str = "SteamSwitch-RS";
const DEFAULT_IP: &str = "192.168.4.1";

type HttpRequest<'request, 'connection> = Request<&'request mut EspHttpConnection<'connection>>;

pub struct MacroWeb {
    _server: EspHttpServer<'static>,
    _storage: SpiffsMount,
    _wifi: EspWifi<'static>,
}

struct SpiffsMount;

impl SpiffsMount {
    fn open() -> Result<Self> {
        // These are static, nul-terminated strings. No other firmware service
        // owns this partition, and the registration is undone in Drop.
        let mut config = sys::esp_vfs_spiffs_conf_t {
            base_path: c"/macros".as_ptr(),
            partition_label: c"storage".as_ptr(),
            max_files: 8,
            ..Default::default()
        };
        let first = unsafe { sys::esp_vfs_spiffs_register(&config) };
        if first != sys::ESP_OK {
            // Formatting is safe on a new, erased partition. Keep existing
            // bytes intact if a mount fails for any other reason.
            let partition = unsafe {
                sys::esp_partition_find_first(
                    sys::esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
                    sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
                    c"storage".as_ptr(),
                )
            };
            let partition = unsafe { partition.as_ref() }.context("Missing macro partition")?;
            let mut block = [0u8; 512];
            for offset in (0..partition.size as usize).step_by(block.len()) {
                let len = block.len().min(partition.size as usize - offset);
                EspError::convert(unsafe {
                    sys::esp_partition_read(partition, offset, block.as_mut_ptr().cast(), len)
                })?;
                if block[..len].iter().any(|&byte| byte != 0xff) {
                    bail!("Could not mount existing macro storage ({first})");
                }
            }
            config.format_if_mount_failed = true;
            EspError::convert(unsafe { sys::esp_vfs_spiffs_register(&config) })?;
        }
        let mounted = Self;
        recover_storage()?;
        Ok(mounted)
    }
}

impl Drop for SpiffsMount {
    fn drop(&mut self) {
        unsafe { sys::esp_vfs_spiffs_unregister(c"storage".as_ptr()) };
    }
}

pub fn start(
    modem: impl WifiModemPeripheral + 'static,
    state: Arc<State>,
    nvs: EspDefaultNvsPartition,
) -> Result<MacroWeb> {
    let storage = SpiffsMount::open()?;
    let event_loop = EspSystemEventLoop::take()?;
    let mut wifi = EspWifi::new(modem, event_loop, Some(nvs))?;
    let configured_ssid = option_env!("STEAM_SWITCH_WIFI_SSID").unwrap_or(SSID);
    let password = option_env!("STEAM_SWITCH_WIFI_PASSWORD").unwrap_or("");
    if !password.is_empty() && !(8..=63).contains(&password.len()) {
        bail!("The Wi-Fi password must be 8–63 bytes or empty");
    }
    wifi.set_configuration(&WifiConfiguration::AccessPoint(AccessPointConfiguration {
        ssid: configured_ssid
            .try_into()
            .map_err(|_| anyhow::anyhow!("SSID is too long"))?,
        password: password
            .try_into()
            .map_err(|_| anyhow::anyhow!("Password is too long"))?,
        auth_method: if password.is_empty() {
            AuthMethod::None
        } else {
            AuthMethod::WPA2Personal
        },
        channel: 6,
        max_connections: 2,
        ..Default::default()
    }))?;
    wifi.start()?;
    let mut server = EspHttpServer::new(&Configuration {
        stack_size: 10 * 1024,
        ..Default::default()
    })?;
    server.fn_handler::<anyhow::Error, _>("/", Method::Get, |req| {
        response(req, 200, "text/html; charset=utf-8", &[], EDITOR.as_bytes())
    })?;
    for (uri, method) in [
        ("/api/status", Method::Get),
        ("/api/macros", Method::Get),
        ("/api/macro", Method::Get),
        ("/api/macro", Method::Post),
        ("/api/macro", Method::Delete),
        ("/api/download", Method::Get),
        ("/api/play", Method::Post),
        ("/api/stop", Method::Post),
        ("/api/record", Method::Post),
        ("/api/rumble", Method::Get),
        ("/api/rumble", Method::Post),
    ] {
        let state = Arc::clone(&state);
        server.fn_handler::<anyhow::Error, _>(uri, method, move |mut req| {
            let result = route(&mut req, &state, uri, method);
            match result {
                Ok((reply, filename)) => {
                    let body = reply.serialize()?;
                    if let Some(filename) = filename {
                        let download = format!("attachment; filename=\"{filename}\"");
                        response(
                            req,
                            200,
                            "application/json",
                            &[("Content-Disposition", &download)],
                            &body,
                        )
                    } else {
                        response(req, 200, "application/json", &[], &body)
                    }
                }
                Err(error) => {
                    let body = serde_json::to_vec(&json!({"error": error.message}))?;
                    response(req, error.status, "application/json", &[], &body)
                }
            }
        })?;
    }
    log::info!("Macro editor on {configured_ssid} at http://{DEFAULT_IP}");
    Ok(MacroWeb {
        _server: server,
        _storage: storage,
        _wifi: wifi,
    })
}

fn response(
    req: HttpRequest<'_, '_>,
    status: u16,
    mime: &str,
    extra: &[(&str, &str)],
    body: &[u8],
) -> Result<()> {
    let mut headers = vec![("Content-Type", mime), ("Cache-Control", "no-store")];
    headers.extend_from_slice(extra);
    let reason = match status {
        200 => "OK",
        400 => "Bad Request",
        404 => "Not Found",
        409 => "Conflict",
        413 => "Payload Too Large",
        507 => "Insufficient Storage",
        _ => "Internal Server Error",
    };
    req.into_response(status, Some(reason), &headers)?
        .write_all(body)?;
    Ok(())
}

struct ApiError {
    status: u16,
    message: String,
}

impl ApiError {
    fn new(status: u16, message: impl Into<String>) -> Self {
        Self {
            status,
            message: message.into(),
        }
    }
    fn invalid(message: impl Into<String>) -> Self {
        Self::new(400, message)
    }
}

impl From<std::io::Error> for ApiError {
    fn from(error: std::io::Error) -> Self {
        let (status, message) = match error.kind() {
            ErrorKind::NotFound => (404, "Macro not found"),
            ErrorKind::StorageFull => (507, "Macro storage is full"),
            _ => (500, "Could not access macro storage"),
        };
        log::warn!("Macro storage: {error}");
        Self::new(status, message)
    }
}

impl From<serde_json::Error> for ApiError {
    fn from(error: serde_json::Error) -> Self {
        Self::invalid(error.to_string())
    }
}

type ApiResult<T> = std::result::Result<T, ApiError>;

enum Reply {
    Json(Value),
    Macro(MacroDocument),
}

impl Reply {
    fn serialize(self) -> Result<Vec<u8>> {
        match self {
            Self::Json(value) => Ok(serde_json::to_vec(&value)?),
            Self::Macro(doc) => Ok(serde_json::to_vec(&MacroWithDuration {
                ms: doc.duration_ms(),
                doc: &doc,
            })?),
        }
    }
}

#[derive(Serialize)]
struct MacroWithDuration<'a> {
    ms: u64,
    #[serde(flatten)]
    doc: &'a MacroDocument,
}

#[derive(Deserialize)]
struct PlayRequest {
    id: Option<String>,
    steps: Option<IgnoredAny>,
}

fn now_us() -> u64 {
    unsafe { sys::esp_timer_get_time() as u64 }
}

fn param<'a>(uri: &'a str, name: &str) -> Option<&'a str> {
    uri.split_once('?')?
        .1
        .split('&')
        .filter_map(|part| part.split_once('='))
        .find_map(|(key, value)| (key == name).then_some(value))
}

fn macro_id(id: Option<&str>) -> ApiResult<&str> {
    id.filter(|id| valid_id(id))
        .ok_or_else(|| ApiError::invalid("Invalid macro ID"))
}

fn body(req: &mut HttpRequest<'_, '_>) -> ApiResult<Vec<u8>> {
    let len = req
        .header("content-length")
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(0);
    if len > MAX_JSON_BYTES {
        return Err(ApiError::new(413, "Macro JSON is too large"));
    }
    let mut data = vec![0; len];
    let mut offset = 0;
    while offset < len {
        let count = req
            .read(&mut data[offset..])
            .map_err(|err| ApiError::invalid(err.to_string()))?;
        if count == 0 {
            return Err(ApiError::invalid("Incomplete request body"));
        }
        offset += count;
    }
    Ok(data)
}

fn route(
    req: &mut HttpRequest<'_, '_>,
    state: &State,
    uri: &str,
    method: Method,
) -> ApiResult<(Reply, Option<String>)> {
    let value = match (uri, method) {
        ("/api/status", _) => {
            let status = state.macro_engine.lock().unwrap().status(now_us());
            json!({"sc2": state.steam_connected.load(Relaxed),
                "console": state.console_connected.load(Relaxed), "streaming": state.console_streaming.load(Relaxed),
                "playing": status.playing, "recording": status.recording, "pos": status.pos,
                "dur": status.dur, "rec_ms": status.rec_ms, "ip": DEFAULT_IP})
        }
        ("/api/macros", _) => list()?,
        ("/api/macro" | "/api/download", Method::Get) => {
            let id = macro_id(param(req.uri(), "id"))?;
            let filename = (uri == "/api/download").then(|| format!("{id}.json"));
            return Ok((Reply::Macro(load(id)?), filename));
        }
        ("/api/macro", Method::Post) => {
            let override_id = param(req.uri(), "id").map(str::to_owned);
            let mut doc = MacroDocument::parse(&body(req)?).map_err(ApiError::invalid)?;
            if let Some(id) = override_id {
                doc.id = id;
            }
            macro_id(Some(&doc.id))?;
            save(&doc)?;
            json!({"ok": true, "id": doc.id})
        }
        ("/api/macro", Method::Delete) => {
            let id = macro_id(param(req.uri(), "id"))?;
            fs::remove_file(path(id, "json"))?;
            json!({"ok": true})
        }
        ("/api/play", _) => {
            let body = body(req)?;
            let request: PlayRequest = serde_json::from_slice(&body)?;
            let doc = if request.steps.is_none() {
                load(macro_id(request.id.as_deref())?)?
            } else {
                MacroDocument::parse(&body).map_err(ApiError::invalid)?
            };
            let ms = doc.duration_ms();
            state.macro_engine.lock().unwrap().play(doc, now_us());
            json!({"ok": true, "ms": ms})
        }
        ("/api/stop", _) => {
            state.macro_engine.lock().unwrap().stop();
            json!({"ok": true})
        }
        ("/api/record", _) => {
            let body = body(req)?;
            let action: Value = if body.is_empty() {
                Value::Null
            } else {
                serde_json::from_slice(&body)?
            };
            match action
                .get("action")
                .and_then(Value::as_str)
                .unwrap_or("start")
            {
                "start" => {
                    let report = *state.report.lock().unwrap();
                    state.macro_engine.lock().unwrap().record(now_us(), &report);
                    json!({"ok": true, "recording": true})
                }
                "stop" => {
                    let doc = state
                        .macro_engine
                        .lock()
                        .unwrap()
                        .finish_recording(now_us())
                        .ok_or_else(|| ApiError::new(409, "There is no active recording"))?;
                    return Ok((Reply::Macro(doc), None));
                }
                _ => return Err(ApiError::invalid("Recording action must be start or stop")),
            }
        }
        ("/api/rumble", _) => {
            let value = |name, default, maximum| -> ApiResult<u64> {
                let Some(raw) = param(req.uri(), name) else {
                    return Ok(default);
                };
                let number = raw
                    .parse::<u64>()
                    .map_err(|_| ApiError::invalid("Invalid rumble setting"))?;
                Ok(number.min(maximum))
            };
            let left = value("l", 60_000, 65_535)?;
            let right = value("r", 60_000, 65_535)?;
            let ms = value("ms", 400, 5_000)?;
            state.rumble(left as u16, right as u16, ms);
            json!({"ok": true, "left": left, "right": right, "ms": ms})
        }
        _ => return Err(ApiError::new(404, "Unknown route")),
    };
    Ok((Reply::Json(value), None))
}

fn path(id: &str, extension: &str) -> PathBuf {
    Path::new(MOUNT).join(format!("{id}.{extension}"))
}

fn files(extension: &str) -> std::io::Result<Vec<String>> {
    let suffix = format!(".{extension}");
    let mut ids = Vec::new();
    for entry in fs::read_dir(MOUNT)? {
        let name = entry?.file_name();
        if let Some(id) = name
            .to_str()
            .and_then(|name| name.strip_suffix(&suffix))
            .filter(|id| valid_id(id))
        {
            ids.push(id.to_owned());
        }
    }
    ids.sort_unstable();
    Ok(ids)
}

fn load(id: &str) -> ApiResult<MacroDocument> {
    let mut data = Vec::new();
    File::open(path(id, "json"))?
        .take((MAX_JSON_BYTES + 1) as u64)
        .read_to_end(&mut data)?;
    let mut doc = MacroDocument::parse(&data).map_err(ApiError::invalid)?;
    doc.id = id.into();
    Ok(doc)
}

fn list() -> ApiResult<Value> {
    let docs = files("json")?
        .iter()
        .filter_map(|id| load(id).ok())
        .map(|doc| {
            json!({"id": doc.id, "name": doc.name, "game": doc.game, "author": doc.author,
               "steps": doc.steps.len(), "ms": doc.duration_ms()})
        })
        .collect::<Vec<_>>();
    Ok(json!(docs))
}

fn save(doc: &MacroDocument) -> ApiResult<()> {
    let target = path(&doc.id, "json");
    let backup = path(&doc.id, "bak");
    let pending = path(&doc.id, "tmp");
    let existing = target.exists();
    if !existing && files("json")?.len() >= MAX_MACROS {
        return Err(ApiError::new(507, "Macro library is full"));
    }
    let data = serde_json::to_vec(doc)?;
    if data.len() > MAX_JSON_BYTES {
        return Err(ApiError::new(413, "Macro JSON is too large"));
    }
    let result = (|| -> std::io::Result<()> {
        let mut file = File::create(&pending)?;
        file.write_all(&data)?;
        file.sync_all()?;
        drop(file);
        if existing {
            fs::rename(&target, &backup)?;
        }
        if let Err(error) = fs::rename(&pending, &target) {
            if existing {
                let _ = fs::rename(&backup, &target);
            }
            return Err(error);
        }
        if existing && let Err(error) = fs::remove_file(backup) {
            log::warn!("Could not remove previous macro version: {error}");
        }
        Ok(())
    })();
    if result.is_err() {
        let _ = fs::remove_file(&pending);
    }
    result.map_err(Into::into)
}

fn recover_storage() -> std::io::Result<()> {
    for id in files("bak")? {
        let backup = path(&id, "bak");
        let target = path(&id, "json");
        if target.exists() {
            fs::remove_file(backup)?;
        } else {
            fs::rename(backup, target)?;
        }
    }
    for id in files("tmp")? {
        fs::remove_file(path(&id, "tmp"))?;
    }
    Ok(())
}
