//! BLE peripheral presented to the Switch 2.
//!
//! GATT layout and interoperability behavior are based on the MIT-licensed
//! esp-cpp/espp Switch2Pro component. See console/LICENSE-espp.

mod protocol;
mod timing;

use std::ffi::{CStr, c_void};
use std::ptr::{null, null_mut};
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering::Relaxed};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use esp_idf_svc::sys;
use esp32_nimble::BLEDevice;
use esp32_nimble::enums::OwnAddrType;
use esp32_nimble::utilities::BleUuid;

use crate::State;
use protocol::{Command, PairingKey, Protocol};
use timing::ReportPacer;

const NO_CONNECTION: u16 = u16::MAX;
const BLE_HS_FOREVER: i32 = i32::MAX; // NimBLE's indefinite duration sentinel.
const COMMON_HANDLE: u16 = 0x000a;
const INPUT_HANDLE: u16 = 0x000e;
const VIBRATION_HANDLE: u16 = 0x0012;
const COMMAND_HANDLE: u16 = 0x0014;
const COMBINED_HANDLE: u16 = 0x0016;
const RESPONSE_HANDLE: u16 = 0x001a;
const COMBINED_RESPONSE_HANDLE: u16 = 0x001e;
const WAKE_DURATION: Duration = Duration::from_secs(10);
const RETRY_DELAY: Duration = Duration::from_millis(100);
const MAX_QUEUED_MBUFS: i32 = 8;
const BOND_NAMESPACE: &CStr = c"switch_rust";
const BOND_KEY: &CStr = c"console";
const ESPP_BOND_NAMESPACE: &CStr = c"switch2pro";
const ESPP_BOND_KEY: &CStr = c"bond";

#[derive(Clone, Copy)]
struct Bond {
    peer: sys::ble_addr_t,
    key: PairingKey,
}

impl Bond {
    const SIZE: usize = 30;

    fn from_bytes(bytes: [u8; Self::SIZE], version: u8) -> Option<Self> {
        if bytes[0] != version || bytes[1] > 3 {
            return None;
        }
        Some(Self {
            peer: sys::ble_addr_t {
                type_: bytes[1],
                val: bytes[2..8].try_into().ok()?,
            },
            key: PairingKey {
                ltk: bytes[8..24].try_into().ok()?,
                host: bytes[24..30].try_into().ok()?,
            },
        })
    }

    fn bytes(self) -> [u8; Self::SIZE] {
        let mut bytes = [0; Self::SIZE];
        bytes[0] = 1;
        bytes[1] = self.peer.type_;
        bytes[2..8].copy_from_slice(&self.peer.val);
        bytes[8..24].copy_from_slice(&self.key.ltk);
        bytes[24..].copy_from_slice(&self.key.host);
        bytes
    }

    fn install(self) -> Result<()> {
        let mut value = sys::ble_store_value_sec::default();
        value.peer_addr = self.peer;
        value.key_size = 16;
        value.ltk = self.key.ltk;
        value.set_ltk_present(1);
        value.set_authenticated(1);
        // NimBLE consumes the native-order key and copies the value into its store.
        check("write console LTK", unsafe {
            sys::ble_store_write_our_sec(&value)
        })
    }
}

struct NvsHandle(sys::nvs_handle_t);

impl NvsHandle {
    fn open() -> Result<Self> {
        let mut handle = 0;
        // The namespace is a C string, and the out pointer is valid for the call.
        check("open console NVS", unsafe {
            sys::nvs_open(
                BOND_NAMESPACE.as_ptr(),
                sys::nvs_open_mode_t_NVS_READWRITE,
                &mut handle,
            )
        })?;
        Ok(Self(handle))
    }

    fn load(&self) -> Result<Option<Bond>> {
        self.read_bond(BOND_KEY, 1)
    }

    fn load_espp_bond() -> Result<Option<Bond>> {
        let mut handle = 0;
        // Older espp firmware may have left a console bond in the shared NVS.
        // Open its namespace read-only to preserve downgrade compatibility.
        let rc = unsafe {
            sys::nvs_open(
                ESPP_BOND_NAMESPACE.as_ptr(),
                sys::nvs_open_mode_t_NVS_READONLY,
                &mut handle,
            )
        };
        if rc == sys::ESP_ERR_NVS_NOT_FOUND {
            return Ok(None);
        }
        check("open espp console NVS", rc)?;
        Self(handle).read_bond(ESPP_BOND_KEY, 0xb2)
    }

    fn read_bond(&self, key: &CStr, version: u8) -> Result<Option<Bond>> {
        let mut bytes = [0u8; Bond::SIZE];
        let mut len = bytes.len();
        // NVS copies at most the indicated buffer capacity.
        let rc =
            unsafe { sys::nvs_get_blob(self.0, key.as_ptr(), bytes.as_mut_ptr().cast(), &mut len) };
        if matches!(
            rc,
            sys::ESP_ERR_NVS_NOT_FOUND | sys::ESP_ERR_NVS_INVALID_LENGTH
        ) {
            return Ok(None);
        }
        check("read console bond", rc)?;
        Ok((len == bytes.len())
            .then(|| Bond::from_bytes(bytes, version))
            .flatten())
    }

    fn save(&self, bond: Bond) -> Result<()> {
        let bytes = bond.bytes();
        // NVS copies the bytes during nvs_set_blob. Commit before returning.
        check("write console bond", unsafe {
            sys::nvs_set_blob(
                self.0,
                BOND_KEY.as_ptr(),
                bytes.as_ptr().cast(),
                bytes.len(),
            )
        })?;
        check("commit console bond", unsafe { sys::nvs_commit(self.0) })
    }
}

impl Drop for NvsHandle {
    fn drop(&mut self) {
        // This object owns the handle and closes it once.
        unsafe { sys::nvs_close(self.0) };
    }
}

struct Callbacks {
    state: Arc<State>,
    protocol: Mutex<Protocol>,
    bond: Mutex<Option<Bond>>,
    pending_bond: Mutex<Option<Bond>>,
    connection: AtomicU16,
    subscribed: AtomicBool,
    encrypted: AtomicBool,
}

static CALLBACKS: OnceLock<Callbacks> = OnceLock::new();

pub struct Console {
    nvs: NvsHandle,
    own_address_type: u8,
    wake_until: Option<Instant>,
    wake_advertisement: bool,
    next_adv_attempt: Instant,
    pacer: ReportPacer,
    stream: StreamStats,
    last_handle: u16,
    sequence: u8,
}

#[derive(Default)]
struct StreamStats {
    period_us: u64,
    since_us: u64,
    last_us: u64,
    sent: u32,
    late: u32,
    busy_retries: u32,
}

impl StreamStats {
    fn update_period(&mut self, now_us: u64, period_us: u64) {
        if self.period_us != period_us {
            log::info!("Switch BLE interval: {period_us} us");
            *self = Self {
                period_us,
                since_us: now_us,
                ..Self::default()
            };
        }
    }

    fn sent(&mut self, now_us: u64) {
        if self.last_us != 0 && now_us - self.last_us >= self.period_us * 3 / 2 {
            self.late += 1;
        }
        self.last_us = now_us;
        self.sent += 1;
    }

    fn log_window(&mut self, now_us: u64) {
        let elapsed = now_us - self.since_us;
        if elapsed >= 5_000_000 {
            let rate = u64::from(self.sent) * 1_000_000 / elapsed;
            log::info!(
                "Switch input: {rate} reports/s, {} long gaps, {} busy retries",
                self.late,
                self.busy_retries,
            );
            self.since_us = now_us;
            self.sent = 0;
            self.late = 0;
            self.busy_retries = 0;
        }
    }
}

impl Console {
    pub fn start(device: &mut BLEDevice, state: Arc<State>) -> Result<Self> {
        let nvs = NvsHandle::open()?;
        let bond = match nvs.load()? {
            Some(bond) => Some(bond),
            None => NvsHandle::load_espp_bond()?,
        };
        if let Some(bond) = bond {
            bond.install()?;
        }
        device
            .set_preferred_mtu(512)
            .map_err(|e| anyhow!("preferred BLE MTU: {e:?}"))?;
        let own_address_type = stable_address_type(device)?;

        CALLBACKS
            .set(Callbacks {
                state,
                protocol: Mutex::new(Protocol::default()),
                bond: Mutex::new(bond),
                pending_bond: Mutex::new(None),
                connection: AtomicU16::new(NO_CONNECTION),
                subscribed: AtomicBool::new(false),
                encrypted: AtomicBool::new(false),
            })
            .map_err(|_| anyhow!("console peripheral already initialized"))?;

        register_gatt()?;
        let now = Instant::now();
        let mut console = Self {
            nvs,
            own_address_type,
            wake_until: None,
            wake_advertisement: false,
            next_adv_attempt: now,
            pacer: ReportPacer::default(),
            stream: StreamStats::default(),
            last_handle: NO_CONNECTION,
            sequence: 0,
        };
        console.advertise(bond.map(|value| value.key.host), false)?;
        Ok(console)
    }

    pub fn poll(&mut self) -> Result<()> {
        let callbacks = CALLBACKS
            .get()
            .context("console peripheral not initialized")?;
        let pending_bond = callbacks.pending_bond.lock().unwrap().take();
        if let Some(bond) = pending_bond {
            if let Err(error) = self.nvs.save(bond) {
                log::warn!("Console pairing won’t survive a reboot: {error}");
            }
        }

        let now = Instant::now();
        let handle = callbacks.connection.load(Relaxed);
        if handle != self.last_handle {
            self.pacer = ReportPacer::default();
            self.stream = StreamStats::default();
            self.sequence = 0;
            self.last_handle = handle;
        }
        let wants_wake = callbacks.state.wake_requested.swap(false, Relaxed);
        if handle == NO_CONNECTION {
            let host = callbacks.bond.lock().unwrap().map(|bond| bond.key.host);
            if wants_wake && host.is_some() {
                self.wake_until = Some(now + WAKE_DURATION);
            }
            let should_wake = self.wake_until.is_some_and(|deadline| now < deadline);
            // Advertising can stop on a failed connection attempt. Retry from
            // the application task after the GAP callback has returned.
            let active = unsafe { sys::ble_gap_adv_active() != 0 };
            if (!active || should_wake != self.wake_advertisement) && now >= self.next_adv_attempt {
                self.next_adv_attempt = now + RETRY_DELAY;
                if let Err(error) = self.advertise(host, should_wake) {
                    log::warn!("Console advertising retry: {error}");
                }
            }
            return Ok(());
        }

        if callbacks.encrypted.load(Relaxed) {
            self.wake_until = None;
        }

        let now_us = crate::runtime::now();
        if !callbacks.state.console_streaming.load(Relaxed) || !self.pacer.due(now_us) {
            return Ok(());
        }
        let Some(desc) = connection(handle) else {
            return Ok(());
        };
        let interval_us = u64::from(desc.conn_itvl.max(4)) * 1_250;
        self.stream.update_period(now_us, interval_us);
        self.stream.log_window(now_us);
        if !has_mbuf_headroom() {
            self.stream.busy_retries += 1;
            return Ok(());
        }
        let mut report = *callbacks.state.report.lock().unwrap();
        let features = callbacks.protocol.lock().unwrap().features;
        report[0] = self.sequence;
        report[11] = if features & 0x20 != 0 { 0x38 } else { 0x30 };
        if features & 4 == 0 {
            report[14..].fill(0);
        } else if report[14] == 0 {
            report[14] = 40;
        }
        if notify(handle, INPUT_HANDLE, &report) == 0 {
            self.sequence = self.sequence.wrapping_add(1);
            self.pacer.sent(now_us, interval_us);
            self.stream.sent(now_us);
        } else {
            self.stream.busy_retries += 1;
        }
        Ok(())
    }

    fn advertise(&mut self, host: Option<[u8; 6]>, wake: bool) -> Result<()> {
        let data = protocol::advertisement(host, wake);
        let mut scan = [0; protocol::NAME.len() + 2];
        scan[0] = protocol::NAME.len() as u8 + 1;
        scan[1] = 9; // Complete Local Name AD type
        scan[2..].copy_from_slice(protocol::NAME);
        let mut params = sys::ble_gap_adv_params::default();
        params.conn_mode = sys::BLE_GAP_CONN_MODE_UND as _;
        params.disc_mode = sys::BLE_GAP_DISC_MODE_GEN as _;

        // NimBLE copies advertising data and parameters. Its callback is static.
        unsafe {
            if sys::ble_gap_adv_active() != 0 {
                check("stop advertising", sys::ble_gap_adv_stop())?;
            }
            check(
                "set advertisement",
                sys::ble_gap_adv_set_data(data.as_ptr(), data.len() as _),
            )?;
            check(
                "set scan response",
                sys::ble_gap_adv_rsp_set_data(scan.as_ptr(), scan.len() as _),
            )?;
            check(
                "start advertising",
                sys::ble_gap_adv_start(
                    self.own_address_type,
                    null(),
                    BLE_HS_FOREVER,
                    &params,
                    Some(gap_event),
                    null_mut(),
                ),
            )?;
        }
        self.wake_advertisement = wake;
        Ok(())
    }
}

fn stable_address_type(device: &mut BLEDevice) -> Result<u8> {
    let mut address = [0; 6];
    // An address stored by the console must survive resets. Prefer the public
    // Bluetooth address, then derive a static-random address from the factory MAC.
    let public_address = unsafe {
        sys::ble_hs_id_copy_addr(
            sys::BLE_OWN_ADDR_PUBLIC as _,
            address.as_mut_ptr(),
            null_mut(),
        )
    };
    if public_address == 0 {
        device.set_own_addr_type(OwnAddrType::Public);
        return Ok(sys::BLE_OWN_ADDR_PUBLIC as _);
    }
    check("read factory Bluetooth address", unsafe {
        sys::esp_read_mac(address.as_mut_ptr(), sys::esp_mac_type_t_ESP_MAC_BT)
    })?;
    address[0] |= 0xc0;
    device
        .set_rnd_addr(address)
        .map_err(|e| anyhow!("set stable BLE address: {e:?}"))?;
    device.set_own_addr_type(OwnAddrType::Random);
    Ok(sys::BLE_OWN_ADDR_RANDOM as _)
}

fn check(operation: &str, code: i32) -> Result<()> {
    if code != 0 {
        bail!("{operation}: code {code}");
    }
    Ok(())
}

fn connection(handle: u16) -> Option<sys::ble_gap_conn_desc> {
    let mut desc = sys::ble_gap_conn_desc::default();
    // NimBLE fills the descriptor only for a live handle.
    (unsafe { sys::ble_gap_conn_find(handle, &mut desc) } == 0).then_some(desc)
}

fn has_mbuf_headroom() -> bool {
    // Leave room for command replies and the Steam Controller connection when
    // the controller's transmit queue takes longer to drain.
    let mut info = sys::os_mempool_info::default();
    let mut pool = null_mut();
    loop {
        // NimBLE owns pool entries until its host shuts down, and terminates the
        // supplied name buffer on each successful iteration.
        pool = unsafe { sys::os_mempool_info_get_next(pool, &mut info) };
        if pool.is_null() {
            return true;
        }
        if unsafe { CStr::from_ptr(info.omi_name.as_ptr()) } == c"msys_1" {
            return info.omi_num_blocks - info.omi_num_free < MAX_QUEUED_MBUFS;
        }
    }
}

fn notify(connection: u16, attribute: u16, data: &[u8]) -> i32 {
    // NimBLE copies the slice and consumes the mbuf even when its send fails.
    let packet = unsafe { sys::ble_hs_mbuf_from_flat(data.as_ptr().cast(), data.len() as _) };
    if packet.is_null() {
        return sys::BLE_HS_ENOMEM as _;
    }
    unsafe { sys::ble_gatts_notify_custom(connection, attribute, packet) }
}

unsafe extern "C" fn gap_event(event: *mut sys::ble_gap_event, _: *mut c_void) -> i32 {
    let Some(callbacks) = CALLBACKS.get() else {
        return 0;
    };
    // NimBLE owns this event for the duration of the callback. Each union member
    // is read only when the event tag selects it.
    let event = unsafe { &*event };
    match u32::from(event.type_) {
        sys::BLE_GAP_EVENT_CONNECT => {
            let connect = unsafe { event.__bindgen_anon_1.connect };
            if connect.status == 0 {
                *callbacks.protocol.lock().unwrap() = Protocol::default();
                callbacks.subscribed.store(false, Relaxed);
                callbacks.encrypted.store(false, Relaxed);
                callbacks.connection.store(connect.conn_handle, Relaxed);
                callbacks.state.console_connected.store(true, Relaxed);
                log::info!("Switch 2 connected");
            }
        }
        sys::BLE_GAP_EVENT_DISCONNECT => {
            callbacks.connection.store(NO_CONNECTION, Relaxed);
            callbacks.subscribed.store(false, Relaxed);
            callbacks.encrypted.store(false, Relaxed);
            callbacks.state.console_connected.store(false, Relaxed);
            callbacks.state.console_streaming.store(false, Relaxed);
            log::info!("Switch 2 disconnected");
        }
        sys::BLE_GAP_EVENT_SUBSCRIBE => {
            let subscribe = unsafe { event.__bindgen_anon_1.subscribe };
            if subscribe.attr_handle == INPUT_HANDLE
                && subscribe.conn_handle == callbacks.connection.load(Relaxed)
            {
                callbacks
                    .subscribed
                    .store(subscribe.cur_notify() != 0, Relaxed);
                update_streaming(callbacks);
            }
        }
        sys::BLE_GAP_EVENT_ENC_CHANGE => {
            let encrypted = unsafe { event.__bindgen_anon_1.enc_change };
            if encrypted.conn_handle == callbacks.connection.load(Relaxed) {
                let ready = encrypted.status == 0
                    && connection(encrypted.conn_handle)
                        .is_some_and(|desc| desc.sec_state.encrypted() != 0);
                callbacks.encrypted.store(ready, Relaxed);
                update_streaming(callbacks);
            }
        }
        _ => {}
    }
    0
}

fn update_streaming(callbacks: &Callbacks) {
    callbacks.state.console_streaming.store(
        callbacks.subscribed.load(Relaxed) && callbacks.encrypted.load(Relaxed),
        Relaxed,
    );
}

unsafe extern "C" fn attribute_access(
    handle: u16,
    attribute: u16,
    context: *mut sys::ble_gatt_access_ctxt,
    _: *mut c_void,
) -> i32 {
    let Some(callbacks) = CALLBACKS.get() else {
        return sys::BLE_ATT_ERR_UNLIKELY as _;
    };
    // The stack provides a valid access context and an owned mbuf for the call.
    let context = unsafe { &*context };
    match u32::from(context.op) {
        sys::BLE_GATT_ACCESS_OP_READ_CHR if attribute == INPUT_HANDLE => {
            let mut report = *callbacks.state.report.lock().unwrap();
            if !callbacks.encrypted.load(Relaxed) {
                report =
                    controller_core::input::neutral(controller_core::input::Battery::default());
            }
            return append_attribute(context, &report);
        }
        sys::BLE_GATT_ACCESS_OP_READ_CHR if attribute == COMMON_HANDLE => {
            // Neutral common controller report (0x05), which a host may read
            // while negotiating the controller-specific streaming report.
            // https://github.com/ndeadly/switch2_controller_research/blob/master/hid_reports.md#input-report-0x05
            let mut report = [0; 63];
            report[10..16].copy_from_slice(&[0, 8, 128, 0, 8, 128]);
            report[31..33].copy_from_slice(&4_000u16.to_le_bytes());
            report[41] = 1;
            return append_attribute(context, &report);
        }
        sys::BLE_GATT_ACCESS_OP_WRITE_CHR
            if matches!(
                attribute,
                COMMAND_HANDLE | COMBINED_HANDLE | VIBRATION_HANDLE
            ) =>
        {
            let mut data = [0u8; 512];
            let mut length = 0u16;
            // Copies from all mbuf segments. An oversized packet is rejected.
            let rc = unsafe {
                sys::ble_hs_mbuf_to_flat(
                    context.om,
                    data.as_mut_ptr().cast(),
                    data.len() as _,
                    &mut length,
                )
            };
            if rc != 0 {
                return sys::BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN as _;
            }
            dispatch(callbacks, handle, attribute, &data[..usize::from(length)]);
        }
        _ => {}
    }
    0
}

fn append_attribute(context: &sys::ble_gatt_access_ctxt, data: &[u8]) -> i32 {
    // NimBLE owns the read mbuf and expects the callback to append its value.
    let rc = unsafe { sys::os_mbuf_append(context.om, data.as_ptr().cast(), data.len() as _) };
    if rc == 0 {
        0
    } else {
        sys::BLE_ATT_ERR_INSUFFICIENT_RES as _
    }
}

fn dispatch(callbacks: &Callbacks, handle: u16, attribute: u16, data: &[u8]) {
    if attribute != COMMAND_HANDLE && callbacks.encrypted.load(Relaxed) {
        let frame = if attribute == COMBINED_HANDLE {
            data.get(..33).unwrap_or(data)
        } else {
            data
        };
        if let Some((left, right)) = protocol::rumble(frame) {
            callbacks.state.rumble(left, right, 150);
        }
    }

    if attribute == VIBRATION_HANDLE {
        return;
    }
    let combined = attribute == COMBINED_HANDLE;
    let Some(command) = Command::parse(data, combined) else {
        return;
    };
    let Some(desc) = connection(handle) else {
        return;
    };
    let Some(reply) = callbacks
        .protocol
        .lock()
        .unwrap()
        .reply(command, desc.our_ota_addr.val)
    else {
        return;
    };
    if let Some(key) = reply.bond {
        let bond = Bond {
            peer: desc.peer_id_addr,
            key,
        };
        // Install before acknowledgement: the console can request LL encryption
        // as soon as the successful finalise reply arrives.
        if let Err(error) = bond.install() {
            log::warn!("Console pairing failed: {error}");
            return;
        }
        *callbacks.bond.lock().unwrap() = Some(bond);
        *callbacks.pending_bond.lock().unwrap() = Some(bond);
    }
    let response = if combined {
        COMBINED_RESPONSE_HANDLE
    } else {
        RESPONSE_HANDLE
    };
    let rc = notify(handle, response, &reply.bytes);
    if rc != 0 {
        log::debug!("Console reply notification failed with code {rc}");
    }
}

const READ: u32 = sys::BLE_GATT_CHR_F_READ as _;
const WRITE: u32 = sys::BLE_GATT_CHR_F_WRITE as _;
const WRITE_NR: u32 = sys::BLE_GATT_CHR_F_WRITE_NO_RSP as _;
const NOTIFY: u32 = sys::BLE_GATT_CHR_F_NOTIFY as _;
const RATE_DESCRIPTOR: &str = "679d5510-5a24-4dee-9557-95df80486ecb";
const RESPONSE_DESCRIPTOR: &str = "b746df8c-f358-495b-9cd2-e3bbeda4f979";
const SERVICE_ONE: &str = "00c5af5d-1964-4e30-8f51-1956f96bd280";
const SERVICE_TWO: &str = "ab7de9be-89fe-49ad-828f-118f09df7fd0";
const SERVICE_ONE_LAYOUT: &[(&str, u32, Option<&str>)] = &[
    ("00c5af5d-1964-4e30-8f51-1956f96bd281", READ, None),
    ("00c5af5d-1964-4e30-8f51-1956f96bd282", WRITE, None),
    ("00c5af5d-1964-4e30-8f51-1956f96bd283", READ, None),
];
const SERVICE_TWO_LAYOUT: &[(&str, u32, Option<&str>)] = &[
    (
        "ab7de9be-89fe-49ad-828f-118f09df7fd2",
        READ | NOTIFY,
        Some(RATE_DESCRIPTOR),
    ),
    (
        "7492866c-ec3e-4619-8258-32755ffcc0f8",
        READ | NOTIFY,
        Some(RATE_DESCRIPTOR),
    ),
    ("cc483f51-9258-427d-a939-630c31f72b05", WRITE_NR, None),
    ("649d4ac9-8eb7-4e6c-af44-1ea54fe5f005", WRITE_NR, None),
    ("3dacbc7e-6955-40b5-8eaf-6f9809e8b379", WRITE_NR, None),
    ("4147423d-fdae-4df7-a4f7-d23e5df59f8d", WRITE_NR, None),
    (
        "c765a961-d9d8-4d36-a20a-5315b111836a",
        NOTIFY,
        Some(RESPONSE_DESCRIPTOR),
    ),
    (
        "506d9f7d-4278-4e95-a549-326ba77657e0",
        NOTIFY,
        Some(RESPONSE_DESCRIPTOR),
    ),
    (
        "d3bd69d2-841c-4241-ab15-f86f406d2a80",
        NOTIFY,
        Some(RESPONSE_DESCRIPTOR),
    ),
    (
        "ab7de9be-89fe-49ad-828f-118f09df7fde",
        READ | NOTIFY,
        Some(RATE_DESCRIPTOR),
    ),
    ("ab7de9be-89fe-49ad-828f-118f09df7fdf", WRITE_NR, None),
    ("cc483f51-9258-427d-a939-630c31f72b06", WRITE_NR, None),
    (
        "7492866c-ec3e-4619-8258-32755ffcc0f9",
        READ | NOTIFY,
        Some(RATE_DESCRIPTOR),
    ),
    ("3dacbc7e-6955-40b5-8eaf-6f9809e8b380", WRITE_NR, None),
];

fn uuid(value: &str) -> Result<sys::ble_uuid_any_t> {
    let parsed =
        BleUuid::from_uuid128_string(value).map_err(|_| anyhow!("invalid UUID: {value}"))?;
    Ok(parsed.into())
}

fn persistent_uuid(value: &str) -> Result<*const sys::ble_uuid_t> {
    let value = Box::leak(Box::new(uuid(value)?));
    // All alternatives of ble_uuid_any_t start with a ble_uuid_t at offset 0.
    Ok(std::ptr::from_ref(value).cast())
}

fn service(name: &str, layout: &[(&str, u32, Option<&str>)]) -> Result<sys::ble_gatt_svc_def> {
    let mut characteristics = Vec::with_capacity(layout.len() + 1);
    for &(name, flags, descriptor) in layout {
        let descriptors = if let Some(descriptor) = descriptor {
            Box::leak(Box::new([
                sys::ble_gatt_dsc_def {
                    uuid: persistent_uuid(descriptor)?,
                    att_flags: (sys::BLE_ATT_F_READ | sys::BLE_ATT_F_WRITE) as _,
                    access_cb: Some(attribute_access),
                    ..Default::default()
                },
                sys::ble_gatt_dsc_def::default(),
            ]))
            .as_mut_ptr()
        } else {
            null_mut()
        };
        characteristics.push(sys::ble_gatt_chr_def {
            uuid: persistent_uuid(name)?,
            flags: flags as _,
            access_cb: Some(attribute_access),
            descriptors,
            ..Default::default()
        });
    }
    characteristics.push(sys::ble_gatt_chr_def::default());
    Ok(sys::ble_gatt_svc_def {
        type_: sys::BLE_GATT_SVC_TYPE_PRIMARY as _,
        uuid: persistent_uuid(name)?,
        characteristics: Box::leak(characteristics.into_boxed_slice()).as_ptr(),
        ..Default::default()
    })
}

fn register_gatt() -> Result<()> {
    // Allocate once for the process lifetime. NimBLE keeps these pointers after
    // registration, including each characteristic and its descriptor array.
    let services = Box::leak(Box::new([
        service(SERVICE_ONE, SERVICE_ONE_LAYOUT)?,
        service(SERVICE_TWO, SERVICE_TWO_LAYOUT)?,
        sys::ble_gatt_svc_def::default(),
    ]));
    // The service order preserves the well-known console attribute handles.
    unsafe {
        check("reset GATT", sys::ble_gatts_reset())?;
        check(
            "count console GATT entries",
            sys::ble_gatts_count_cfg(services.as_ptr()),
        )?;
        check(
            "register console GATT entries",
            sys::ble_gatts_add_svcs(services.as_ptr()),
        )?;
        sys::ble_svc_gap_init();
        sys::ble_svc_gatt_init();
        check(
            "set BLE device name",
            sys::ble_svc_gap_device_name_set(c"Pro Controller".as_ptr()),
        )?;
        check("start GATT", sys::ble_gatts_start())?;
    }

    let service = uuid(SERVICE_TWO)?;
    for (index, expected) in [
        (0, COMMON_HANDLE),
        (1, INPUT_HANDLE),
        (2, VIBRATION_HANDLE),
        (3, COMMAND_HANDLE),
        (4, COMBINED_HANDLE),
        (6, RESPONSE_HANDLE),
        (7, COMBINED_RESPONSE_HANDLE),
    ] {
        let name = SERVICE_TWO_LAYOUT[index].0;
        let characteristic = uuid(name)?;
        let mut actual = 0;
        // NimBLE only borrows the local UUIDs and out handle for the lookup.
        check("find console GATT characteristic", unsafe {
            sys::ble_gatts_find_chr(
                std::ptr::from_ref(&service).cast(),
                std::ptr::from_ref(&characteristic).cast(),
                null_mut(),
                &mut actual,
            )
        })?;
        if actual != expected {
            bail!("console GATT {name} has handle 0x{actual:04x}, expected 0x{expected:04x}");
        }
    }
    Ok(())
}
