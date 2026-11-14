use crate::{console::Console, macros_web, steam};
use anyhow::{Context, Result};
use controller_core::{
    input::{self, Battery, Mapper, PadEffect, PadFeedback},
    macros::MacroEngine,
    motion::Motion,
};
use esp_idf_svc::{hal::peripherals::Peripherals, nvs::EspDefaultNvsPartition};
use esp32_nimble::{
    BLEDevice,
    enums::{AuthReq, SecurityIOCap},
};
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering::Relaxed},
    },
    thread,
    time::Duration,
};

pub(crate) fn now() -> u64 {
    // ESP-IDF uses a monotonic, nonnegative microsecond clock after boot.
    unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 }
}

struct Source {
    mapper: Mapper,
    motion: Motion,
    battery: Battery,
    wake_buttons: u32,
    live: [u8; 63],
    pad_feedback: PadFeedback,
}

impl Default for Source {
    fn default() -> Self {
        let battery = Battery::default();
        Self {
            mapper: Mapper::default(),
            motion: Motion::default(),
            battery,
            wake_buttons: 0,
            live: input::neutral(battery),
            pad_feedback: PadFeedback::default(),
        }
    }
}

pub(crate) struct State {
    source: Mutex<Source>,
    pub report: Mutex<[u8; 63]>,
    pub macro_engine: Mutex<MacroEngine>,
    pub steam_connected: AtomicBool,
    pub console_connected: AtomicBool,
    pub console_streaming: AtomicBool,
    pub wake_requested: AtomicBool,
    rumble: Mutex<(u16, u16, u64)>,
}

impl State {
    fn new() -> Self {
        let neutral = input::neutral(Battery::default());
        Self {
            source: Mutex::default(),
            report: Mutex::new(neutral),
            macro_engine: Mutex::default(),
            steam_connected: AtomicBool::default(),
            console_connected: AtomicBool::default(),
            console_streaming: AtomicBool::default(),
            wake_requested: AtomicBool::default(),
            rumble: Mutex::default(),
        }
    }

    pub fn input(&self, id: u8, payload: &[u8], time: u64) {
        let Some(input) = input::decode(id, payload) else {
            return;
        };
        let mut source = self.source.lock().unwrap();
        if input.buttons & !source.wake_buttons != 0 && !self.console_connected.load(Relaxed) {
            self.wake_requested.store(true, Relaxed);
        }
        source.wake_buttons = input.buttons;
        source.pad_feedback.update(&input, time);
        let battery = source.battery;
        let mut report = source.mapper.encode(&input, battery, time);
        if let Some(imu) = input.imu {
            source.motion.apply(imu, time, &mut report);
        }
        source.live = report;
    }

    pub fn set_battery(&self, payload: &[u8]) {
        if let Some(battery) = Battery::decode(payload) {
            self.source.lock().unwrap().battery = battery;
        }
    }

    pub fn steam_disconnected(&self) {
        self.steam_connected.store(false, Relaxed);
        let mut source = self.source.lock().unwrap();
        *source = Source::default();
        *self.report.lock().unwrap() = source.live;
        *self.rumble.lock().unwrap() = (0, 0, 0);
    }

    pub fn rumble(&self, left: u16, right: u16, duration_ms: u64) {
        *self.rumble.lock().unwrap() = (
            left,
            right,
            now().saturating_add(duration_ms.saturating_mul(1000)),
        );
    }

    pub fn pending_rumble(&self, now_us: u64) -> (u16, u16) {
        let (left, right, until) = *self.rumble.lock().unwrap();
        if now_us < until {
            (left, right)
        } else {
            (0, 0)
        }
    }

    pub fn take_pad_feedback(&self, time: u64) -> [Option<PadEffect>; 2] {
        self.source.lock().unwrap().pad_feedback.take(time)
    }

    fn output(&self, time: u64, select: bool) {
        let source = self.source.lock().unwrap();
        let mut live = source.live;
        source.mapper.apply_aim(&mut live, time);
        let mut macros = self.macro_engine.lock().unwrap();
        macros.capture(&live, time);
        if select {
            live[2] |= 0x10;
            live[3] |= 0x10;
        }
        *self.report.lock().unwrap() = macros.output(&live, time);
    }
}

pub fn run() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    // The native INFO logger prints several lines per BLE packet on the serial port.
    unsafe {
        esp_idf_svc::sys::esp_log_level_set(
            c"NimBLE".as_ptr(),
            esp_idf_svc::sys::esp_log_level_t_ESP_LOG_WARN,
        );
    }
    let peripherals = Peripherals::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let state = Arc::new(State::new());
    let _editor = match macros_web::start(peripherals.modem, state.clone(), nvs.clone()) {
        Ok(editor) => Some(editor),
        Err(error) => {
            log::error!("Macro editor unavailable: {error:#}");
            None
        }
    };
    let device = BLEDevice::take();
    device
        .security()
        .set_auth(AuthReq::Bond | AuthReq::Sc)
        .set_io_cap(SecurityIOCap::NoInputNoOutput)
        .resolve_rpa();
    let mut console = Console::start(device, state.clone())?;
    let shared = state.clone();
    thread::Builder::new()
        .name("switch-output".into())
        .stack_size(8192)
        .spawn(move || {
            let mut select_until = 0;
            let mut was_streaming = false;
            loop {
                let time = now();
                let streaming = shared.console_streaming.load(Relaxed);
                if streaming && !was_streaming {
                    select_until = time + 1_000_000;
                }
                was_streaming = streaming;
                shared.output(time, streaming && time < select_until);
                if let Err(error) = console.poll() {
                    log::warn!("Switch output: {error:#}");
                }
                thread::sleep(Duration::from_millis(if streaming { 1 } else { 2 }));
            }
        })
        .context("start Switch output task")?;
    steam::run(device, state, nvs)
}
