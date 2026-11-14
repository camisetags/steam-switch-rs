use crate::{console::Console, macros_web, steam};
use anyhow::{Context, Result};
use controller_core::{
    input::{self, Battery, Mapper},
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

#[derive(Default)]
struct Source {
    mapper: Mapper,
    motion: Motion,
    battery: Battery,
    wake_buttons: u32,
}

pub(crate) struct State {
    source: Mutex<Source>,
    live: Mutex<[u8; 63]>,
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
            live: Mutex::new(neutral),
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
        let battery = source.battery;
        let mut report = source.mapper.encode(&input, battery, time);
        if let Some(imu) = input.imu {
            source.motion.apply(imu, time, &mut report);
        }
        *self.live.lock().unwrap() = report;
        self.macro_engine.lock().unwrap().capture(&report, time);
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
        let neutral = input::neutral(source.battery);
        *self.live.lock().unwrap() = neutral;
        *self.report.lock().unwrap() = neutral;
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

    fn output(&self, time: u64, select: bool) {
        let mut live = *self.live.lock().unwrap();
        if select {
            live[2] |= 0x10;
            live[3] |= 0x10;
        }
        *self.report.lock().unwrap() = self.macro_engine.lock().unwrap().output(&live, time);
    }
}

pub fn run() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
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
                thread::sleep(Duration::from_millis(2));
            }
        })
        .context("start Switch output task")?;
    steam::run(device, state, nvs)
}
