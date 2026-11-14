//! A BLE central for Steam Controller (2026).
//!
//! The Valve service has separate characteristics for input, features,
//! battery and haptics. The standard HID service activates the trackpads.
//! UUIDs and wire formats are documented by the JoypadOS and SDL drivers.
//!
//! https://github.com/joypad-ai/joypad-os/blob/main/src/bt/btstack/btstack_host.c
//! https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_steam_triton.c

mod wire;

use std::future::{Future, poll_fn};
use std::pin::pin;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::task::Poll;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use esp_idf_svc::hal::task::asynch::Notification;
use esp_idf_svc::hal::task::block_on;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition};
use esp32_nimble::{
    BLEAddress, BLEClient, BLEDevice, BLERemoteCharacteristic, BLEScan, utilities::BleUuid, uuid128,
};

use crate::State;

const VALVE: BleUuid = uuid128!("100f6c32-1735-4313-b402-38567131e5f3");
const INPUT_45: BleUuid = uuid128!("100f6c7a-1735-4313-b402-38567131e5f3");
const INPUT_47: BleUuid = uuid128!("100f6c7c-1735-4313-b402-38567131e5f3");
const FEATURES: BleUuid = uuid128!("100f6c34-1735-4313-b402-38567131e5f3");
const BATTERY: BleUuid = uuid128!("100f6c78-1735-4313-b402-38567131e5f3");
const HAPTICS: BleUuid = uuid128!("100f6cb5-1735-4313-b402-38567131e5f3");
const HID: BleUuid = BleUuid::Uuid16(0x1812);
const HID_REPORT: BleUuid = BleUuid::Uuid16(0x2a4d);

const KEEPALIVE: Duration = Duration::from_millis(2200);
const RUMBLE_REFRESH: Duration = Duration::from_millis(25);
const POLL: Duration = Duration::from_millis(10);

struct Controller {
    features: BLERemoteCharacteristic,
    haptics: Option<BLERemoteCharacteristic>,
}

fn monotonic_us() -> u64 {
    // ESP-IDF's monotonic clock is safe to use on the NimBLE task as well.
    unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 }
}

async fn subscribe(report: &mut BLERemoteCharacteristic) -> Result<()> {
    if report.subscribe_notify(true).await.is_err() {
        report.subscribe_notify(false).await?;
    }
    Ok(())
}

async fn set_feature(feature: &mut BLERemoteCharacteristic, packet: &[u8]) -> Result<()> {
    feature
        .write_value(packet, !feature.can_write_no_response())
        .await?;
    Ok(())
}

async fn attach(client: &mut BLEClient, state: &Arc<State>) -> Result<Controller> {
    let valve = client
        .get_service(VALVE)
        .await
        .context("Steam Controller Valve service")?;
    let mut features = valve
        .get_characteristic(FEATURES)
        .await
        .context("Steam Controller feature report")?
        .clone();
    let haptics = valve.get_characteristic(HAPTICS).await.ok().cloned();

    let mut reports = valve.get_characteristics().await?;
    let report = reports
        .find(|report| report.uuid() == INPUT_45 || report.uuid() == INPUT_47)
        .context("Steam Controller input report")?;
    let id = if report.uuid() == INPUT_45 {
        0x45
    } else {
        0x47
    };
    let input_state = Arc::clone(state);
    report.on_notify(move |bytes| input_state.input(id, bytes, monotonic_us()));
    subscribe(report)
        .await
        .context("Steam Controller input subscription")?;

    set_feature(&mut features, &wire::LIZARD_OFF).await?;
    set_feature(&mut features, &wire::MOTION_ON).await?;

    if let Ok(battery) = valve.get_characteristic(BATTERY).await {
        let battery_state = Arc::clone(state);
        battery.on_notify(move |bytes| battery_state.set_battery(bytes));
        if let Err(error) = subscribe(battery).await {
            log::warn!("Battery notifications unavailable: {error:#}");
        }
    }

    let mut subscriptions = 0;
    if let Ok(hid) = client.get_service(HID).await {
        match hid.get_characteristics().await {
            Ok(reports) => {
                for report in
                    reports.filter(|report| report.uuid() == HID_REPORT && report.can_notify())
                {
                    if subscribe(report).await.is_ok() {
                        subscriptions += 1;
                    }
                }
            }
            Err(error) => log::warn!("HID report discovery failed: {error}"),
        }
    }
    // Characteristic discovery can take longer than the controller watchdog.
    set_feature(&mut features, &wire::LIZARD_OFF).await?;
    anyhow::ensure!(client.connected(), "Steam Controller detached during setup");
    state.steam_connected.store(true, Ordering::Release);
    log::info!("Steam Controller ready, input 0x{id:02x}, {subscriptions} HID subscriptions");
    Ok(Controller { features, haptics })
}

fn settle_speed(client: &mut BLEClient, attempted: &mut usize) {
    const RANGES: [(u16, u16); 3] = [(6, 6), (6, 12), (12, 24)];
    let Some(&(min, max)) = RANGES.get(*attempted) else {
        return;
    };
    if client.desc().is_ok_and(|link| link.interval() <= 6) {
        *attempted = RANGES.len();
    } else {
        if let Err(error) = client.update_conn_params(min, max, 0, 400) {
            log::debug!("Steam Controller interval request failed: {error}");
        }
        *attempted += 1;
    }
}

fn service(client: &mut BLEClient, state: &State, controller: &mut Controller) -> Result<()> {
    let mut last_feature = Instant::now();
    let mut last_rumble = Instant::now();
    let mut previous_rumble = (0, 0);
    let mut last_interval_request = Instant::now();
    let mut attempts = 0;
    let mut feature_failures = 0;
    while client.connected() {
        let now = Instant::now();
        if now.duration_since(last_interval_request) >= Duration::from_secs(1) {
            settle_speed(client, &mut attempts);
            last_interval_request = now;
        }
        if now.duration_since(last_feature) >= KEEPALIVE {
            match block_on(set_feature(&mut controller.features, &wire::LIZARD_OFF)) {
                Ok(()) => feature_failures = 0,
                Err(error) => {
                    feature_failures += 1;
                    log::warn!("Steam Controller keepalive failed: {error:#}");
                    if feature_failures >= 3 {
                        return Err(error);
                    }
                }
            }
            last_feature = now;
        }

        let rumble = state.pending_rumble(monotonic_us());
        let refresh = rumble != (0, 0) && now.duration_since(last_rumble) >= RUMBLE_REFRESH;
        if let Some(output) = controller.haptics.as_mut()
            && (rumble != previous_rumble || refresh)
            && now.duration_since(last_rumble) >= POLL
        {
            last_rumble = now;
            let bytes = wire::rumble(rumble.0, rumble.1);
            match block_on(output.write_value(&bytes, false)) {
                Ok(()) => previous_rumble = rumble,
                Err(error) => log::debug!("Steam Controller rumble write failed: {error}"),
            }
        }
        thread::sleep(POLL);
    }
    Ok(())
}

fn is_steam_name(name: &[u8]) -> bool {
    name.windows(5)
        .any(|prefix| prefix.eq_ignore_ascii_case(b"steam"))
}

async fn open_link(
    client: &mut BLEClient,
    address: &BLEAddress,
    disconnected: &Notification,
) -> Result<()> {
    let mut opening = pin!(async {
        client
            .connect(address)
            .await
            .context("Steam Controller connect")?;
        if !client.desc().is_ok_and(|link| link.encrypted()) {
            client
                .secure_connection()
                .await
                .context("Steam Controller encryption")?;
        }
        Ok(())
    });
    // NimBLE's Rust client waits for MTU and encryption events. A disconnect can
    // end either native procedure before those events reach the waiting client.
    poll_fn(|cx| {
        if disconnected.poll_wait(cx).is_ready() {
            return Poll::Ready(Err(anyhow::anyhow!(
                "Steam Controller disconnected during setup"
            )));
        }
        opening.as_mut().poll(cx)
    })
    .await
}

pub fn run(device: &BLEDevice, state: Arc<State>, nvs: EspDefaultNvsPartition) -> Result<()> {
    let allowed = option_env!("STEAM_CONTROLLER_MAC")
        .unwrap_or_default()
        .split([' ', ',', ';'])
        .filter(|address| !address.is_empty())
        .map(|address| {
            BLEAddress::from_str(address, esp32_nimble::BLEAddressType::Public)
                .with_context(|| format!("invalid STEAM_CONTROLLER_MAC address {address}"))
        })
        .collect::<Result<Vec<_>>>()?;

    let mut scan = BLEScan::new();
    scan.active_scan(true).interval(100).window(80);
    let saved_peer = EspDefaultNvs::new(nvs, "steam-bridge", true).context("Steam peer storage")?;
    let mut stored = [0; 6];
    let mut last_steam = match saved_peer.get_blob("peer", &mut stored) {
        Ok(Some(bytes)) => <[u8; 6]>::try_from(bytes)
            .ok()
            .map(|addr| BLEAddress::from_le_bytes(addr, esp32_nimble::BLEAddressType::Public)),
        Ok(None) => None,
        Err(error) => {
            log::warn!("Saved Steam Controller identity could not be read: {error}");
            None
        }
    };

    loop {
        let candidate = block_on(scan.start(device, 3000, |peer, data| {
            let address = peer.addr();
            if !allowed.is_empty() {
                return allowed.contains(&address).then_some(address);
            }
            let named = data.name().is_some_and(|name| is_steam_name(name.as_ref()));
            (last_steam == Some(address) || named || data.is_advertising_service(&VALVE))
                .then_some(address)
        }));
        let address = match candidate {
            Ok(Some(address)) => address,
            Ok(None) => continue,
            Err(error) => {
                log::warn!("Steam Controller scan failed: {error}");
                thread::sleep(Duration::from_millis(500));
                continue;
            }
        };

        let mut client = device.new_client();
        let disconnect_state = Arc::clone(&state);
        let disconnected = Arc::new(Notification::new());
        let link_notice = Arc::clone(&disconnected);
        client.on_disconnect(move |_| {
            disconnect_state.steam_disconnected();
            link_notice.notify_lsb();
        });
        let connected = block_on(async {
            let opened = open_link(&mut client, &address, &disconnected).await;
            disconnected.reset();
            opened?;
            attach(&mut client, &state).await
        });

        match connected {
            Ok(mut controller) => {
                let identity = client
                    .desc()
                    .ok()
                    .map(|link| link.id_address())
                    .unwrap_or(address);
                if last_steam != Some(identity) {
                    if let Err(error) = saved_peer.set_blob("peer", &identity.as_le_bytes()) {
                        log::warn!("Steam Controller identity could not be saved: {error}");
                    }
                    last_steam = Some(identity);
                }
                if let Err(error) = service(&mut client, &state, &mut controller) {
                    log::warn!("Steam Controller stopped: {error:#}");
                }
            }
            Err(error) => log::warn!("Steam Controller setup failed: {error:#}"),
        }

        while client.connected() {
            if let Err(error) = client.disconnect() {
                log::debug!("Steam Controller disconnect pending: {error}");
            }
            thread::sleep(Duration::from_millis(100));
        }
        state.steam_disconnected();
        thread::sleep(Duration::from_millis(250));
    }
}
