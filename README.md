# Steam Switch RS

A Rust firmware project for an ESP32-S3 that connects a Steam Controller (2026) to a Nintendo Switch 2 over Bluetooth LE. The controller is the input device and the ESP32 is the adapter.

- The physical A, B, X and Y buttons follow the printed letters.
- Hold R4 for 40% of the physical right stick's range or L4 for 50%. Holding both uses 40%. Release both to restore full range. L5 and R5 are disabled.
- Touching the left trackpad away from its center presses one of the four D-pad directions. The right trackpad steers the camera while the finger moves, smoothing the transitions in stick deflection. The physical D-pad and right stick take priority over their trackpad counterparts.
- The right trackpad eases back into motion after a gap in Bluetooth input and uses its recent sample cadence when reports arrive together.
- Trackpad feedback uses short, local ticks for D-pad directions and camera swipes, and a stronger pulse for a physical trackpad click.
- The firmware implements console input reports, motion, battery level, rumble, pairing, and reconnect. The local web editor records, edits, saves and plays controller macros.

## Read the Rust code

| Module | What it does |
| --- | --- |
| [`input`](crates/logic/src/input.rs) | Decode Steam BLE input and translate it to a Switch report. |
| [`motion`](crates/logic/src/motion.rs) | Combine gyroscope and accelerometer readings and encode the motion payload. |
| [`macros`](crates/logic/src/macros.rs) | Validate macro data and manage playback and recording. |
| [`runtime`](firmware/src/runtime.rs) | Connect the radio, input pipeline and macro engine. |
| [`steam`](firmware/src/steam.rs) | Discover and connect to the Steam Controller. |
| [`console`](firmware/src/console.rs) | Advertise and connect to the Switch 2. |
| [`macros_web`](firmware/src/macros_web.rs) | Host the macro editor and store macros on the board. |

## Test on a computer

Install stable Rust, then run `cargo test --workspace --locked` from this directory. The input tests use synthetic packets for both controller report formats. Other tests cover console pairing messages, rumble packet formats, motion timestamp rollover and macro playback.

## Build for ESP32-S3

Install [`espup`](https://github.com/esp-rs/espup) and [`ldproxy`](https://crates.io/crates/ldproxy). The firmware uses ESP-IDF 6.1.1 and the Espressif Rust toolchain for Xtensa. See the [Espressif toolchain guide](https://docs.espressif.com/projects/rust/book/getting-started/toolchain.html) for system prerequisites.

```sh
cargo install espup --locked
cargo install ldproxy --locked
espup install --targets esp32s3
. "$HOME/export-esp.sh"
cd firmware
cargo +esp build --release --locked
```

To flash from a system that can access the board's serial port, install
[`espflash`](https://github.com/esp-rs/espflash). From `firmware`, run the command
below. Replace `PORT` with the board's serial device. With a custom Cargo target
directory, use its `xtensa-esp32s3-espidf/release` subdirectory instead.

```sh
build=../target/xtensa-esp32s3-espidf/release
espflash flash --port PORT --monitor --flash-mode dio --flash-size 4mb \
  --flash-freq 80mhz --bootloader "$build/bootloader.bin" \
  --partition-table partitions.csv "$build/steam-switch-firmware"
```

Optional build-time variables are `STEAM_CONTROLLER_MAC` (an allowlist of Bluetooth addresses separated by spaces or commas), `STEAM_SWITCH_WIFI_SSID` and `STEAM_SWITCH_WIFI_PASSWORD` (8–63 bytes). The editor uses `SteamSwitch-RS` and an open local Wi-Fi network by default. Set a password if people nearby should be unable to join. After connecting to the access point, open [the editor](http://192.168.4.1/).

Build with `STEAM_SWITCH_AIM_DIAGNOSTICS=1` to log one timing summary per second over UART at 115200 baud while both Bluetooth links are active. The `rx` and `touch` counts and gaps describe Steam Controller samples. The `tx` count and gaps describe accepted NimBLE submissions to the Switch. `pool_busy` counts waits for buffer headroom, and `tx_errors` counts failed submissions. The reported stick step is the largest per-axis change between consecutive submissions in 12-bit stick counts.

Pair the Steam Controller in Bluetooth mode by holding **B + R1 + Steam** from power off. On the Switch 2, open **Controllers → Change Grip/Order**. Pairing and gameplay on physical hardware are separate from the PC tests.

## Sources and licensing

This repository has its own Rust implementation and history. The tested Puck2Switch2 firmware supplied the expected behaviors. Protocol references include [SDL](https://github.com/libsdl-org/SDL) and [JoypadOS](https://github.com/joypad-ai/joypad-os) for the controller, [espp](https://github.com/esp-cpp/espp) for the Switch peripheral, and [PicoSwitch2](https://github.com/notsosaelin/PicoSwitch2) for motion encoding. See [NOTICE](NOTICE) and the [Apache 2.0 license](LICENSE).
