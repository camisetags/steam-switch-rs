#[cfg(not(target_os = "espidf"))]
fn main() {
    eprintln!("Build from the firmware directory with cargo +esp build --release.");
}

#[cfg(target_os = "espidf")]
mod console;
#[cfg(target_os = "espidf")]
mod macros_web;
#[cfg(target_os = "espidf")]
mod runtime;
#[cfg(target_os = "espidf")]
mod steam;

#[cfg(target_os = "espidf")]
pub(crate) use runtime::State;

#[cfg(target_os = "espidf")]
fn main() -> anyhow::Result<()> {
    runtime::run()
}

#[cfg(all(test, not(target_os = "espidf")))]
#[path = "console/protocol.rs"]
#[allow(dead_code)]
mod console_protocol_tests;

#[cfg(all(test, not(target_os = "espidf")))]
#[path = "console/timing.rs"]
mod console_timing_tests;

#[cfg(all(test, not(target_os = "espidf")))]
#[path = "steam/wire.rs"]
mod steam_wire_tests;
