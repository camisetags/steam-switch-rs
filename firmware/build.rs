fn main() {
    embuild::espidf::sysenv::output();
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("espidf") {
        println!("cargo:rustc-link-arg-bin=steam-switch-firmware=-Wl,--wrap=esp_wifi_init");
    }
}
