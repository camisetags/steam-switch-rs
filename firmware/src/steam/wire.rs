// Protocol references: JoypadOS's Steam Controller 2 BLE driver (Apache-2.0)
// and SDL's Steam Triton HID driver (Zlib).
// https://github.com/joypad-ai/joypad-os/blob/main/src/bt/btstack/btstack_host.c
// https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_steam_triton.c

use controller_core::input::PadEffect;

const SET_SETTINGS: u8 = 0x87;

const fn setting(id: u8, value: u16) -> [u8; 5] {
    let [lo, hi] = value.to_le_bytes();
    [SET_SETTINGS, 3, id, lo, hi]
}

pub(super) const LIZARD_OFF: [u8; 5] = setting(9, 0);
pub(super) const MOTION_ON: [u8; 5] = setting(0x30, 0x08 | 0x10);
pub(super) const FAST_INTERVAL_MIN: u16 = 6;
pub(super) const FAST_INTERVAL_MAX: u16 = 12;

pub(super) fn faster_interval(current: u16, attempt: usize) -> Option<u16> {
    [FAST_INTERVAL_MIN, 8, FAST_INTERVAL_MAX]
        .get(attempt)
        .copied()
        .filter(|&target| target < current)
}

pub(super) fn rumble(left: u16, right: u16) -> [u8; 9] {
    let mut packet = [0; 9];
    packet[3..5].copy_from_slice(&left.to_le_bytes());
    packet[6..8].copy_from_slice(&right.to_le_bytes());
    packet
}

// Triton output report 0x82 (MsgHapticCommand). BLE omits the report ID.
// Actuator indices were verified on hardware by SteamlessController:
// https://github.com/ddeverill/SteamlessController/blob/main/src/steam/SteamController.cpp
pub(super) fn pad_effect(right: bool, effect: PadEffect) -> [u8; 3] {
    let (command, gain): (u8, i8) = match effect {
        PadEffect::Tick => (1, -16),
        PadEffect::Click => (2, -9),
    };
    [u8::from(right), command, gain as u8]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_independent_haptic_motors() {
        assert_eq!(
            rumble(0x1234, 0xabcd),
            [0, 0, 0, 0x34, 0x12, 0, 0xcd, 0xab, 0]
        );
        assert_eq!(rumble(0, 0), [0; 9]);
    }

    #[test]
    fn configures_native_gamepad_and_raw_motion() {
        assert_eq!(LIZARD_OFF, [0x87, 3, 9, 0, 0]);
        assert_eq!(MOTION_ON, [0x87, 3, 0x30, 0x18, 0]);
    }

    #[test]
    fn addresses_discrete_effects_to_the_trackpads() {
        assert_eq!(pad_effect(false, PadEffect::Tick), [0, 1, 0xf0]);
        assert_eq!(pad_effect(true, PadEffect::Click), [1, 2, 0xf7]);
    }

    #[test]
    fn interval_fallbacks_only_request_a_faster_link() {
        assert_eq!(faster_interval(36, 0), Some(6));
        assert_eq!(faster_interval(36, 1), Some(8));
        assert_eq!(faster_interval(36, 2), Some(12));
        assert_eq!(faster_interval(36, 3), None);
        assert_eq!(faster_interval(8, 1), None);
        assert_eq!(faster_interval(6, 0), None);
    }
}
