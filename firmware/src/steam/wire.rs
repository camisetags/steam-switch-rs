// Protocol references: JoypadOS's Steam Controller 2 BLE driver (Apache-2.0)
// and SDL's Steam Triton HID driver (Zlib).
// https://github.com/joypad-ai/joypad-os/blob/main/src/bt/btstack/btstack_host.c
// https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_steam_triton.c

const SET_SETTINGS: u8 = 0x87;

const fn setting(id: u8, value: u16) -> [u8; 5] {
    let [lo, hi] = value.to_le_bytes();
    [SET_SETTINGS, 3, id, lo, hi]
}

pub(super) const LIZARD_OFF: [u8; 5] = setting(9, 0);
pub(super) const MOTION_ON: [u8; 5] = setting(0x30, 0x08 | 0x10);

pub(super) fn rumble(left: u16, right: u16) -> [u8; 9] {
    let mut packet = [0; 9];
    packet[3..5].copy_from_slice(&left.to_le_bytes());
    packet[6..8].copy_from_slice(&right.to_le_bytes());
    packet
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
}
