use controller_core::input::{Battery, Input, Mapper, decode};

#[test]
fn face_buttons_follow_the_printed_labels() {
    let mut mapper = Mapper::default();
    for (index, mask) in [0x02, 0x01, 0x08, 0x04].into_iter().enumerate() {
        let input = Input {
            buttons: 1 << index,
            ..Input::default()
        };
        assert_eq!(mapper.encode(&input, Battery::default(), 0)[2], mask);
    }
}

fn axis(report: &[u8; 63], offset: usize) -> u16 {
    u16::from(report[offset]) | (u16::from(report[offset + 1] & 15) << 8)
}

fn packet(right: i16, touched: bool, format: u8) -> Input {
    let mut payload = [0; 45];
    payload[1] = 1; // A
    if touched {
        payload[3] = 0x20;
    }
    let offset = if format == 0x47 { 25 } else { 23 };
    payload[offset..offset + 2].copy_from_slice(&right.to_le_bytes());
    decode(format, &payload).unwrap()
}

#[test]
fn gestures_start_and_end_at_neutral() {
    let mut mapper = Mapper::default();
    for format in [0x45, 0x47] {
        assert_eq!(
            axis(
                &mapper.encode(&packet(8000, true, format), Battery::default(), 100_000),
                8
            ),
            2048
        );
        let moved = mapper.encode(&packet(8900, true, format), Battery::default(), 120_000);
        assert!(axis(&moved, 8) > 2048);
        assert_eq!(moved[2], 2);
        assert_eq!(
            axis(
                &mapper.encode(&packet(8900, true, format), Battery::default(), 140_000),
                8
            ),
            2048
        );
        assert_eq!(
            axis(
                &mapper.encode(&packet(8900, false, format), Battery::default(), 160_000),
                8
            ),
            2048
        );
    }
}

#[test]
fn physical_right_stick_wins_and_unknown_battery_is_healthy() {
    let mut mapper = Mapper::default();
    let mut input = packet(0, true, 0x45);
    mapper.encode(&input, Battery::default(), 0);
    input.right_stick.x = -32768;
    input.right_pad.position.x = 30_000;
    let report = mapper.encode(&input, Battery::default(), 15_000);
    assert_eq!(axis(&report, 8), 0);
    assert_eq!(report[1] >> 2, 9);
    assert!(decode(0x45, &[0; 16]).is_none());
}
