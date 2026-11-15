use controller_core::input::{Battery, Input, Mapper, Pad, PadEffect, PadFeedback, Stick, decode};

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
fn left_pad_uses_cardinal_dpad_buttons_and_leaves_the_stick_separate() {
    let mut mapper = Mapper::default();
    let battery = Battery::default();
    for (x, y, expected) in [
        (4_000, 4_000, 0),
        (20_000, 4_000, 0x02),
        (-20_000, 4_000, 0x04),
        (4_000, 20_000, 0x08),
        (4_000, -20_000, 0x01),
    ] {
        let mut input = Input {
            left_pad: Pad {
                position: Stick { x, y },
                touched: true,
            },
            ..Input::default()
        };
        let report = mapper.encode(&input, battery, 0);
        assert_eq!(report[3] & 0x0f, expected);
        assert_eq!(axis(&report, 5), 2048);
        input.left_stick.x = 32_767;
        let report = mapper.encode(&input, battery, 0);
        assert_eq!(report[3] & 0x0f, expected);
        assert_eq!(axis(&report, 5), 4095);

        input.buttons = 1 << 10; // Physical Down
        assert_eq!(mapper.encode(&input, battery, 0)[3] & 0x0f, 0x01);
        input.buttons = 0;
        input.left_pad.touched = false;
        assert_eq!(mapper.encode(&input, battery, 0)[3] & 0x0f, 0);
    }
}

#[test]
fn right_stick_precision_uses_the_stricter_held_paddle() {
    let mut mapper = Mapper::default();
    let mut payload = [0; 45];
    payload[9..11].copy_from_slice(&i16::MAX.to_le_bytes());
    payload[11..13].copy_from_slice(&i16::MIN.to_le_bytes());
    payload[13..15].copy_from_slice(&i16::MAX.to_le_bytes());
    payload[15..17].copy_from_slice(&i16::MIN.to_le_bytes());
    const NORMAL: [u8; 3] = [0xff, 0x0f, 0x00];
    const HALF: [u8; 3] = [0xff, 0x0b, 0x40];
    const FORTY_PERCENT: [u8; 3] = [0x33, 0xdb, 0x4c];
    for (paddles, expected) in [
        (0, NORMAL),
        (1u32 << 7, FORTY_PERCENT), // R4
        (1 << 17, HALF),            // L4
        ((1 << 7) | (1 << 17), FORTY_PERCENT),
        (1 << 17, HALF),
        ((1 << 8) | (1 << 18), NORMAL), // R5 and L5
        (0, NORMAL),
    ] {
        payload[1..5].copy_from_slice(&(paddles | (1 << 13)).to_le_bytes());
        let input = decode(0x45, &payload).unwrap();
        let report = mapper.encode(&input, Battery::default(), 0);
        assert_eq!(report[8..11], expected);
        assert_eq!(report[5..8], NORMAL);
        assert_eq!(&report[2..5], &[0, 0x08, 0]); // D-pad Up
    }
}

#[test]
fn precision_paddles_leave_trackpad_aim_at_full_sensitivity() {
    let aim_after_swipe = |buttons| {
        let mut mapper = Mapper::default();
        let battery = Battery::default();
        let mut input = packet(0, true, 0x45);
        input.buttons |= buttons;
        mapper.encode(&input, battery, 100_000);
        input.right_pad.position.x = 1_500;
        let mut report = mapper.encode(&input, battery, 115_000);
        mapper.apply_aim(&mut report, 125_000);
        axis(&report, 8)
    };
    let normal = aim_after_swipe(0);
    assert!(normal > 2048);
    assert_eq!(aim_after_swipe((1 << 7) | (1 << 17)), normal);
}

#[test]
fn back_paddles_do_not_wake_the_console() {
    let mut input = Input {
        buttons: (1 << 8) | (1 << 18), // R5 and L5
        ..Input::default()
    };
    assert_eq!(input.wake_buttons(), 0);

    input.buttons |= (1 << 7) | (1 << 17); // R4 and L4
    assert_eq!(input.wake_buttons(), 0);

    input.buttons |= 1; // A
    assert_eq!(input.wake_buttons(), 1);
}

#[test]
fn only_l4_enables_motion_aim() {
    let mut payload = [0; 45];
    for (buttons, enabled) in [
        (0u32, false),
        (1 << 7, false),
        (1 << 17, true),
        (1 << 18, false),
    ] {
        payload[1..5].copy_from_slice(&buttons.to_le_bytes());
        assert_eq!(
            decode(0x45, &payload).unwrap().motion_aim_enabled(),
            enabled
        );
    }
}

#[test]
fn aim_ramps_between_packets_and_settles_when_the_finger_stops() {
    let mut mapper = Mapper::default();
    let battery = Battery::default();
    for format in [0x45, 0x47] {
        mapper.encode(&packet(0, false, format), battery, 90_000);
        assert_eq!(
            axis(
                &mapper.encode(&packet(8000, true, format), battery, 100_000),
                8
            ),
            2048
        );
        let mut moved = mapper.encode(&packet(8900, true, format), battery, 120_000);
        let at_sample = axis(&moved, 8);
        mapper.apply_aim(&mut moved, 125_000);
        let early = axis(&moved, 8);
        mapper.apply_aim(&mut moved, 135_000);
        assert!(at_sample < early && early < axis(&moved, 8));
        assert_eq!(moved[2], 2);

        let mut stopped = mapper.encode(&packet(8900, true, format), battery, 140_000);
        let at_stop = axis(&stopped, 8);
        assert!(at_stop > 2048);
        mapper.apply_aim(&mut stopped, 155_000);
        assert!((2049..at_stop).contains(&axis(&stopped, 8)));
        mapper.apply_aim(&mut stopped, 240_000);
        assert_eq!(axis(&stopped, 8), 2048);
        assert_eq!(
            axis(
                &mapper.encode(&packet(8900, false, format), battery, 245_000),
                8
            ),
            2048
        );
    }
}

#[test]
fn right_pad_keeps_the_steady_gain_and_clears_after_release_or_a_packet_gap() {
    let mut mapper = Mapper::default();
    let battery = Battery::default();
    mapper.encode(&packet(8_000, true, 0x45), battery, 100_000);
    let mut report = [0; 63];
    for i in 1..=12 {
        report = mapper.encode(
            &packet(8_000 + i * 450, true, 0x45),
            battery,
            100_000 + u64::try_from(i).unwrap() * 10_000,
        );
    }
    // 450 counts in 10 ms gives the previous firmware's 22.65% deflection.
    assert!((2510..=2512).contains(&axis(&report, 8)));
    mapper.apply_aim(&mut report, 320_000);
    assert!((2048..=2049).contains(&axis(&report, 8)));
    let fresh = mapper.encode(&packet(-20_000, true, 0x45), battery, 340_000);
    assert_eq!(axis(&fresh, 8), 2048);

    let mut moving = mapper.encode(&packet(-18_000, true, 0x45), battery, 350_000);
    mapper.apply_aim(&mut moving, 365_000);
    assert!(axis(&moving, 8) > 2048);
    let released = mapper.encode(&packet(-18_000, false, 0x45), battery, 370_000);
    assert_eq!(axis(&released, 8), 2048);
}

#[test]
fn aim_resumes_gradually_after_a_bluetooth_gap() {
    let battery = Battery::default();
    let mut normal = Mapper::default();
    normal.encode(&packet(0, true, 0x45), battery, 100_000);
    let mut moving = normal.encode(&packet(1_500, true, 0x45), battery, 115_000);
    normal.apply_aim(&mut moving, 120_000);
    let normal_step = axis(&moving, 8) - 2048;

    for gap in [45_000, 105_000] {
        let mut recovering = Mapper::default();
        recovering.encode(&packet(0, true, 0x45), battery, 100_000);
        recovering.encode(&packet(1_500, true, 0x45), battery, 115_000);
        let resumed_at = 115_000 + gap;
        let mut report = recovering.encode(&packet(1_500, true, 0x45), battery, resumed_at);
        if gap == 105_000 {
            assert_eq!(axis(&report, 8), 2048);
        }
        report = recovering.encode(&packet(3_000, true, 0x45), battery, resumed_at + 15_000);
        recovering.apply_aim(&mut report, resumed_at + 20_000);
        let first_step = axis(&report, 8) - 2048;
        assert!(first_step > 0 && first_step * 3 < normal_step * 2);
        recovering.apply_aim(&mut report, resumed_at + 30_000);
        assert!(axis(&report, 8) - 2048 > first_step);
    }
}

#[test]
fn aim_uses_the_recent_sample_cadence_for_back_to_back_packets() {
    let render_step = |sample_spacing: u64| {
        let battery = Battery::default();
        let mut mapper = Mapper::default();
        mapper.encode(&packet(0, true, 0x45), battery, 100_000);
        mapper.encode(&packet(0, true, 0x45), battery, 110_000);
        let sample_time = 110_000 + sample_spacing;
        let mut report = mapper.encode(&packet(1_500, true, 0x45), battery, sample_time);
        mapper.apply_aim(&mut report, sample_time + 5_000);
        axis(&report, 8)
    };
    let normal = render_step(10_000);
    assert!(normal > 2048);
    assert_eq!(render_step(500), normal);
    assert_eq!(render_step(0), normal);
}

#[test]
fn physical_right_stick_wins_and_unknown_battery_is_healthy() {
    let mut mapper = Mapper::default();
    let mut input = packet(0, true, 0x45);
    mapper.encode(&input, Battery::default(), 0);
    input.right_stick.x = -32768;
    input.right_pad.position.x = 30_000;
    let mut report = mapper.encode(&input, Battery::default(), 15_000);
    mapper.apply_aim(&mut report, 25_000);
    assert_eq!(axis(&report, 8), 0);
    assert_eq!(report[1] >> 2, 9);
    assert!(decode(0x45, &[0; 16]).is_none());
}

#[test]
fn feedback_tracks_left_directions_right_swipes_and_physical_pad_clicks() {
    let mut feedback = PadFeedback::default();
    let mut input = Input {
        left_pad: Pad {
            position: Stick { x: 12_000, y: 0 },
            touched: true,
        },
        right_pad: Pad {
            position: Stick { x: 0, y: 0 },
            touched: true,
        },
        ..Input::default()
    };
    feedback.update(&input, 0);
    assert_eq!(feedback.take(0), [Some(PadEffect::Tick), None]);
    feedback.update(&input, 10_000);
    assert_eq!(feedback.take(10_000), [None, None]);

    input.left_pad.position.x = -12_000;
    input.right_pad.position.x = 20;
    feedback.update(&input, 20_000);
    assert_eq!(feedback.take(20_000), [Some(PadEffect::Tick), None]);
    input.right_pad.position.x = 1_200;
    feedback.update(&input, 30_000);
    assert_eq!(feedback.take(30_000), [None, Some(PadEffect::Tick)]);
    input.right_pad.position.x = 2_500;
    feedback.update(&input, 40_000);
    assert_eq!(feedback.take(40_000), [None, None]);

    input.buttons = (1 << 22) | (1 << 26);
    feedback.update(&input, 50_000);
    assert_eq!(
        feedback.take(50_000),
        [Some(PadEffect::Click), Some(PadEffect::Click)]
    );
    feedback.update(&input, 60_000);
    assert_eq!(feedback.take(60_000), [None, None]);
    input.right_pad.touched = false;
    feedback.update(&input, 120_000);
    input.right_pad.touched = true;
    input.right_pad.position.x = -15_000;
    feedback.update(&input, 130_000);
    assert_eq!(feedback.take(130_000), [None, None]);
}

#[test]
fn feedback_drops_stale_events_instead_of_playing_them_late() {
    let mut feedback = PadFeedback::default();
    feedback.update(
        &Input {
            buttons: 1 << 22,
            ..Input::default()
        },
        0,
    );
    assert_eq!(feedback.take(80_000), [None, None]);
}
