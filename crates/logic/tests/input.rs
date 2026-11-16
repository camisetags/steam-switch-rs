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

fn gyro_packet(format: u8, timestamp: u16, buttons: u32, gyro: [i16; 3]) -> Input {
    let mut payload = [0; 45];
    payload[1..5].copy_from_slice(&buttons.to_le_bytes());
    let time_offset = if format == 0x47 { 31 } else { 29 };
    payload[time_offset..time_offset + 2].copy_from_slice(&timestamp.to_le_bytes());
    payload[37..39].copy_from_slice(&16_384i16.to_le_bytes());
    for (bytes, value) in payload[39..45].chunks_exact_mut(2).zip(gyro) {
        bytes.copy_from_slice(&value.to_le_bytes());
    }
    decode(format, &payload).unwrap()
}

fn right_stick(report: &[u8; 63]) -> [u16; 2] {
    [
        axis(report, 8),
        u16::from(report[9] >> 4) | (u16::from(report[10]) << 4),
    ]
}

#[test]
fn l4_routes_controller_rotation_to_the_regular_right_stick() {
    let battery = Battery::default();
    let l4 = 1 << 17;
    for format in [0x45, 0x47] {
        for (gyro, (x_ranges, y_ranges)) in [
            ([0, 0, -1200], ((2300..=4095), (2048..=2048))),
            ([0, 0, 1200], ((0..=1800), (2048..=2048))),
            ([1200, 0, 0], ((2048..=2048), (2300..=4095))),
            ([-1200, 0, 0], ((2048..=2048), (0..=1800))),
            ([0, 1200, 0], ((2048..=2048), (2048..=2048))),
        ] {
            let mut mapper = Mapper::default();
            let input = gyro_packet(format, 1, l4, gyro);
            let mut report = mapper.encode(&input, battery, 10_000);
            mapper.apply_aim(&mut report, 25_000);
            let [x, y] = right_stick(&report);
            assert!(
                x_ranges.contains(&x),
                "format {format:#x} gyro {gyro:?}: x={x}"
            );
            assert!(
                y_ranges.contains(&y),
                "format {format:#x} gyro {gyro:?}: y={y}"
            );
            assert_eq!(&report[5..8], &[0, 8, 128]);
        }
    }
}

#[test]
fn gyro_aim_returns_to_center_on_release_and_stale_sensor_data() {
    let mut mapper = Mapper::default();
    let battery = Battery::default();
    let l4 = 1 << 17;
    let rotation = [0, 0, -1800];
    let moving = gyro_packet(0x45, 15_000, l4, rotation);
    let mut report = mapper.encode(&moving, battery, 15_000);
    mapper.apply_aim(&mut report, 30_000);
    assert!(right_stick(&report)[0] > 2300);

    let mut stale = mapper.encode(&moving, battery, 110_000);
    mapper.apply_aim(&mut stale, 120_000);
    assert_eq!(right_stick(&stale), [2048; 2]);

    let moving = gyro_packet(0x45, 30_000, l4, rotation);
    let mut report = mapper.encode(&moving, battery, 125_000);
    mapper.apply_aim(&mut report, 140_000);
    assert!(right_stick(&report)[0] > 2300);
    let released = gyro_packet(0x45, 45_000, 0, rotation);
    let mut report = mapper.encode(&released, battery, 145_000);
    mapper.apply_aim(&mut report, 160_000);
    assert_eq!(right_stick(&report), [2048; 2]);
}

#[test]
fn gyro_aim_ignores_sensor_noise_and_keeps_physical_stick_priority() {
    let battery = Battery::default();
    let mut mapper = Mapper::default();
    let mut input = gyro_packet(0x45, 15_000, 1 << 17, [24, 0, -24]);
    let mut report = mapper.encode(&input, battery, 15_000);
    mapper.apply_aim(&mut report, 30_000);
    assert_eq!(right_stick(&report), [2048; 2]);

    input = gyro_packet(0x45, 30_000, (1 << 17) | (1 << 7), [0, 0, -1800]);
    input.right_stick.x = i16::MIN;
    let mut report = mapper.encode(&input, battery, 30_000);
    mapper.apply_aim(&mut report, 45_000);
    assert_eq!(right_stick(&report), [1229, 2048]);
}

#[test]
fn gyro_keeps_the_measured_rotation_through_a_short_gap() {
    let mut mapper = Mapper::default();
    let battery = Battery::default();
    let l4 = 1 << 17;
    let mut report = [0; 63];
    for time in [15_000, 30_000, 45_000, 60_000, 75_000] {
        let input = gyro_packet(0x45, time as u16, l4, [0, 0, -1200]);
        report = mapper.encode(&input, battery, time);
    }
    mapper.apply_aim(&mut report, 90_000);
    let steady = axis(&report, 8);
    let duplicate = gyro_packet(0x45, 75_000u32 as u16, l4, [0, 0, -1200]);
    report = mapper.encode(&duplicate, battery, 120_000);
    assert!(steady > 2300);
    assert!(axis(&report, 8).abs_diff(steady) <= 30);
    mapper.apply_aim(&mut report, 145_000);
    assert!((2049..steady).contains(&axis(&report, 8)));
    mapper.apply_aim(&mut report, 175_000);
    assert_eq!(right_stick(&report), [2048; 2]);
}

#[test]
fn gyro_drops_the_prediction_immediately_when_rotation_reverses() {
    let mut mapper = Mapper::default();
    let battery = Battery::default();
    let l4 = 1 << 17;
    let right = gyro_packet(0x45, 15_000, l4, [0, 0, -2500]);
    let mut report = mapper.encode(&right, battery, 15_000);
    mapper.apply_aim(&mut report, 50_000);
    assert!(axis(&report, 8) > 2200);

    let left = gyro_packet(0x45, 55_000, l4, [0, 0, 2500]);
    let mut report = mapper.encode(&left, battery, 55_000);
    assert_eq!(axis(&report, 8), 2048);
    mapper.apply_aim(&mut report, 56_000);
    assert!(axis(&report, 8) < 2048);
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
fn right_pad_keeps_steady_speed_across_a_short_gap_and_resumes_without_a_jump() {
    let battery = Battery::default();
    for format in [0x45, 0x47] {
        let mut mapper = Mapper::default();
        let mut report = [0; 63];
        for i in 0..=8 {
            let time = 100_000 + u64::try_from(i).unwrap() * 15_000;
            report = mapper.encode(&packet(i * 600, true, format), battery, time);
        }
        let steady = axis(&report, 8);
        mapper.apply_aim(&mut report, 265_000);
        assert!((2480..=2500).contains(&steady));
        assert!(axis(&report, 8).abs_diff(steady) <= 8);

        // Two samples were lost. The latest position spans all three intervals.
        let mut resumed = mapper.encode(&packet(6600, true, format), battery, 265_000);
        assert_eq!(axis(&resumed, 8), axis(&report, 8));
        mapper.apply_aim(&mut resumed, 280_000);
        assert!(axis(&resumed, 8).abs_diff(steady) <= 8);
    }
}

#[test]
fn right_pad_prediction_lets_a_direction_change_or_release_take_over() {
    let mut mapper = Mapper::default();
    let battery = Battery::default();
    mapper.encode(&packet(0, true, 0x45), battery, 100_000);
    let mut report = mapper.encode(&packet(2000, true, 0x45), battery, 115_000);
    mapper.apply_aim(&mut report, 150_000);
    assert!(axis(&report, 8) > 2450);

    let mut report = mapper.encode(&packet(0, true, 0x45), battery, 150_000);
    assert_eq!(axis(&report, 8), 2048);
    mapper.apply_aim(&mut report, 151_000);
    assert!(axis(&report, 8) < 2048);
    let released = mapper.encode(&packet(0, false, 0x45), battery, 152_000);
    assert_eq!(right_stick(&released), [2048; 2]);
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
        let before_recovery = axis(&report, 8);
        recovering.apply_aim(&mut report, resumed_at + 20_000);
        let first_step = axis(&report, 8) - before_recovery;
        assert!(first_step > 0 && first_step * 3 < normal_step * 2);
        recovering.apply_aim(&mut report, resumed_at + 30_000);
        assert!(axis(&report, 8) - before_recovery > first_step);
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

fn replay_swipe(arrivals: &[u64], step: i16, report_times: &[u64]) -> Vec<u16> {
    let mut mapper = Mapper::default();
    let battery = Battery::default();
    let mut report = [0; 63];
    let mut next = 0;
    report_times
        .iter()
        .map(|&time| {
            while let Some(&arrival) = arrivals.get(next).filter(|&&arrival| arrival <= time) {
                let input = packet(next as i16 * step, true, 0x45);
                report = mapper.encode(&input, battery, arrival);
                next += 1;
            }
            mapper.apply_aim(&mut report, time);
            axis(&report, 8)
        })
        .collect()
}

#[test]
fn slow_swipes_have_the_same_deflection_at_different_sample_cadences() {
    let deflections = [7_500, 10_000, 15_000].map(|interval| {
        let arrivals: Vec<_> = (0..=150_000).step_by(interval).collect();
        let step = (interval / 500) as i16; // 2,000 counts per second
        replay_swipe(&arrivals, step, &[150_000])[0]
    });
    println!("Slow swipe, x at 7.5/10/15 ms: {deflections:?}");
    assert!((2052..2100).contains(&deflections[0]));
    assert!(deflections.iter().all(|&x| x.abs_diff(deflections[0]) <= 2));
}

#[test]
fn delayed_batches_preserve_steady_swipe_without_a_rebound() {
    let regular = [
        0, 15_000, 30_000, 45_000, 60_000, 75_000, 90_000, 105_000, 120_000,
    ];
    let bunched = [
        0, 15_000, 30_000, 75_000, 77_000, 79_000, 90_000, 105_000, 120_000,
    ];
    let times = [
        60_000, 75_000, 77_000, 79_000, 85_000, 90_000, 100_000, 115_000,
    ];
    let reference = replay_swipe(&regular, 1500, &times);
    let resumed = replay_swipe(&bunched, 1500, &times);
    let maximum_error = resumed
        .iter()
        .zip(reference)
        .map(|(&a, b)| a.abs_diff(b))
        .max()
        .unwrap();
    println!("Batch replay, largest difference from regular cadence: {maximum_error} stick counts");
    assert!(maximum_error <= 8);
}

#[test]
fn fast_diagonal_flick_keeps_its_strength_and_stops_on_release() {
    let mut mapper = Mapper::default();
    let battery = Battery::default();
    let mut input = packet(0, true, 0x45);
    mapper.encode(&input, battery, 100_000);
    input.right_pad.position = Stick { x: 9000, y: 9000 };
    let mut report = mapper.encode(&input, battery, 115_000);
    mapper.apply_aim(&mut report, 130_000);
    let [x, y] = right_stick(&report);
    assert_eq!(x, y);
    assert!(x >= 3150);
    input.right_pad.touched = false;
    assert_eq!(
        right_stick(&mapper.encode(&input, battery, 131_000)),
        [2048; 2]
    );
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
