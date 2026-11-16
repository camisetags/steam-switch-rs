//! Valve BLE reports and Nintendo Pro Controller input reports.
//! Wire layouts: SDL's Triton structures (zlib) and espp's Pro2InputReport (MIT).

use crate::motion::StickGyro;

pub const REPORT_LEN: usize = 63;
pub const HOME: u8 = 16;
const LEFT_TOUCH: u32 = 1 << 25;
const RIGHT_TOUCH: u32 = 1 << 21;
const R4: u32 = 1 << 7;
const L4: u32 = 1 << 17;
const LOWER_PADDLES: u32 = (1 << 8) | (1 << 18); // R5 and L5
const LEFT_PAD_CENTER: u32 = 8192;
pub(crate) const AIM_SMOOTHING_US: f32 = 10_000.;
const AIM_RECOVERY_SMOOTHING_US: f32 = 24_000.;
const AIM_RECOVERY_WINDOW_US: u64 = 60_000;
const AIM_RECOVERY_GAP_US: u64 = 35_000;
const AIM_PREDICTION_US: u64 = 45_000;
const AIM_RESET_US: u64 = 100_000;
const AIM_BURST_US: u64 = 5_000;
const AIM_NOMINAL_SAMPLE_US: u64 = 15_000;
const AIM_MAX_REGULAR_SAMPLE_US: u64 = 25_000;
const AIM_MIN_SPEED: f32 = 1_600.;
const AIM_FULL_LOW_SPEED_GAIN: f32 = 8_000.;
const PAD_CLICK_MASKS: [u32; 2] = [1 << 26, 1 << 22];

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stick {
    pub x: i16,
    pub y: i16,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Pad {
    pub position: Stick,
    pub touched: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Imu {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
    pub timestamp: u32,
    pub timestamp_bits: u8,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Input {
    pub buttons: u32,
    pub left_stick: Stick,
    pub right_stick: Stick,
    pub left_pad: Pad,
    pub right_pad: Pad,
    pub triggers: [u16; 2],
    pub imu: Option<Imu>,
}

impl Input {
    pub fn wake_buttons(&self) -> u32 {
        self.buttons & !(R4 | L4 | LOWER_PADDLES)
    }

    pub fn motion_aim_enabled(&self) -> bool {
        self.buttons & L4 != 0
    }

    fn pad_direction(&self) -> u8 {
        if !self.left_pad.touched || self.buttons & (0x0f << 10) != 0 {
            return 0;
        }
        let Stick { x, y } = self.left_pad.position;
        let [abs_x, abs_y] = [x, y].map(|axis| i32::from(axis).unsigned_abs());
        if abs_x.max(abs_y) < LEFT_PAD_CENTER {
            return 0;
        }
        if abs_x >= abs_y {
            if x > 0 { 0x02 } else { 0x04 }
        } else if y > 0 {
            0x08
        } else {
            0x01
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PadEffect {
    Tick,
    Click,
}

#[derive(Default)]
pub struct PadFeedback {
    left_direction: u8,
    right_anchor: Option<Stick>,
    right_next_tick: u64,
    previous_buttons: u32,
    pending: [Option<(PadEffect, u64)>; 2],
}

impl PadFeedback {
    pub fn update(&mut self, input: &Input, time: u64) {
        let direction = input.pad_direction();
        let mut effects = [None; 2];
        if direction != 0 && direction != self.left_direction {
            effects[0] = Some(PadEffect::Tick);
        }
        self.left_direction = direction;

        let point = input.right_pad.position;
        if input.right_pad.touched {
            let anchor = self.right_anchor.get_or_insert(point);
            let dx = i64::from(point.x) - i64::from(anchor.x);
            let dy = i64::from(point.y) - i64::from(anchor.y);
            if time >= self.right_next_tick && dx * dx + dy * dy >= 1_000 * 1_000 {
                effects[1] = Some(PadEffect::Tick);
                self.right_next_tick = time + 45_000;
                *anchor = point;
            }
        } else {
            self.right_anchor = None;
        }

        let pressed = input.buttons & !self.previous_buttons;
        self.previous_buttons = input.buttons;
        for (side, (effect, mask)) in effects.into_iter().zip(PAD_CLICK_MASKS).enumerate() {
            if pressed & mask != 0 {
                self.pending[side] = Some((PadEffect::Click, time));
            } else if let Some(effect) = effect
                && self.pending[side].is_none()
            {
                self.pending[side] = Some((effect, time));
            }
        }
    }

    pub fn take(&mut self, time: u64) -> [Option<PadEffect>; 2] {
        std::mem::take(&mut self.pending).map(|pending| {
            pending
                .filter(|&(_, then)| time.saturating_sub(then) < 70_000)
                .map(|(effect, _)| effect)
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Battery {
    percent: u8,
    charging: bool,
}

impl Default for Battery {
    fn default() -> Self {
        Self {
            percent: 100,
            charging: false,
        }
    }
}

impl Battery {
    pub fn decode(data: &[u8]) -> Option<Self> {
        let data = if data.len() >= 3 && data[0] == 0x43 {
            &data[1..]
        } else {
            data
        };
        let [state, percent, ..] = data else {
            return None;
        };
        if !(1..=4).contains(state) || *percent > 100 {
            return None;
        }
        Some(Self {
            percent: *percent,
            charging: *state == 2,
        })
    }

    pub fn encode(self) -> u8 {
        let level = match self.percent {
            0 => 0,
            n => ((u16::from(n) * 9 + 50) / 100).max(1) as u8,
        };
        (level << 2) | (u8::from(self.charging) << 1)
    }
}

/// BLE notifications omit the report ID. The caller supplies it from the characteristic.
pub fn decode(id: u8, payload: &[u8]) -> Option<Input> {
    let shift = match id {
        0x45 => 0,
        0x47 => 2,
        _ => return None,
    };
    if payload.len() < 17 {
        return None;
    }
    let unsigned = |i| u16::from_le_bytes([payload[i], payload[i + 1]]);
    let signed = |i| unsigned(i) as i16;
    let buttons = u32::from_le_bytes(payload[1..5].try_into().ok()?);
    let stick_at = |i| Stick {
        x: signed(i),
        y: signed(i + 2),
    };
    let (left_pad, right_pad) = if payload.len() >= 27 + shift {
        (
            Pad {
                position: stick_at(17 + shift),
                touched: buttons & LEFT_TOUCH != 0,
            },
            Pad {
                position: stick_at(23 + shift),
                touched: buttons & RIGHT_TOUCH != 0,
            },
        )
    } else {
        (Pad::default(), Pad::default())
    };
    let imu = (payload.len() >= 45).then(|| Imu {
        accel: [signed(33), signed(35), signed(37)],
        gyro: [signed(39), signed(41), signed(43)],
        timestamp: if id == 0x45 {
            u32::from_le_bytes([payload[29], payload[30], payload[31], payload[32]])
        } else {
            u32::from(unsigned(31))
        },
        timestamp_bits: if id == 0x45 { 32 } else { 16 },
    });
    Some(Input {
        buttons,
        left_stick: stick_at(9),
        right_stick: stick_at(13),
        left_pad,
        right_pad,
        triggers: [unsigned(5), unsigned(7)],
        imu,
    })
}

pub fn neutral(battery: Battery) -> [u8; REPORT_LEN] {
    let mut report = [0; REPORT_LEN];
    report[1] = battery.encode();
    report[11] = 0x30;
    pack(&mut report[5..8], [0.; 2]);
    pack(&mut report[8..11], [0.; 2]);
    report
}

fn axes(stick: Stick) -> [f32; 2] {
    [stick.x, stick.y].map(|axis| {
        let v = (f32::from(axis) / 32767.).max(-1.);
        if v.abs() < 0.06 { 0. } else { v }
    })
}

fn pack(dst: &mut [u8], axes: [f32; 2]) {
    let [x, y] = axes.map(|v| ((v.clamp(-1., 1.) + 1.) * 2047.5).round() as u16);
    dst.copy_from_slice(&[x as u8, ((x >> 8) | (y << 4)) as u8, (y >> 4) as u8]);
}

pub(crate) fn predict_aim(
    start: [f32; 2],
    target: [f32; 2],
    elapsed_us: u64,
    smoothing_us: f32,
) -> [f32; 2] {
    if start == [0.; 2] && target == [0.; 2] {
        return [0.; 2];
    }
    // Follow the last measured velocity through a short Bluetooth gap.
    // Fade toward neutral once the sample is too old to keep predicting.
    let rise = -(-(elapsed_us.min(AIM_PREDICTION_US) as f32) / smoothing_us).exp_m1();
    let decay = (-(elapsed_us.saturating_sub(AIM_PREDICTION_US) as f32) / AIM_SMOOTHING_US).exp();
    std::array::from_fn(|axis| (start[axis] + (target[axis] - start[axis]) * rise) * decay)
}

pub(crate) fn reconcile_aim(start: [f32; 2], target: [f32; 2]) -> [f32; 2] {
    // A measured reversal cancels the prediction on that axis immediately.
    std::array::from_fn(|axis| {
        if start[axis] * target[axis] < 0. {
            0.
        } else {
            start[axis]
        }
    })
}

#[derive(Default)]
pub struct Mapper {
    previous_aim: Option<(Stick, u64)>,
    aim_start: [f32; 2],
    aim_target: [f32; 2],
    aim_speed: f32,
    aim_recover_until: u64,
    aim_sample_interval: Option<u64>,
    right_stick_active: bool,
    stick_gyro: StickGyro,
}

impl Mapper {
    pub fn encode(&mut self, input: &Input, battery: Battery, now_us: u64) -> [u8; REPORT_LEN] {
        let mut report = neutral(battery);
        // Bit numbers in source order. Left/right sticks and button bytes are independent.
        const BUTTONS: [Option<(usize, u8)>; 30] = [
            Some((2, 2)),
            Some((2, 1)),
            Some((2, 8)),
            Some((2, 4)),
            Some((4, 2)),
            Some((2, 128)),
            Some((2, 64)),
            None, // R4: right-stick precision
            None,
            Some((2, 16)),
            Some((3, 1)),
            Some((3, 2)),
            Some((3, 4)),
            Some((3, 8)),
            Some((3, 64)),
            Some((3, 128)),
            Some((4, 1)),
            None, // L4: motion aim and right-stick precision
            None,
            Some((3, 16)),
            None,
            None,
            None,
            Some((2, 32)),
            None,
            None,
            None,
            Some((3, 32)),
            None,
            None,
        ];
        for (bit, mapping) in BUTTONS.into_iter().enumerate() {
            if let Some((offset, mask)) = mapping.filter(|_| input.buttons & (1 << bit) != 0) {
                report[offset] |= mask;
            }
        }
        report[3] |= input.pad_direction();
        pack(&mut report[5..8], axes(input.left_stick));
        let right = axes(input.right_stick);
        self.right_stick_active = right != [0.; 2];
        let gain = if input.buttons & R4 != 0 {
            0.4
        } else if input.buttons & L4 != 0 {
            0.5
        } else {
            1.
        };
        self.update_aim(input.right_pad, now_us);
        self.stick_gyro
            .update(input.imu, input.motion_aim_enabled(), now_us);
        pack(&mut report[8..11], right.map(|axis| axis * gain));
        self.apply_aim(&mut report, now_us);
        report
    }

    /// Refresh the virtual right stick at output time.
    pub fn apply_aim(&self, report: &mut [u8; REPORT_LEN], now_us: u64) {
        if !self.right_stick_active {
            let [x, y] = self.aim_at(now_us);
            let [yaw, pitch] = self.stick_gyro.at(now_us);
            pack(&mut report[8..11], [x + yaw, y + pitch]);
        }
    }

    fn aim_at(&self, now_us: u64) -> [f32; 2] {
        let Some((_, last_us)) = self.previous_aim else {
            return [0.; 2];
        };
        let elapsed = now_us.saturating_sub(last_us);
        if elapsed >= AIM_RESET_US {
            return [0.; 2];
        }
        let smoothing_us = if last_us < self.aim_recover_until && self.aim_target != [0.; 2] {
            AIM_RECOVERY_SMOOTHING_US
        } else {
            AIM_SMOOTHING_US
        };
        predict_aim(self.aim_start, self.aim_target, elapsed, smoothing_us)
    }

    fn update_aim(&mut self, pad: Pad, now_us: u64) {
        let start = self.aim_at(now_us);
        let current = pad.touched.then_some((pad.position, now_us));
        let previous = std::mem::replace(&mut self.previous_aim, current);
        let last_target = self.aim_target;
        let last_speed = std::mem::take(&mut self.aim_speed);
        self.aim_start = [0.; 2];
        self.aim_target = [0.; 2];
        let Some(((point, now), (old, then))) = current.zip(previous) else {
            self.aim_recover_until = 0;
            return;
        };
        let observed_us = now.saturating_sub(then);
        if observed_us >= AIM_RECOVERY_GAP_US {
            self.aim_recover_until = now + AIM_RECOVERY_WINDOW_US;
        }
        if observed_us >= AIM_RESET_US {
            return;
        }
        let recovering = now < self.aim_recover_until;
        if !recovering && (AIM_BURST_US..=AIM_MAX_REGULAR_SAMPLE_US).contains(&observed_us) {
            let previous = self.aim_sample_interval.unwrap_or(observed_us);
            self.aim_sample_interval = Some((3 * previous + observed_us) / 4);
        }
        self.aim_start = start;
        let dx = f32::from(point.x) - f32::from(old.x);
        let dy = f32::from(point.y) - f32::from(old.y);
        let distance = dx.hypot(dy);
        if distance == 0. {
            return;
        }
        let cadence = self.aim_sample_interval.unwrap_or(AIM_NOMINAL_SAMPLE_US);
        let expected_step = last_speed * cadence as f32 / 1_000_000.;
        // The first packet after a gap may have been sampled earlier. Infer
        // one sample interval only when its direction and step match the swipe.
        let likely_delayed_sample = (AIM_RECOVERY_GAP_US..=AIM_PREDICTION_US)
            .contains(&observed_us)
            && last_target[0] * dx + last_target[1] * dy > 0.
            && (expected_step * 0.75..=expected_step * 1.25).contains(&distance);
        let sample_us = if observed_us < AIM_BURST_US
            || (recovering && observed_us < cadence)
            || likely_delayed_sample
        {
            cadence
        } else {
            observed_us
        };
        let speed = distance * 1_000_000. / sample_us as f32;
        self.aim_speed = speed;
        // Fade in the existing low-speed gain based on speed, so the same
        // motion has the same threshold at different report intervals.
        let ramp =
            ((speed - AIM_MIN_SPEED) / (AIM_FULL_LOW_SPEED_GAIN - AIM_MIN_SPEED)).clamp(0., 1.);
        let strength = (0.15 + 0.85 * (speed / 500_000.).min(1.)) * ramp;
        self.aim_target = [dx * strength / distance, dy * strength / distance];
        self.aim_start = reconcile_aim(start, self.aim_target);
    }
}
