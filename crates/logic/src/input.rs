//! Valve BLE reports and Nintendo Pro Controller input reports.
//! Wire layouts: SDL's Triton structures (zlib) and espp's Pro2InputReport (MIT).

pub const REPORT_LEN: usize = 63;
pub const HOME: u8 = 16;
const LEFT_TOUCH: u32 = 1 << 25;
const RIGHT_TOUCH: u32 = 1 << 21;
const RIGHT_STICK_PRECISION_PADDLES: u32 = (1 << 7) | (1 << 17); // R4 and L4
const LOWER_PADDLES: u32 = (1 << 8) | (1 << 18); // R5 and L5
const RIGHT_STICK_PRECISION_GAIN: f32 = 0.5;
const LEFT_PAD_CENTER: u32 = 8192;
const AIM_SMOOTHING_US: f32 = 10_000.;
const AIM_RECOVERY_SMOOTHING_US: f32 = 24_000.;
const AIM_RECOVERY_WINDOW_US: u64 = 60_000;
const AIM_HOLD_US: u64 = 35_000;
const AIM_RESET_US: u64 = 100_000;
const AIM_BURST_US: u64 = 5_000;
const AIM_NOMINAL_SAMPLE_US: u64 = 15_000;
const AIM_MAX_REGULAR_SAMPLE_US: u64 = 25_000;
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
        self.buttons & !(RIGHT_STICK_PRECISION_PADDLES | LOWER_PADDLES)
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

#[derive(Default)]
pub struct Mapper {
    previous_aim: Option<(Stick, u64)>,
    aim_start: [f32; 2],
    aim_target: [f32; 2],
    aim_recover_until: u64,
    aim_sample_interval: Option<u64>,
    right_stick_active: bool,
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
            None, // L4: right-stick precision
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
        let mut right = axes(input.right_stick);
        self.right_stick_active = right != [0.; 2];
        if input.buttons & RIGHT_STICK_PRECISION_PADDLES != 0 {
            right = right.map(|axis| axis * RIGHT_STICK_PRECISION_GAIN);
        }
        self.update_aim(input.right_pad, now_us);
        pack(&mut report[8..11], right);
        self.apply_aim(&mut report, now_us);
        report
    }

    /// Refresh the right trackpad in the latest encoded report at output time.
    pub fn apply_aim(&self, report: &mut [u8; REPORT_LEN], now_us: u64) {
        if !self.right_stick_active {
            pack(&mut report[8..11], self.aim_at(now_us));
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
        if self.aim_start == [0.; 2] && self.aim_target == [0.; 2] {
            return [0.; 2];
        }
        // Reconstruct the same curve regardless of how often the console polls.
        // If input stalls, let the output decay even without a new touch sample.
        let smoothing_us = if last_us < self.aim_recover_until && self.aim_target != [0.; 2] {
            AIM_RECOVERY_SMOOTHING_US
        } else {
            AIM_SMOOTHING_US
        };
        let rise = -(-(elapsed.min(AIM_HOLD_US) as f32) / smoothing_us).exp_m1();
        let decay = (-(elapsed.saturating_sub(AIM_HOLD_US) as f32) / AIM_SMOOTHING_US).exp();
        std::array::from_fn(|i| {
            (self.aim_start[i] + (self.aim_target[i] - self.aim_start[i]) * rise) * decay
        })
    }

    fn update_aim(&mut self, pad: Pad, now_us: u64) {
        let start = self.aim_at(now_us);
        let current = pad.touched.then_some((pad.position, now_us));
        let previous = std::mem::replace(&mut self.previous_aim, current);
        self.aim_start = [0.; 2];
        self.aim_target = [0.; 2];
        let Some(((point, now), (old, then))) = current.zip(previous) else {
            self.aim_recover_until = 0;
            return;
        };
        let observed_us = now.saturating_sub(then);
        if observed_us >= AIM_HOLD_US {
            self.aim_recover_until = now + AIM_RECOVERY_WINDOW_US;
        }
        if observed_us >= AIM_RESET_US {
            return;
        }
        if (AIM_BURST_US..=AIM_MAX_REGULAR_SAMPLE_US).contains(&observed_us) {
            let previous = self.aim_sample_interval.unwrap_or(observed_us);
            self.aim_sample_interval = Some((3 * previous + observed_us) / 4);
        }
        self.aim_start = start;
        let dx = f32::from(point.x) - f32::from(old.x);
        let dy = f32::from(point.y) - f32::from(old.y);
        let distance = dx.hypot(dy);
        if distance < 24. {
            return;
        }
        // Bluetooth can deliver buffered samples together. Use the usual sample
        // interval for these arrivals so the transport doesn't amplify the swipe.
        let sample_us = if observed_us < AIM_BURST_US {
            self.aim_sample_interval.unwrap_or(AIM_NOMINAL_SAMPLE_US)
        } else {
            observed_us
        };
        let speed = distance * 1_000_000. / sample_us as f32;
        let strength = 0.15 + 0.85 * (speed / 500_000.).min(1.);
        self.aim_target = [dx * strength / distance, dy * strength / distance];
    }
}
