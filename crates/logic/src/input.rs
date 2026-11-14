//! Valve BLE reports and Nintendo Pro Controller input reports.
//! Wire layouts: SDL's Triton structures (zlib) and espp's Pro2InputReport (MIT).

pub const REPORT_LEN: usize = 63;
pub const HOME: u8 = 16;
const LEFT_TOUCH: u32 = 1 << 25;
const RIGHT_TOUCH: u32 = 1 << 21;

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
            Some((4, 4)),
            None,
            Some((2, 16)),
            Some((3, 1)),
            Some((3, 2)),
            Some((3, 4)),
            Some((3, 8)),
            Some((3, 64)),
            Some((3, 128)),
            Some((4, 1)),
            Some((4, 8)),
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
        let left = axes(input.left_stick);
        let right = axes(input.right_stick);
        let aim = axes(self.aim(input.right_pad, now_us));
        pack(
            &mut report[5..8],
            if left == [0.; 2] && input.left_pad.touched {
                axes(input.left_pad.position)
            } else {
                left
            },
        );
        pack(
            &mut report[8..11],
            if right == [0.; 2] { aim } else { right },
        );
        report
    }

    fn aim(&mut self, pad: Pad, now_us: u64) -> Stick {
        let current = pad.touched.then_some((pad.position, now_us));
        let previous = std::mem::replace(&mut self.previous_aim, current);
        let Some(((point, now), (old, then))) = current.zip(previous) else {
            return Stick::default();
        };
        let dt = now.saturating_sub(then);
        if dt == 0 || dt > 100_000 {
            return Stick::default();
        }
        let dx = f32::from(point.x) - f32::from(old.x);
        let dy = f32::from(point.y) - f32::from(old.y);
        let distance = dx.hypot(dy);
        if distance < 24. {
            return Stick::default();
        }
        let speed = distance * 1_000_000. / dt as f32;
        let strength = 0.15 + 0.85 * (speed / 500_000.).min(1.);
        let gain = strength * 32767. / distance;
        Stick {
            x: (gain * dx) as i16,
            y: (gain * dy) as i16,
        }
    }
}
