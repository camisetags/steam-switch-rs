//! Switch 2 30-byte motion carrier. Wire layout and quantization reference:
//! PicoSwitch2, `ns2_motion_pdu.c` and `ns2_ds5_motion.c` (Apache-2.0).

use crate::input::Imu;

const GYRO_DPS_PER_COUNT: f32 = 2000. / 32768.;
const STICK_SMOOTHING_US: f32 = 10_000.;
const STICK_HOLD_US: u64 = 25_000;
const STICK_TIMEOUT_US: u64 = 75_000;
const GYRO_DEADZONE_DPS: f32 = 3.;
const GYRO_FULL_STICK_DPS: f32 = 140.;
const MIN_STICK_DEFLECTION: f32 = 0.16;

#[derive(Default)]
pub(crate) struct StickGyro {
    // Width, sensor timestamp, and host arrival of the last fresh sample.
    sample: Option<(u8, u32, u64)>,
    start: [f32; 2],
    target: [f32; 2],
}

impl StickGyro {
    pub(crate) fn update(&mut self, imu: Option<Imu>, held: bool, now_us: u64) {
        if !held {
            *self = Self::default();
            return;
        }
        let Some(imu) = imu else { return };
        if self.sample.is_some_and(|(width, timestamp, _)| {
            width == imu.timestamp_bits && timestamp == imu.timestamp
        }) {
            return;
        }

        let [pitch, roll, yaw] = imu.gyro.map(|value| f32::from(value) * GYRO_DPS_PER_COUNT);
        let [_, ay, az] = imu.accel.map(f32::from);
        let vertical = ay.hypot(az);
        // SDL maps Triton X to pitch and Z to yaw. Project yaw onto gravity
        // so turning left/right also works with the controller tilted forward.
        let yaw = if vertical > 8192. {
            -(roll * ay + yaw * az) / vertical
        } else {
            -yaw
        };
        let speed = yaw.hypot(pitch);
        let target = if speed <= GYRO_DEADZONE_DPS {
            [0.; 2]
        } else {
            let fraction = (speed - GYRO_DEADZONE_DPS) / (GYRO_FULL_STICK_DPS - GYRO_DEADZONE_DPS);
            // A small nonzero deflection clears the game's stick deadzone.
            let gain = MIN_STICK_DEFLECTION + (1. - MIN_STICK_DEFLECTION) * fraction.min(1.);
            [yaw * gain / speed, pitch * gain / speed]
        };
        self.start = self.at(now_us);
        self.target = target;
        self.sample = Some((imu.timestamp_bits, imu.timestamp, now_us));
    }

    pub(crate) fn at(&self, now_us: u64) -> [f32; 2] {
        let Some((_, _, last_us)) = self.sample else {
            return [0.; 2];
        };
        let elapsed = now_us.saturating_sub(last_us);
        if elapsed >= STICK_TIMEOUT_US || (self.start == [0.; 2] && self.target == [0.; 2]) {
            return [0.; 2];
        }
        let rise = -(-(elapsed.min(STICK_HOLD_US) as f32) / STICK_SMOOTHING_US).exp_m1();
        let decay = (-(elapsed.saturating_sub(STICK_HOLD_US) as f32) / STICK_SMOOTHING_US).exp();
        std::array::from_fn(|axis| {
            (self.start[axis] + (self.target[axis] - self.start[axis]) * rise) * decay
        })
    }
}

pub struct Motion {
    attitude: [f32; 4],
    reported_attitude: [f32; 4],
    aim_offset: Option<[f32; 4]>,
    previous: Option<(u32, u8, u64)>,
    clock_us: u64,
    chart: usize,
    bias: [f32; 3],
}

impl Default for Motion {
    fn default() -> Self {
        Self {
            attitude: [0., 0., 0., 1.],
            reported_attitude: [0., 0., 0., 1.],
            aim_offset: None,
            previous: None,
            clock_us: 0,
            chart: 0,
            bias: [0.; 3],
        }
    }
}

impl Motion {
    pub fn apply(
        &mut self,
        imu: Option<Imu>,
        aim_enabled: bool,
        arrival_us: u64,
        report: &mut [u8; 63],
    ) {
        if !aim_enabled {
            self.aim_offset = None;
        }
        let Some(imu) = imu else {
            return;
        };
        let accel = imu.accel.map(|v| f32::from(v) / 16384.);
        let mut elapsed = 0;
        if let Some((last_tick, width, last_arrival)) = self.previous {
            if width == imu.timestamp_bits && arrival_us.saturating_sub(last_arrival) < 100_000 {
                let diff = imu.timestamp.wrapping_sub(last_tick);
                elapsed = if width == 16 {
                    (diff & 0xffff) * 32
                } else {
                    diff
                };
                if elapsed > 100_000 {
                    elapsed = 0;
                }
            }
        }
        self.previous = Some((imu.timestamp, imu.timestamp_bits, arrival_us));
        let last_slot = self.clock_us / 1250;
        self.clock_us += u64::from(elapsed);
        if elapsed > 0 {
            self.integrate(imu.gyro, accel, elapsed as f32 / 1_000_000.);
        }
        // On press, align the controller's current pose to the last pose
        // reported to the console. Track the physical pose between presses.
        let mut reported_accel = gravity_direction(self.reported_attitude);
        if aim_enabled {
            match self.aim_offset {
                Some(offset) if elapsed > 0 => {
                    self.reported_attitude = quaternion_product(offset, self.attitude);
                    let physical_gravity = gravity_direction(self.attitude);
                    let reported_gravity = gravity_direction(self.reported_attitude);
                    // Retain linear acceleration with the rebased gravity.
                    reported_accel = std::array::from_fn(|axis| {
                        reported_gravity[axis] + (accel[axis] - physical_gravity[axis])
                    });
                }
                None => {
                    let [w, x, y, z] = self.attitude;
                    let offset = quaternion_product(self.reported_attitude, [w, -x, -y, -z]);
                    let length = norm(offset);
                    self.aim_offset = Some(offset.map(|value| value / length));
                }
                Some(_) => {}
            }
        }
        let slot = self.clock_us / 1250;
        let ticks = ((slot - last_slot).min(15) as u16) << 12;
        let mut carrier = [0; 30];
        carrier[0..2].copy_from_slice(&(ticks | (slot as u16 & 0x0fff)).to_le_bytes());
        carrier[3] = 12;
        self.orientation(&mut carrier);
        for (axis, g) in reported_accel.into_iter().enumerate() {
            let offset = 16 + 4 * axis;
            let value = (g * 4096. * 68963.) as i32;
            carrier[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        carrier[29] = 2;
        report[14] = 30;
        report[15..45].copy_from_slice(&carrier);
    }

    fn integrate(&mut self, gyro: [i16; 3], accel: [f32; 3], dt: f32) {
        let mut angular = gyro.map(|v| (f32::from(v) * GYRO_DPS_PER_COUNT).to_radians());
        let gravity = norm(accel);
        if (0.96..1.04).contains(&gravity) && norm(angular) < 0.025 {
            for (estimate, sample) in self.bias.iter_mut().zip(angular) {
                *estimate += (sample - *estimate) * (dt * 0.7).min(1.);
            }
        }
        for (v, bias) in angular.iter_mut().zip(self.bias) {
            *v -= bias;
        }
        let [w, x, y, z] = self.attitude;
        if (0.92..1.08).contains(&gravity) {
            let down = gravity_direction(self.attitude);
            let measured = accel.map(|g| g / gravity);
            let correction = [
                measured[1] * down[2] - measured[2] * down[1],
                measured[2] * down[0] - measured[0] * down[2],
                measured[0] * down[1] - measured[1] * down[0],
            ];
            for (v, correction) in angular.iter_mut().zip(correction) {
                *v += 0.25 * correction;
            }
        }
        let [a, b, c] = angular.map(|v| v * dt / 2.);
        let q = [
            w - x * a - y * b - z * c,
            x + w * a + y * c - z * b,
            y + w * b - x * c + z * a,
            z + w * c + x * b - y * a,
        ];
        let length = norm(q);
        if length.is_finite() && length > 0. {
            self.attitude = q.map(|v| v / length);
        }
    }

    fn orientation(&mut self, carrier: &mut [u8; 30]) {
        let q = self.reported_attitude;
        const LIMIT: f32 = std::f32::consts::FRAC_1_SQRT_2;
        if (0..4).any(|i| i != self.chart && q[i].abs() > LIMIT) {
            self.chart = (0..4)
                .max_by(|&a, &b| q[a].abs().total_cmp(&q[b].abs()))
                .unwrap();
        }
        let sign = q[self.chart].signum();
        for (i, (bits, low, high)) in [(26, 5, 8), (25, 9, 12), (24, 13, 4)]
            .into_iter()
            .enumerate()
        {
            let index = (self.chart + i + 1) % 4;
            let ceiling = 1u32 << bits;
            let encoded = ((q[index] * sign * LIMIT + 0.5) * ceiling as f32).round() as u32;
            let value = encoded.min(ceiling - 1).to_le_bytes();
            carrier[low..low + 3].copy_from_slice(&value[..3]);
            carrier[high] = if i == 2 { self.chart as u8 } else { value[3] };
        }
    }
}

fn quaternion_product([w, x, y, z]: [f32; 4], [a, b, c, d]: [f32; 4]) -> [f32; 4] {
    [
        w * a - x * b - y * c - z * d,
        w * b + x * a + y * d - z * c,
        w * c - x * d + y * a + z * b,
        w * d + x * c - y * b + z * a,
    ]
}

fn gravity_direction([w, x, y, z]: [f32; 4]) -> [f32; 3] {
    [
        2. * (x * z - w * y),
        2. * (w * x + y * z),
        w * w - x * x - y * y + z * z,
    ]
}

fn norm<const N: usize>(v: [f32; N]) -> f32 {
    v.into_iter().map(|x| x * x).sum::<f32>().sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_survives_timestamp_wrap() {
        let mut motion = Motion::default();
        let mut report = [0; 63];
        let mut imu = Imu {
            accel: [0, 0, 16384],
            gyro: [0, 0, 3000],
            timestamp: 65500,
            timestamp_bits: 16,
        };
        motion.apply(Some(imu), true, 2000, &mut report);
        let first = report;
        imu.timestamp = 220;
        motion.apply(Some(imu), true, 10000, &mut report);
        assert_eq!(report[14], 30);
        assert_ne!(report[15..17], first[15..17]);
        assert_ne!(report[19..31], first[19..31]);
        assert_eq!(
            i32::from_le_bytes(report[39..43].try_into().unwrap()),
            4096 * 68963
        );
    }

    #[test]
    fn holding_motion_aim_rebases_after_repositioning() {
        let mut motion = Motion::default();
        let mut report = [0; 63];
        let sample = |timestamp, accel, gyro| {
            Some(Imu {
                accel,
                gyro,
                timestamp,
                timestamp_bits: 32,
            })
        };
        let up = [0, 0, 16384];
        let tilt = [0, 3000, 0];
        motion.apply(sample(0, up, [0; 3]), false, 0, &mut report);
        let initial = report;
        motion.apply(
            sample(15_000, [4096, 0, 15863], tilt),
            false,
            15_000,
            &mut report,
        );
        assert_eq!(report[19..44], initial[19..44]);
        assert_ne!(report[15..17], initial[15..17]);

        motion.apply(sample(30_000, up, tilt), true, 30_000, &mut report);
        assert_eq!(report[19..44], initial[19..44]);
        motion.apply(sample(45_000, up, tilt), true, 45_000, &mut report);
        assert_ne!(report[19..31], initial[19..31]);
        let aimed = report;

        motion.apply(sample(60_000, up, tilt), false, 60_000, &mut report);
        assert_eq!(report[19..31], aimed[19..31]);
        let released = report;
        motion.apply(
            sample(75_000, [4096, 0, 15863], tilt),
            false,
            75_000,
            &mut report,
        );
        assert_eq!(report[19..44], released[19..44]);
        motion.apply(sample(90_000, up, tilt), true, 90_000, &mut report);
        assert_eq!(report[19..44], released[19..44]);
        motion.apply(sample(105_000, up, tilt), true, 105_000, &mut report);
        assert_ne!(report[19..31], released[19..31]);

        let before_missing_imu = report;
        motion.apply(None, false, 110_000, &mut report);
        motion.apply(sample(120_000, up, tilt), true, 120_000, &mut report);
        assert_eq!(report[19..31], before_missing_imu[19..31]);
    }
}
