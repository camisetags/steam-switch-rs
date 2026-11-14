//! Switch 2 30-byte motion carrier. Wire layout and quantization reference:
//! PicoSwitch2, `ns2_motion_pdu.c` and `ns2_ds5_motion.c` (Apache-2.0).

use crate::input::Imu;

pub struct Motion {
    attitude: [f32; 4],
    previous: Option<(u32, u8, u64)>,
    clock_us: u64,
    chart: usize,
    bias: [f32; 3],
}

impl Default for Motion {
    fn default() -> Self {
        Self {
            attitude: [0., 0., 0., 1.],
            previous: None,
            clock_us: 0,
            chart: 0,
            bias: [0.; 3],
        }
    }
}

impl Motion {
    pub fn apply(&mut self, imu: Imu, arrival_us: u64, report: &mut [u8; 63]) {
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
        let slot = self.clock_us / 1250;
        let ticks = ((slot - last_slot).min(15) as u16) << 12;
        let mut carrier = [0; 30];
        carrier[0..2].copy_from_slice(&(ticks | (slot as u16 & 0x0fff)).to_le_bytes());
        carrier[3] = 12;
        self.orientation(&mut carrier);
        for (axis, g) in accel.into_iter().enumerate() {
            let offset = 16 + 4 * axis;
            let value = (g * 4096. * 68963.) as i32;
            carrier[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        carrier[29] = 2;
        report[14] = 30;
        report[15..45].copy_from_slice(&carrier);
    }

    fn integrate(&mut self, gyro: [i16; 3], accel: [f32; 3], dt: f32) {
        let mut angular = gyro.map(|v| (f32::from(v) * 2000. / 32768.).to_radians());
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
            let down = [
                2. * (x * z - w * y),
                2. * (w * x + y * z),
                w * w - x * x - y * y + z * z,
            ];
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
        let q = self.attitude;
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
        motion.apply(imu, 2000, &mut report);
        let first = report;
        imu.timestamp = 220;
        motion.apply(imu, 10000, &mut report);
        assert_eq!(report[14], 30);
        assert_ne!(report[15..17], first[15..17]);
        assert_ne!(report[19..31], first[19..31]);
        assert_eq!(
            i32::from_le_bytes(report[39..43].try_into().unwrap()),
            4096 * 68963
        );
    }
}
