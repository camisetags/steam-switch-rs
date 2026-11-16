#[cfg(test)]
mod tests {
    use super::*;
    use controller_core::input::Stick;

    #[test]
    fn input_windows_keep_gap_history_and_exclude_touch_edges_from_motion() {
        let mut trace = InputTrace::default();
        trace.receive(1_000, Pad::default());
        let mut pad = Pad {
            position: Stick { x: 100, y: 0 },
            touched: true,
        };
        trace.receive(8_500, pad);
        assert_eq!(trace.take().touched, 1);

        pad.position.x = 120;
        trace.receive(16_000, pad);
        trace.receive(31_000, pad);
        trace.receive(36_000, Pad::default());
        pad.position.x = -20_000;
        trace.receive(48_000, pad);
        let stats = trace.take();
        assert_eq!(stats.received, 4);
        assert_eq!(stats.touched, 3);
        assert_eq!(stats.moved, 1);
        assert_eq!(stats.max_gap_us, 15_000);
        assert_eq!(stats.max_touch_gap_us, 15_000);
        assert_eq!(stats.max_step, 20);
    }

    #[test]
    fn output_windows_distinguish_queue_wait_from_submit_error() {
        let mut trace = OutputTrace::default();
        assert!(trace.window_due(1_000).is_none());
        let mut report = [0; 63];
        report[8..11].copy_from_slice(&[0x00, 0x08, 0x80]);
        trace.queued(2_000, &report);
        trace.pool_busy();
        trace.rejected(6);
        report[8..11].copy_from_slice(&[0x20, 0x08, 0x81]);
        trace.queued(17_000, &report);
        assert!(trace.window_due(1_000_999).is_none());
        let (elapsed, stats) = trace.window_due(1_001_000).unwrap();
        assert_eq!(elapsed, 1_000_000);
        assert_eq!(stats.queued, 2);
        assert_eq!(stats.pool_busy, 1);
        assert_eq!(stats.rejected, 1);
        assert_eq!(stats.last_error, 6);
        assert_eq!(stats.max_gap_us, 15_000);
        assert_eq!(stats.max_step, 32);

        let (_, idle) = trace.window_due(2_001_000).unwrap();
        assert_eq!(idle.queued, 0);
        assert_eq!(idle.pool_busy, 0);
    }

    #[test]
    fn timings_report_mean_percentile_bounds_and_gap_thresholds() {
        let mut timing = Timing::default();
        for delay in [25, 100, 7_500, 24_999, 25_000, 35_000, 45_000, 100_000] {
            timing.record(delay);
        }
        assert_eq!(timing.mean(), 29_703);
        assert_eq!(timing.percentile(50), 24_999);
        assert_eq!(timing.percentile(95), 100_000);
        assert_eq!(timing.percentile(99), 100_000);
        assert_eq!(timing.long_gaps(), [4, 3, 2, 1, 1]);
    }

    #[test]
    fn sample_age_is_measured_only_for_accepted_reports_with_an_input_sample() {
        let mut trace = OutputTrace::default();
        trace.window_due(1000);
        trace.attempted(1500);
        trace.pool_busy();
        trace.attempted(2200);
        trace.record_submit_age(2250, Some(1000));
        trace.attempted(3000);
        trace.record_submit_age(3050, None);
        let (_, window) = trace.window_due(1_001_000).unwrap();
        assert_eq!(window.attempts, 3);
        assert_eq!(window.attempt_gap.mean(), 750);
        assert_eq!(window.sample_age.count, 1);
        assert_eq!(window.sample_age.max, 1250);
        assert_eq!(window.pool_busy, 1);
    }
}
use std::fmt;

use controller_core::input::{Pad, REPORT_LEN};

#[cfg(target_os = "espidf")]
pub(crate) const ENABLED: bool = option_env!("STEAM_SWITCH_AIM_DIAGNOSTICS").is_some();

// Lower bounds in microseconds. The final five are the gap cutoffs in the log.
const BUCKETS: [u64; 20] = [
    0, 25, 50, 100, 250, 500, 1_000, 2_000, 5_000, 7_500, 10_000, 12_500, 15_000, 17_500, 20_000,
    25_000, 35_000, 45_000, 60_000, 100_000,
];

#[derive(Default)]
pub(crate) struct Timing {
    bins: [u32; BUCKETS.len()],
    count: u32,
    total: u64,
    max: u64,
}

impl Timing {
    pub fn record(&mut self, value: u64) {
        let bin = BUCKETS.partition_point(|&bound| bound <= value) - 1;
        self.bins[bin] += 1;
        self.count += 1;
        self.total += value;
        self.max = self.max.max(value);
    }

    fn mean(&self) -> u64 {
        self.total.checked_div(u64::from(self.count)).unwrap_or(0)
    }

    fn percentile(&self, percent: u32) -> u64 {
        if self.count == 0 {
            return 0;
        }
        let rank = (self.count * percent).div_ceil(100);
        let mut cumulative = 0;
        for (bin, &count) in self.bins.iter().enumerate() {
            cumulative += count;
            if cumulative >= rank {
                return BUCKETS
                    .get(bin + 1)
                    .map_or(self.max, |&upper| self.max.min(upper - 1));
            }
        }
        self.max
    }

    pub fn long_gaps(&self) -> [u32; 5] {
        let start = BUCKETS.len() - 5;
        std::array::from_fn(|i| self.bins[start + i..].iter().sum())
    }
}

impl fmt::Display for Timing {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}/{}/{}/{}/{}/{}",
            self.count,
            self.mean(),
            self.percentile(50),
            self.percentile(95),
            self.percentile(99),
            self.max,
        )
    }
}

#[derive(Default)]
pub(crate) struct InputWindow {
    pub received: u32,
    pub touched: u32,
    pub moved: u32,
    pub max_gap_us: u64,
    pub max_touch_gap_us: u64,
    pub max_step: u16,
    pub gap: Timing,
    pub touch_gap: Timing,
    #[cfg(target_os = "espidf")]
    pub decode: Timing,
    #[cfg(target_os = "espidf")]
    pub input_wait: Timing,
    #[cfg(target_os = "espidf")]
    pub input_update: Timing,
    #[cfg(target_os = "espidf")]
    pub output_wait: Timing,
    #[cfg(target_os = "espidf")]
    pub aim: Timing,
    #[cfg(target_os = "espidf")]
    pub aim_sample_age: Timing,
    #[cfg(target_os = "espidf")]
    pub macro_wait: Timing,
    #[cfg(target_os = "espidf")]
    pub output_work: Timing,
}

#[derive(Default)]
pub(crate) struct InputTrace {
    last: Option<(u64, Pad)>,
    window: InputWindow,
}

impl InputTrace {
    pub fn receive(&mut self, time: u64, pad: Pad) {
        self.window.received += 1;
        self.window.touched += u32::from(pad.touched);
        if let Some((then, previous)) = self.last.replace((time, pad)) {
            let gap = time.saturating_sub(then);
            self.window.gap.record(gap);
            self.window.max_gap_us = self.window.max_gap_us.max(gap);
            if pad.touched && previous.touched {
                self.window.touch_gap.record(gap);
                let step = pad
                    .position
                    .x
                    .abs_diff(previous.position.x)
                    .max(pad.position.y.abs_diff(previous.position.y));
                self.window.moved += u32::from(step != 0);
                self.window.max_touch_gap_us = self.window.max_touch_gap_us.max(gap);
                self.window.max_step = self.window.max_step.max(step);
            }
        }
    }

    pub fn take(&mut self) -> InputWindow {
        std::mem::take(&mut self.window)
    }

    #[cfg(target_os = "espidf")]
    pub fn window(&mut self) -> &mut InputWindow {
        &mut self.window
    }
}

#[derive(Default)]
pub(crate) struct OutputWindow {
    pub queued: u32,
    pub pool_busy: u32,
    pub rejected: u32,
    pub last_error: i32,
    pub max_gap_us: u64,
    pub max_step: u16,
    pub attempts: u32,
    pub attempt_gap: Timing,
    pub submit_gap: Timing,
    pub sample_age: Timing,
    #[cfg(target_os = "espidf")]
    pub interval_us: u64,
}

#[derive(Default)]
pub(crate) struct OutputTrace {
    since_us: Option<u64>,
    last: Option<(u64, [u16; 2])>,
    last_attempt: Option<u64>,
    window: OutputWindow,
}

impl OutputTrace {
    pub fn attempted(&mut self, time: u64) {
        self.window.attempts += 1;
        if let Some(then) = self.last_attempt.replace(time) {
            self.window.attempt_gap.record(time.saturating_sub(then));
        }
    }

    pub fn record_submit_age(&mut self, time: u64, sample: Option<u64>) {
        if let Some(sample) = sample {
            self.window.sample_age.record(time.saturating_sub(sample));
        }
    }

    #[cfg(target_os = "espidf")]
    pub fn set_interval(&mut self, interval_us: u64) {
        self.window.interval_us = interval_us;
    }

    pub fn queued(&mut self, time: u64, report: &[u8; REPORT_LEN]) {
        let [lo, packed, hi] = [report[8], report[9], report[10]].map(u16::from);
        let axes = [lo | ((packed & 0x0f) << 8), (packed >> 4) | (hi << 4)];
        self.window.queued += 1;
        if let Some((then, previous)) = self.last.replace((time, axes)) {
            let gap = time.saturating_sub(then);
            self.window.submit_gap.record(gap);
            self.window.max_gap_us = self.window.max_gap_us.max(gap);
            let step = axes[0]
                .abs_diff(previous[0])
                .max(axes[1].abs_diff(previous[1]));
            self.window.max_step = self.window.max_step.max(step);
        }
    }

    pub fn pool_busy(&mut self) {
        self.window.pool_busy += 1;
    }

    pub fn rejected(&mut self, code: i32) {
        self.window.rejected += 1;
        self.window.last_error = code;
    }

    pub fn window_due(&mut self, time: u64) -> Option<(u64, OutputWindow)> {
        let elapsed = time.saturating_sub(*self.since_us.get_or_insert(time));
        if elapsed < 1_000_000 {
            return None;
        }
        self.since_us = Some(time);
        Some((elapsed, std::mem::take(&mut self.window)))
    }
}
