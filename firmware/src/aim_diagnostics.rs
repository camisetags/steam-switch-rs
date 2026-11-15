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
}
use controller_core::input::{Pad, REPORT_LEN};

#[cfg(target_os = "espidf")]
pub(crate) const ENABLED: bool = option_env!("STEAM_SWITCH_AIM_DIAGNOSTICS").is_some();

#[derive(Default)]
pub(crate) struct InputWindow {
    pub received: u32,
    pub touched: u32,
    pub moved: u32,
    pub max_gap_us: u64,
    pub max_touch_gap_us: u64,
    pub max_step: u16,
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
            self.window.max_gap_us = self.window.max_gap_us.max(gap);
            if pad.touched && previous.touched {
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
}

#[derive(Default)]
pub(crate) struct OutputWindow {
    pub queued: u32,
    pub pool_busy: u32,
    pub rejected: u32,
    pub last_error: i32,
    pub max_gap_us: u64,
    pub max_step: u16,
}

#[derive(Default)]
pub(crate) struct OutputTrace {
    since_us: Option<u64>,
    last: Option<(u64, [u16; 2])>,
    window: OutputWindow,
}

impl OutputTrace {
    pub fn queued(&mut self, time: u64, report: &[u8; REPORT_LEN]) {
        let [lo, packed, hi] = [report[8], report[9], report[10]].map(u16::from);
        let axes = [lo | ((packed & 0x0f) << 8), (packed >> 4) | (hi << 4)];
        self.window.queued += 1;
        if let Some((then, previous)) = self.last.replace((time, axes)) {
            self.window.max_gap_us = self.window.max_gap_us.max(time.saturating_sub(then));
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
