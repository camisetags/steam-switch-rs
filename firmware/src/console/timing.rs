#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retries_a_busy_slot_and_keeps_the_negotiated_cadence() {
        let mut pacer = ReportPacer::default();
        assert!(pacer.due(0));
        // Headroom is unavailable at 0. The slot remains due on the next poll.
        assert!(pacer.due(1_000));
        pacer.sent(1_000, 7_500);
        assert!(!pacer.due(7_499));
        assert!(pacer.due(7_500));
        pacer.sent(8_000, 7_500);
        assert!(pacer.due(15_000));
    }

    #[test]
    fn resumes_at_a_regular_spacing_after_a_long_stall() {
        let mut pacer = ReportPacer::default();
        pacer.sent(0, 7_500);
        assert!(pacer.due(30_000));
        pacer.sent(30_000, 7_500);
        assert!(!pacer.due(37_499));
        assert!(pacer.due(37_500));
    }
}
#[derive(Default)]
pub(super) struct ReportPacer {
    next_us: u64,
}

impl ReportPacer {
    pub(super) fn due(&self, now_us: u64) -> bool {
        now_us >= self.next_us
    }

    pub(super) fn sent(&mut self, now_us: u64, period_us: u64) {
        // Keep the original phase for small scheduler delays. After a long
        // stall, start a new period so the next update doesn't queue as a burst.
        let phase = if now_us.saturating_sub(self.next_us) < period_us / 2 {
            self.next_us
        } else {
            now_us
        };
        self.next_us = phase + period_us;
    }
}
