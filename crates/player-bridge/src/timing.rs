//! Fixed Player measurement-clock to host-clock mapping, shared by scan and odom.
//!
//! Player timestamps may be uptime, not Unix time. Anchor the first finite,
//! positive measurement to its host receive time, then retain source deltas,
//! including out-of-order measurements. Invalid timestamps use their own receive
//! time and never change the anchor. Unrepresentable mapped times also fall back.
//!
//! This cannot recover transport latency before the first sample, synchronize
//! clock rates, or detect a Player clock reset. No drift/reset correction is
//! inferred: a new bridge session establishes a new anchor.

#[derive(Default)]
pub struct PlayerClock {
    anchor: Option<(f64, i64)>,
}

impl PlayerClock {
    /// Map Player seconds to host nanoseconds, called in the reader, not dequeue.
    pub fn map(&mut self, source_seconds: f64, received_ns: i64) -> i64 {
        if !source_seconds.is_finite() || source_seconds <= 0.0 {
            return received_ns;
        }
        let (source_anchor, host_anchor) =
            *self.anchor.get_or_insert((source_seconds, received_ns));
        let delta_ns = ((source_seconds - source_anchor) * 1_000_000_000.0).round();
        // Reject overflow rather than silently saturating a float-to-int cast.
        if !delta_ns.is_finite() || delta_ns < i64::MIN as f64 || delta_ns >= i64::MAX as f64 {
            return received_ns;
        }
        host_anchor
            .checked_add(delta_ns as i64)
            .unwrap_or(received_ns)
    }
}
