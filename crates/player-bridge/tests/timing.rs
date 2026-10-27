//! BUG: measurement time is replaced by transport/dequeue time.
//! HYPOTHESIS: a fixed shared source-to-host offset preserves source deltas.
//! SOLUTION: anchor the Player clock once and retain measurement intervals.
//! RESULT: FIXED — delays, batching and out-of-order measurements retain their times.
//! STATUS: FIXED

use player_bridge::timing::PlayerClock;

#[test]
fn shared_anchor_preserves_delays_batches_and_out_of_order_interfaces() {
    let mut clock = PlayerClock::default();
    let host = 1_700_000_000_000_000_000;
    assert_eq!(clock.map(60_000.0, host), host);
    assert_eq!(clock.map(60_000.0, host + 250_000_000), host);
    for i in 0..10 {
        assert_eq!(
            clock.map(60_001.0 + i as f64 * 0.05, host + 500_000_000),
            host + 1_000_000_000 + i * 50_000_000
        );
    }
    assert_eq!(clock.map(59_999.5, host + 900_000_000), host - 500_000_000);
}

#[test]
fn invalid_timestamps_use_receive_time_without_establishing_or_moving_anchor() {
    let mut clock = PlayerClock::default();
    for source in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(clock.map(source, 10_000_000_000), 10_000_000_000);
    }
    assert_eq!(clock.map(123.0, 20_000_000_000), 20_000_000_000);
    for source in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(clock.map(source, 30_000_000_000), 30_000_000_000);
    }
    assert_eq!(clock.map(123.25, 40_000_000_000), 20_250_000_000);
    assert_eq!(clock.map(f64::MAX, 50_000_000_000), 50_000_000_000);
    assert_eq!(clock.map(123.5, 60_000_000_000), 20_500_000_000);
}
