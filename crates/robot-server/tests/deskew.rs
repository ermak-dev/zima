// FEATURE: motion compensation only from explicit beam times and a verified trajectory.
// SPEC: SE(2) interpolation, LDS lever arm, bounded finite input and no inferred beam order.
// IMPLEMENTATION: explicit-time SE(2) kernel shared with verified odometry interpolation.
// RESULT: IMPLEMENTED — all six checks pass; physical Player beam timing remains unverified.
// STATUS: DONE (offline kernel only; tracker #41 remains open)
use robot_server::motion_compensation::{
    deskew, deskew_rays_from_slots, Pose2, TimedPose, TimedRay,
};

fn pose(time: f64, x: f64, y: f64, yaw: f64) -> TimedPose {
    TimedPose {
        time,
        pose: Pose2 { x, y, yaw },
    }
}
fn ray(time: f64, angle: f64, range: f64) -> TimedRay {
    TimedRay { time, angle, range }
}
fn xy(ray: &TimedRay, sensor_x: f64) -> (f64, f64) {
    (
        sensor_x + ray.range * ray.angle.cos(),
        ray.range * ray.angle.sin(),
    )
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 1e-8, "{actual} != {expected}");
}

#[test]
fn translates_each_explicit_beam_time_to_reference_pose() {
    let trajectory = [pose(0., 0., 0., 0.), pose(1., 0.5, 0., 0.)];
    let result = deskew(
        &[ray(1., 0., 2.), ray(0., 0., 2.), ray(0.5, 0., 2.)],
        &trajectory,
        0.,
        -0.1,
    )
    .unwrap();
    close(result[0].range, 2.5);
    close(result[1].range, 2.);
    close(result[2].range, 2.25);
}
#[test]
fn in_place_rotation_includes_lidar_lever_arm() {
    let trajectory = [
        pose(0., 0., 0., 0.),
        pose(1., 0., 0., std::f64::consts::FRAC_PI_2),
    ];
    let result = deskew(&[ray(1., 0., 2.)], &trajectory, 0., -0.1).unwrap();
    let (x, y) = xy(&result[0], -0.1);
    close(x, 0.);
    close(y, 1.9);
}
#[test]
fn interpolates_a_constant_twist_arc_in_se2_not_linear_world_coordinates() {
    let trajectory = [
        pose(0., 0., 0., 0.),
        pose(1., 1., 1., std::f64::consts::FRAC_PI_2),
    ];
    let result = deskew(&[ray(0.5, 0., 2.)], &trajectory, 0., -0.1).unwrap();
    let c = std::f64::consts::FRAC_1_SQRT_2;
    let (x, y) = xy(&result[0], -0.1);
    close(x, c + 1.9 * c);
    close(y, 1. - c + 1.9 * c);
}
#[test]
fn yaw_wrap_uses_the_short_rotation_and_keeps_unknown_input_unknown() {
    let trajectory = [
        pose(0., 0., 0., 179f64.to_radians()),
        pose(1., 0., 0., (-179f64).to_radians()),
    ];
    let result = deskew(&[ray(0.5, 0., 2.)], &trajectory, 0., 0.).unwrap();
    close(result[0].angle, 1f64.to_radians());
    close(result[0].range, 2.);
    assert!(deskew(&[ray(-0.01, 0., 2.)], &trajectory, 0., 0.).is_err());
    assert!(deskew(&[ray(1.01, 0., 2.)], &trajectory, 0., 0.).is_err());
    assert!(deskew(&[ray(0.5, 0., 2.)], &trajectory, 2., 0.).is_err());
}
#[test]
fn rolling_room_scans_become_straight_with_explicit_trajectory_and_times() {
    let sensor_x = -0.1;
    let period = 0.25;
    for (v, w) in [
        (0.5f64, 0.0f64),
        (0., 0.4),
        (0., 1.44),
        (0., 3.14),
        (0.5, 1.44),
    ] {
        let actual = |time: f64| {
            let dx = if w.abs() < 1e-9 {
                v * time
            } else {
                v / w * (w * time).sin()
            };
            let dy = if w.abs() < 1e-9 {
                0.
            } else {
                v / w * (1. - (w * time).cos())
            };
            pose(time, -1. + dx, dy, w * time)
        };
        let trajectory: Vec<_> = (0..=100)
            .map(|i| actual(-period / 2. + period * i as f64 / 100.))
            .collect();
        let mut rays = Vec::new();
        for i in 0..360 {
            let time = -period / 2. + period * i as f64 / 359.;
            let position = actual(time).pose;
            let angle = -std::f64::consts::PI + std::f64::consts::TAU * i as f64 / 360.;
            let px = position.x + sensor_x * position.yaw.cos();
            let py = position.y + sensor_x * position.yaw.sin();
            let dx = (position.yaw + angle).cos();
            let dy = (position.yaw + angle).sin();
            let mut range = f64::INFINITY;
            for x in [-3., 4.] {
                let t = (x - px) / dx;
                if t > 0. && (-2. ..=3.).contains(&(py + t * dy)) {
                    range = range.min(t);
                }
            }
            for y in [-2., 3.] {
                let t = (y - py) / dy;
                if t > 0. && (-3. ..=4.).contains(&(px + t * dx)) {
                    range = range.min(t);
                }
            }
            rays.push(ray(time, angle, range));
        }
        let error = |beam: &TimedRay| {
            let (x, y) = xy(beam, sensor_x);
            let x = x - 1.;
            (x + 3.)
                .abs()
                .min((x - 4.).abs())
                .min((y + 2.).abs())
                .min((y - 3.).abs())
        };
        let before = rays.iter().map(error).fold(0., f64::max);
        rays.reverse();
        let corrected = deskew(&rays, &trajectory, 0., sensor_x).unwrap();
        let after = corrected.iter().map(error).fold(0., f64::max);
        assert!(before > 0.001, "synthetic scan must actually be distorted");
        assert!(after < 1e-7, "v={v}, w={w}, corrected wall error={after}");
    }
}

#[test]
fn rejects_missing_nonmonotonic_or_nonfinite_timing_instead_of_guessing() {
    assert!(deskew(&[ray(0., 0., 2.)], &[], 0., 0.).is_err());
    assert!(deskew(&[ray(0., 0., 2.)], &[pose(0., 0., 0., 0.)], 0., 0.).is_err());
    for trajectory in [
        vec![pose(1., 0., 0., 0.), pose(0., 0., 0., 0.)],
        vec![pose(0., 0., 0., 0.), pose(0., 0., 0., 0.)],
        vec![pose(0., 0., 0., 0.), pose(f64::NAN, 0., 0., 0.)],
        vec![pose(0., 0., 0., 0.), pose(1., f64::INFINITY, 0., 0.)],
    ] {
        assert!(deskew(&[ray(0., 0., 2.)], &trajectory, 0., 0.).is_err());
    }
    let valid = [pose(0., 0., 0., 0.), pose(1., 0., 0., 0.)];
    for beam in [
        ray(f64::NAN, 0., 2.),
        ray(0., f64::INFINITY, 2.),
        ray(0., 0., -1.),
        ray(0., 0., f64::NAN),
    ] {
        assert!(deskew(&[beam], &valid, 0., 0.).is_err());
    }
    assert!(deskew(&[], &valid, 0., 0.).unwrap().is_empty());
    assert!(deskew(&vec![ray(0., 0., 2.); 4097], &valid, 0., 0.).is_err());
}

// FEATURE: physical LDS deskew from the measured slot/revolution timing model.
// SPEC: beam times come from the measured revolution period and slot indices,
// never from sorted angles; rotation uses the SE(2) kernel and LDS lever arm.
fn traj(t0: f64, w: f64) -> Vec<TimedPose> {
    (0..=10)
        .map(|i| {
            let t = t0 + i as f64 * 0.02;
            pose(t, 0., 0., w * (t - t0))
        })
        .collect()
}

#[test]
fn rotating_scan_becomes_upright_with_measured_revolution_timing() {
    let period = 0.2;
    let n = 360;
    let w = 1.0f64;
    let stamp = 100.2; // publication right after the last beam
    let t0 = stamp - period;
    // A fixed world point straight ahead of the starting pose.
    let (wx, wy) = (2.0f64, 0.5);
    let rays: Vec<TimedRay> = (0..n)
        .map(|slot| {
            let time = stamp - (n - 1 - slot) as f64 * period / n as f64;
            let yaw = w * (time - t0);
            // Hit point in base_link at beam time, then the ray from the LDS.
            let bx = yaw.cos() * wx + yaw.sin() * wy;
            let by = -yaw.sin() * wx + yaw.cos() * wy;
            let rx = bx + 0.1;
            let ry = by;
            TimedRay {
                time,
                angle: ry.atan2(rx),
                range: rx.hypot(ry),
            }
        })
        .collect();
    let out = deskew_rays_from_slots(&rays, &traj(t0, w), stamp, period, -0.1).unwrap();
    for ray in &out {
        // Deskewed rays point at the same fixed world point from the LDS at the reference pose.
        let x = -0.1 + ray.range * ray.angle.cos();
        let y = ray.range * ray.angle.sin();
        let yaw = w * period;
        let rx = yaw.cos() * x - yaw.sin() * y;
        let ry = yaw.sin() * x + yaw.cos() * y;
        assert!((rx - wx).abs() < 1e-3, "x {} vs {wx}", rx);
        assert!((ry - wy).abs() < 1e-3, "y {} vs {wy}", ry);
        assert_eq!(ray.time, stamp);
    }
}

#[test]
fn slot_times_pair_with_array_position_not_angle_value() {
    // Same angle at the same slot must get the same compensated result,
    // independent of which angles occupy the other slots.
    let trajectory = &traj(0., 0.8);
    // Angle 0.1 sits at slot 0 in both arrays; only the neighbour differs.
    let a = deskew_rays_from_slots(
        &[ray(0., 0.1, 1.), ray(0., 1.7, 1.)],
        trajectory,
        0.2,
        0.2,
        -0.1,
    )
    .unwrap();
    let b = deskew_rays_from_slots(
        &[ray(0., 0.1, 1.), ray(0., -2.9, 1.)],
        trajectory,
        0.2,
        0.2,
        -0.1,
    )
    .unwrap();
    assert!((a[0].angle - b[0].angle).abs() < 1e-12);
    assert!((a[0].range - b[0].range).abs() < 1e-12);
    assert_eq!(a[0].time, b[0].time);
    // The kernel returns the common reference time for every corrected ray.
    assert_eq!(a[0].time, 0.2);
    // Rotation plus lever arm must actually change slot-0 vs slot-1 geometry.
    assert!((a[0].angle - a[1].angle - (0.1 - 1.7)).abs() > 1e-6);
}

#[test]
fn slot_model_rejects_uncovered_trajectory_or_bad_period() {
    let rays = vec![ray(0.05, 0., 1.), ray(0.1, 1., 1.)];
    assert!(deskew_rays_from_slots(&rays, &traj(0.15, 0.), 0.2, 0.2, -0.1).is_err());
    for bad in [0., -0.2, f64::NAN] {
        assert!(deskew_rays_from_slots(&rays, &traj(0.05, 0.), 0.2, bad, -0.1).is_err());
    }
    assert!(deskew_rays_from_slots(&rays, &traj(0.05, 0.), 0.2, 0.2, -0.1).is_ok());
}
