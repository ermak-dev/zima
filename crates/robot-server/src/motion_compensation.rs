#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}
#[derive(Debug, Clone, Copy)]
pub struct TimedPose {
    pub time: f64,
    pub pose: Pose2,
}
#[derive(Debug, Clone, Copy)]
pub struct TimedRay {
    pub time: f64,
    pub angle: f64,
    pub range: f64,
}
fn wrap(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
fn rotation_terms(angle: f64) -> (f64, f64) {
    if angle.abs() < 1e-7 {
        (1. - angle * angle / 6., angle / 2. - angle.powi(3) / 24.)
    } else {
        (angle.sin() / angle, (1. - angle.cos()) / angle)
    }
}
fn interpolate(trajectory: &[TimedPose], time: f64) -> Result<Pose2, &'static str> {
    if time < trajectory[0].time || time > trajectory[trajectory.len() - 1].time {
        return Err("trajectory_does_not_cover_beam_time");
    }
    let index = trajectory
        .partition_point(|sample| sample.time < time)
        .max(1);
    let a = trajectory[index - 1];
    let b = trajectory[index];
    let fraction = (time - a.time) / (b.time - a.time);
    let (s, c) = a.pose.yaw.sin_cos();
    let dx = c * (b.pose.x - a.pose.x) + s * (b.pose.y - a.pose.y);
    let dy = -s * (b.pose.x - a.pose.x) + c * (b.pose.y - a.pose.y);
    let angle = wrap(b.pose.yaw - a.pose.yaw);
    let (aa, bb) = rotation_terms(angle);
    let determinant = aa * aa + bb * bb;
    let vx = (aa * dx + bb * dy) / determinant;
    let vy = (-bb * dx + aa * dy) / determinant;
    let (ua, ub) = rotation_terms(angle * fraction);
    let x = fraction * (ua * vx - ub * vy);
    let y = fraction * (ub * vx + ua * vy);
    Ok(Pose2 {
        x: a.pose.x + c * x - s * y,
        y: a.pose.y + s * x + c * y,
        yaw: wrap(a.pose.yaw + fraction * angle),
    })
}

fn validate_trajectory(trajectory: &[TimedPose]) -> Result<(), &'static str> {
    if trajectory.len() < 2
        || trajectory.len() > 4096
        || trajectory.iter().any(|p| {
            ![p.time, p.pose.x, p.pose.y, p.pose.yaw]
                .iter()
                .all(|v| v.is_finite())
        })
        || trajectory
            .windows(2)
            .any(|w| w[0].time >= w[1].time || !(w[1].time - w[0].time).is_finite())
    {
        return Err("invalid_trajectory");
    }
    Ok(())
}

pub fn interpolate_pose(trajectory: &[TimedPose], time: f64) -> Result<Pose2, &'static str> {
    validate_trajectory(trajectory)?;
    if !time.is_finite() {
        return Err("invalid_reference_time");
    }
    interpolate(trajectory, time)
}

// Beam times and trajectory must share a verified acquisition clock. Player
// does not currently supply verified beam timing; never derive it from angles.
pub fn deskew(
    rays: &[TimedRay],
    trajectory: &[TimedPose],
    reference_time: f64,
    sensor_x: f64,
) -> Result<Vec<TimedRay>, &'static str> {
    if rays.len() > 4096 {
        return Err("invalid_deskew_input_size");
    }
    validate_trajectory(trajectory)?;
    if !reference_time.is_finite() || !sensor_x.is_finite() {
        return Err("invalid_reference_geometry");
    }
    let reference = interpolate(trajectory, reference_time)?;
    let (rs, rc) = reference.yaw.sin_cos();
    rays.iter()
        .map(|ray| {
            if ![ray.time, ray.angle, ray.range]
                .iter()
                .all(|v| v.is_finite())
                || ray.range <= 0.
            {
                return Err("invalid_timed_ray");
            }
            let pose = interpolate(trajectory, ray.time)?;
            let (s, c) = pose.yaw.sin_cos();
            let bx = sensor_x + ray.range * ray.angle.cos();
            let by = ray.range * ray.angle.sin();
            let wx = pose.x + c * bx - s * by - reference.x;
            let wy = pose.y + s * bx + c * by - reference.y;
            let x = rc * wx + rs * wy - sensor_x;
            let y = -rs * wx + rc * wy;
            let range = x.hypot(y);
            if !range.is_finite() || range <= 0. {
                return Err("invalid_corrected_ray");
            }
            Ok(TimedRay {
                time: reference_time,
                angle: y.atan2(x),
                range,
            })
        })
        .collect()
}

// Physical LDS timing model verified on the robot (PLAYER_TIMING.md): the
// driver fills slots 0..359 in acquisition order, one revolution spans
// `period` seconds, and the scan is published right after the last beam.
// Beam time therefore comes from the slot index — never from sorted angles.
// Slot k of N is assigned k*period/N seconds before the publication stamp;
// array order is irrelevant because times pair with slots, not neighbours.
pub fn deskew_rays_from_slots(
    rays: &[TimedRay],
    trajectory: &[TimedPose],
    reference_time: f64,
    period: f64,
    sensor_x: f64,
) -> Result<Vec<TimedRay>, &'static str> {
    if !period.is_finite() || period <= 0. {
        return Err("invalid_revolution_period");
    }
    let n = rays.len();
    if n == 0 || n > 4096 {
        return Err("invalid_deskew_input_size");
    }
    let timed: Vec<TimedRay> = rays
        .iter()
        .enumerate()
        .map(|(slot, ray)| TimedRay {
            time: reference_time - (n - 1 - slot) as f64 * period / n as f64,
            angle: ray.angle,
            range: ray.range,
        })
        .collect();
    deskew(&timed, trajectory, reference_time, sensor_x)
}
