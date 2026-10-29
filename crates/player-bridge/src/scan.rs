//! LDS measurements below this range are inside the vacuum body or turret.

pub const LDS_RANGE_MIN: f32 = 0.15;

/// LDS turret position along base_link x [m] (behind the body center).
pub const LDS_X: f64 = -0.1;

/// Additional yaw for published bearings [deg]. The vendor driver already
/// includes the configured mounting yaw; only the origin needs translation.
pub const LDS_YAW_DEG: f64 = 0.0;
