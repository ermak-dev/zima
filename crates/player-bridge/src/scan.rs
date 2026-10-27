//! LDS measurements below this range are inside the vacuum body or turret.

pub const LDS_RANGE_MIN: f32 = 0.15;

/// LDS turret position along base_link x [m] (behind the body center).
pub const LDS_X: f64 = -0.1;

/// Yaw added to raw LDS bearings to express them in base_link [deg].
pub const LDS_YAW_DEG: f64 = 97.6;
