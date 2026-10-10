//! Turn the vendor per-beam scan into the uniform grid a ROS LaserScan
//! describes (start, step, count), without ROS types.
//!
//! The vendor ruby_laser driver sends per-beam bearings in a wrapped,
//! descending order; sorting by bearing gives the ascending grid. Each
//! beam keeps its own range and intensity through the sort.
//!
//! Empty beams arrive as 0.0, and a few beams always read 0.11-0.15 m
//! (the turret's own structure; measured on the live session bag). The
//! LDS cannot measure that close anyway, and anything nearer than
//! LDS_RANGE_MIN to the turret is inside the vacuum's body, so the scan
//! advertises that range_min: by REP 117 every consumer (costmaps, the
//! collision monitor, AMCL, slam_toolbox) then drops those beams instead
//! of seeing an obstacle inside the robot.

use crate::LaserScanAngle;

/// Shortest range the scan reports as a measurement, m.
pub const LDS_RANGE_MIN: f32 = 0.15;

/// A scan on a uniform ascending bearing grid.
#[derive(Debug, Clone, PartialEq)]
pub struct UniformScan {
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<f32>,
}

/// Sort the vendor beams by bearing and describe them as a uniform grid.
pub fn uniform_scan(scan: &LaserScanAngle) -> UniformScan {
    let with_intensity = scan.intensities.len() == scan.ranges.len();
    let mut beams: Vec<(f32, f32, f32)> = scan
        .angles
        .iter()
        .zip(&scan.ranges)
        .enumerate()
        .map(|(i, (&a, &r))| (a, r, if with_intensity { scan.intensities[i] as f32 } else { 0.0 }))
        .collect();
    beams.sort_by(|x, y| x.0.total_cmp(&y.0));
    let n = beams.len();
    let (angle_min, angle_max, angle_increment) = if n >= 2 {
        let (first, last) = (beams[0].0, beams[n - 1].0);
        (first, last, (last - first) / (n - 1) as f32)
    } else {
        (0.0, 0.0, 0.0)
    };
    UniformScan {
        angle_min,
        angle_max,
        angle_increment,
        range_min: LDS_RANGE_MIN,
        range_max: scan.max_range,
        ranges: beams.iter().map(|b| b.1).collect(),
        intensities: if with_intensity { beams.iter().map(|b| b.2).collect() } else { Vec::new() },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vendor(angles: &[f32], ranges: &[f32], intensities: &[u8]) -> LaserScanAngle {
        LaserScanAngle {
            max_range: 6.0,
            ranges: ranges.to_vec(),
            angles: angles.to_vec(),
            intensities: intensities.to_vec(),
            id: 0,
        }
    }

    #[test]
    fn wrapped_descending_bearings_become_an_ascending_grid() {
        // As the driver sends them: descending, wrapping past -pi to +pi.
        let s = uniform_scan(&vendor(&[-1.0, -2.0, -3.0, 3.0, 2.0], &[1.0, 2.0, 3.0, 4.0, 5.0], &[]));
        assert_eq!(s.angle_min, -3.0);
        assert_eq!(s.angle_max, 3.0);
        assert!((s.angle_increment - 1.5).abs() < 1e-6);
        assert_eq!(s.ranges, vec![3.0, 2.0, 1.0, 5.0, 4.0]);
    }

    #[test]
    fn intensities_travel_with_their_beams() {
        let s = uniform_scan(&vendor(&[0.5, -0.5, 0.0], &[1.0, 2.0, 3.0], &[10, 20, 30]));
        assert_eq!(s.ranges, vec![2.0, 3.0, 1.0]);
        assert_eq!(s.intensities, vec![20.0, 30.0, 10.0]);
    }

    #[test]
    fn missing_intensities_stay_empty() {
        let s = uniform_scan(&vendor(&[0.0, 1.0], &[1.0, 2.0], &[]));
        assert!(s.intensities.is_empty());
    }

    #[test]
    fn empty_and_turret_beams_fall_below_range_min() {
        let s = uniform_scan(&vendor(&[0.0, 1.0, 2.0], &[0.0, 0.117, 2.5], &[]));
        assert_eq!(s.range_min, LDS_RANGE_MIN);
        assert_eq!(s.range_max, 6.0);
        assert!(s.ranges[0] < s.range_min && s.ranges[1] < s.range_min);
        assert!(s.ranges[2] >= s.range_min);
    }

    #[test]
    fn degenerate_scans_have_no_grid() {
        let s = uniform_scan(&vendor(&[0.3], &[1.0], &[]));
        assert_eq!((s.angle_min, s.angle_max, s.angle_increment), (0.0, 0.0, 0.0));
        assert_eq!(s.ranges, vec![1.0]);
    }
}
