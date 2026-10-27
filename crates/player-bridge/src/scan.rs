//! Turn the vendor per-beam scan into the uniform grid a ROS LaserScan
//! describes (start, step, count), without ROS types.
//!
//! The vendor ruby_laser driver sends per-beam bearings in a wrapped,
//! descending order; sorting by bearing gives ascending directions. Only
//! uniformly spaced bearings can form a LaserScan without changing geometry.
//! Each beam keeps its own range and intensity through the sort.
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

/// Sort the vendor beams, rejecting scans that cannot form a uniform grid.
pub fn uniform_scan(scan: &LaserScanAngle) -> Result<UniformScan, &'static str> {
    if scan.angles.len() != scan.ranges.len() {
        return Err("bearing and range counts differ");
    }
    if scan.angles.len() < 2 {
        return Err("scan has no angular grid");
    }
    if scan.angles.iter().any(|a| !a.is_finite()) {
        return Err("nonfinite bearing");
    }
    if !scan.intensities.is_empty() && scan.intensities.len() != scan.ranges.len() {
        return Err("intensity and range counts differ");
    }
    let with_intensity = !scan.intensities.is_empty();
    let mut beams: Vec<(f32, f32, f32)> = scan
        .angles
        .iter()
        .zip(&scan.ranges)
        .enumerate()
        .map(|(i, (&a, &r))| {
            (
                a,
                r,
                if with_intensity {
                    scan.intensities[i] as f32
                } else {
                    0.0
                },
            )
        })
        .collect();
    beams.sort_by(|x, y| x.0.total_cmp(&y.0));
    let n = beams.len();
    let (angle_min, angle_max) = (beams[0].0, beams[n - 1].0);
    let angle_increment = (angle_max - angle_min) / (n - 1) as f32;
    if !angle_increment.is_finite() || angle_increment <= 0.0 {
        return Err("invalid angular increment");
    }
    if beams.windows(2).any(|pair| pair[0].0 >= pair[1].0) {
        return Err("duplicate bearing");
    }
    // Allow f32 rounding, not a different direction for an interior ray.
    if beams
        .iter()
        .enumerate()
        .any(|(i, beam)| (beam.0 - (angle_min + i as f32 * angle_increment)).abs() > 1e-5)
    {
        return Err("nonuniform bearings");
    }
    Ok(UniformScan {
        angle_min,
        angle_max,
        angle_increment,
        range_min: LDS_RANGE_MIN,
        range_max: scan.max_range,
        ranges: beams.iter().map(|b| b.1).collect(),
        intensities: if with_intensity {
            beams.iter().map(|b| b.2).collect()
        } else {
            Vec::new()
        },
    })
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

    // BUG: sorting explicit bearings does not make their spacing uniform.
    // HYPOTHESIS: averaging endpoint spacing silently changes interior directions.
    // SOLUTION: validate the grid before advertising it as a LaserScan.
    // RESULT: FIXED — invalid geometry is rejected; uniform ray bearings are preserved.
    // STATUS: FIXED

    #[test]
    fn nonuniform_bearings_are_rejected_instead_of_rotating_rays() {
        assert!(uniform_scan(&vendor(&[-0.2, 0.0, 0.3], &[1.0, 2.0, 3.0], &[])).is_err());
    }

    #[test]
    fn mismatched_bearing_and_range_counts_are_rejected() {
        assert!(uniform_scan(&vendor(&[-0.2, 0.0], &[1.0], &[])).is_err());
    }

    #[test]
    fn nonfinite_or_duplicate_bearings_are_rejected() {
        for angles in [[0.0, f32::NAN], [0.0, f32::INFINITY], [0.0, 0.0]] {
            assert!(uniform_scan(&vendor(&angles, &[1.0, 2.0], &[])).is_err());
        }
    }

    #[test]
    fn scans_without_an_angular_grid_are_rejected() {
        assert!(uniform_scan(&vendor(&[], &[], &[])).is_err());
        assert!(uniform_scan(&vendor(&[0.3], &[1.0], &[])).is_err());
    }

    #[test]
    fn mismatched_intensities_are_rejected() {
        assert!(uniform_scan(&vendor(&[0.0, 0.1], &[1.0, 2.0], &[10])).is_err());
    }

    #[test]
    fn complete_uniform_scan_preserves_all_bearings_and_intensities() {
        let angles: Vec<_> = (0..360)
            .rev()
            .map(|i| -std::f32::consts::PI + i as f32 * std::f32::consts::TAU / 360.0)
            .collect();
        let ranges: Vec<_> = (0..360).map(|i| 1.0 + i as f32 / 1000.0).collect();
        let intensities: Vec<_> = (0..360).map(|i| (i % 256) as u8).collect();
        let input = vendor(&angles, &ranges, &intensities);
        let output = uniform_scan(&input).unwrap();
        assert_eq!(output.ranges.len(), input.ranges.len());
        for (i, &range) in output.ranges.iter().enumerate() {
            let original = input.ranges.iter().position(|&r| r == range).unwrap();
            let bearing = output.angle_min + i as f32 * output.angle_increment;
            assert!((bearing - input.angles[original]).abs() < 1e-6);
            assert_eq!(output.intensities[i], input.intensities[original] as f32);
        }
    }

    #[test]
    fn wrapped_descending_bearings_become_an_ascending_grid() {
        // As the driver sends them: descending, wrapping past -pi to +pi.
        let s = uniform_scan(&vendor(
            &[-1.0, -2.0, -3.0, 3.0, 2.0, 1.0, 0.0],
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            &[],
        ))
        .unwrap();
        assert_eq!(s.angle_min, -3.0);
        assert_eq!(s.angle_max, 3.0);
        assert!((s.angle_increment - 1.0).abs() < 1e-6);
        assert_eq!(s.ranges, vec![3.0, 2.0, 1.0, 7.0, 6.0, 5.0, 4.0]);
        assert!(uniform_scan(&vendor(
            &[-1.0, -2.0, -3.0, 3.0, 2.0],
            &[1.0, 2.0, 3.0, 4.0, 5.0],
            &[],
        ))
        .is_err());
    }

    #[test]
    fn intensities_travel_with_their_beams() {
        let s = uniform_scan(&vendor(&[0.5, -0.5, 0.0], &[1.0, 2.0, 3.0], &[10, 20, 30])).unwrap();
        assert_eq!(s.ranges, vec![2.0, 3.0, 1.0]);
        assert_eq!(s.intensities, vec![20.0, 30.0, 10.0]);
    }

    #[test]
    fn missing_intensities_stay_empty() {
        let s = uniform_scan(&vendor(&[0.0, 1.0], &[1.0, 2.0], &[])).unwrap();
        assert!(s.intensities.is_empty());
    }

    #[test]
    fn empty_and_turret_beams_fall_below_range_min() {
        let s = uniform_scan(&vendor(&[0.0, 1.0, 2.0], &[0.0, 0.117, 2.5], &[])).unwrap();
        assert_eq!(s.range_min, LDS_RANGE_MIN);
        assert_eq!(s.range_max, 6.0);
        assert!(s.ranges[0] < s.range_min && s.ranges[1] < s.range_min);
        assert!(s.ranges[2] >= s.range_min);
    }

    #[test]
    fn degenerate_scans_have_no_grid() {
        assert_eq!(
            uniform_scan(&vendor(&[0.3], &[1.0], &[])),
            Err("scan has no angular grid")
        );
    }
}
