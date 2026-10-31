//! Pure occupancy-grid utilities for serving bounded map regions (#58).
//!
//! The SLAM adapter publishes the grid as a JSON object
//! `{"x_min","y_min","res","w","h","cells"}` where `cells` is a CSV string of
//! occupancy values (`-1` = unknown, `0..=100` = free..occupied) in row-major
//! order starting at `(x_min, y_min)`. This module crops, downsamples and
//! renders such grids without touching I/O or the controller; every input is
//! treated as untrusted and size-bounded before any allocation.

use serde_json::{json, Value};

/// Hard cap on the cell count of any grid we parse or produce. Bounds the
/// response size (and the PNG buffer) for hostile inputs.
pub const MAX_CELLS: usize = 1 << 20;
/// Cap on any single grid dimension; checked before `w * h` is even computed.
pub const MAX_DIM: u64 = 1 << 16;
/// Occupancy value marking unknown space.
pub const UNKNOWN: i64 = -1;

pub type Result<T> = std::result::Result<T, String>;

struct Grid {
    x_min: f64,
    y_min: f64,
    res: f64,
    w: usize,
    h: usize,
    cells: Vec<i64>,
}

impl Grid {
    /// Parse and validate an adapter grid. Dimensions are bounded before the
    /// cells string is touched, so hostile `w`/`h` can never size an
    /// allocation; a wrong cells count is an error, not a truncation.
    fn parse(v: &Value) -> Result<Grid> {
        let num = |k: &str| -> Result<f64> {
            v.get(k)
                .and_then(Value::as_f64)
                .filter(|n| n.is_finite())
                .ok_or_else(|| format!("grid field {k} missing or not a finite number"))
        };
        let x_min = num("x_min")?;
        let y_min = num("y_min")?;
        let res = num("res")?;
        if res <= 0.0 {
            return Err("grid res must be positive".into());
        }
        let w = v.get("w").and_then(Value::as_u64).ok_or("grid w missing")?;
        let h = v.get("h").and_then(Value::as_u64).ok_or("grid h missing")?;
        if w > MAX_DIM || h > MAX_DIM {
            return Err(format!("grid w={} exceeds dimension limit", w.max(h)));
        }
        if w.checked_mul(h).ok_or("grid w*h overflow")? > MAX_CELLS as u64 {
            return Err(format!("grid has {} cells over limit {}", w * h, MAX_CELLS));
        }
        let cells_str = v
            .get("cells")
            .and_then(Value::as_str)
            .ok_or("grid cells missing or not a string")?;
        let mut cells = Vec::new();
        for part in cells_str.split(',') {
            // Count-then-parse: an oversized string is rejected before
            // allocating for it.
            if cells.len() >= w as usize * h as usize {
                return Err("grid cells count exceeds w*h".into());
            }
            let value: i64 = part
                .trim()
                .parse()
                .map_err(|_| format!("invalid cell value {part:?}"))?;
            if value != UNKNOWN && !(0..=100).contains(&value) {
                return Err(format!("cell value {value} outside -1..=100"));
            }
            cells.push(value);
        }
        if cells.len() != w as usize * h as usize {
            return Err(format!(
                "grid cells count {} does not match w*h {}",
                cells.len(),
                w * h
            ));
        }
        Ok(Grid {
            x_min,
            y_min,
            res,
            w: w as usize,
            h: h as usize,
            cells,
        })
    }

    fn get(&self, x: usize, y: usize) -> i64 {
        self.cells[y * self.w + x]
    }

    fn to_json(&self) -> Value {
        let mut cells = String::new();
        for (i, c) in self.cells.iter().enumerate() {
            if i > 0 {
                cells.push(',');
            }
            cells.push_str(&c.to_string());
        }
        json!({
            "x_min": self.x_min,
            "y_min": self.y_min,
            "res": self.res,
            "w": self.w,
            "h": self.h,
            "cells": cells,
        })
    }
}

/// Extract the region within `radius` meters of `(cx, cy)`. The crop is
/// explicit: output `x_min`/`y_min`/`w`/`h` describe the exact rectangle
/// returned, cells outside the known area are unknown, and origin/resolution
/// are preserved.
pub fn crop_map(grid: &Value, region: (f64, f64, f64)) -> Result<Value> {
    let (cx, cy, radius) = region;
    if !radius.is_finite() || !cx.is_finite() || !cy.is_finite() || radius <= 0.0 {
        return Err("region radius and center must be finite, radius positive".into());
    }
    let g = Grid::parse(grid)?;
    // Bounded rectangle: cells the circle can touch at all, snapped outward
    // to the cell grid; indices may run outside the known area, where cells
    // are reported as unknown. The cell count is checked before allocating.
    let span = |center: f64, origin: f64| -> (f64, f64) {
        let lo = ((center - radius - origin) / g.res).floor();
        let hi = ((center + radius - origin) / g.res).ceil();
        (lo.min(hi), hi.max(lo))
    };
    let (x0, x1) = span(cx, g.x_min);
    let (y0, y1) = span(cy, g.y_min);
    let w = (x1 - x0).max(0.0) as usize;
    let h = (y1 - y0).max(0.0) as usize;
    if w * h > MAX_CELLS {
        return Err(format!(
            "crop region {}x{} cells over limit {}",
            w, h, MAX_CELLS
        ));
    }
    let mut cells = Vec::with_capacity(w * h);
    for row in 0..h {
        for col in 0..w {
            let gx = x0 + col as f64;
            let gy = y0 + row as f64;
            let value = if gx >= 0.0 && gy >= 0.0 && gx < g.w as f64 && gy < g.h as f64 {
                g.get(gx as usize, gy as usize)
            } else {
                UNKNOWN
            };
            cells.push(value);
        }
    }
    Ok(Grid {
        x_min: g.x_min + x0 * g.res,
        y_min: g.y_min + y0 * g.res,
        res: g.res,
        w,
        h,
        cells,
    }
    .to_json())
}

/// Thin `factor`-by-`factor` blocks to one cell: a block containing any
/// unknown stays unknown, otherwise the worst (highest) occupancy wins.
/// Resolution and origin are scaled so the covered area is unchanged.
pub fn downsample(grid: &Value, factor: u32) -> Result<Value> {
    if factor == 0 {
        return Err("downsample factor must be >= 1".into());
    }
    let g = Grid::parse(grid)?;
    let f = factor as usize;
    let w = (g.w + f - 1) / f;
    let h = (g.h + f - 1) / f;
    let mut cells = Vec::with_capacity(w * h);
    for by in 0..h {
        for bx in 0..w {
            let mut value: Option<i64> = None;
            'cell: for dy in 0..f {
                for dx in 0..f {
                    let (x, y) = (bx * f + dx, by * f + dy);
                    if x < g.w && y < g.h {
                        let c = g.get(x, y);
                        if c == UNKNOWN {
                            value = Some(UNKNOWN);
                            break 'cell;
                        }
                        value = Some(value.map_or(c, |m: i64| m.max(c)));
                    }
                }
            }
            cells.push(value.unwrap_or(UNKNOWN));
        }
    }
    Ok(Grid {
        x_min: g.x_min,
        y_min: g.y_min,
        res: g.res * f as f64,
        w,
        h,
        cells,
    }
    .to_json())
}

/// Short-circuit answer for `known_version` clients: true means the map has
/// not changed and no payload needs to be rendered.
pub fn not_modified(known_version: u64, current_version: u64) -> bool {
    known_version == current_version
}

/// Render the grid as an 8-bit RGB PNG. Unknown space gets its own color,
/// free-to-occupied space is shaded, and `robot_pose` (map frame, meters,
/// yaw in radians) draws a marker only when explicitly supplied and finite.
pub fn render_png(grid: &Value, robot_pose: Option<(f64, f64, f64)>) -> Result<Vec<u8>> {
    let g = Grid::parse(grid)?;
    let pose_xy: Option<(f64, f64, f64)> = match robot_pose {
        Some((x, y, yaw)) => {
            if !x.is_finite() || !y.is_finite() || !yaw.is_finite() {
                return Err("robot pose must be finite".into());
            }
            Some(((x - g.x_min) / g.res, (y - g.y_min) / g.res, yaw))
        }
        None => None,
    };
    // Rows draw bottom-up so the image has a math-like y-up orientation.
    let mut rgb = vec![0u8; g.w * g.h * 3];
    for row in 0..g.h {
        for col in 0..g.w {
            let src = g.get(col, g.h - 1 - row);
            let dst = (row * g.w + col) * 3;
            let color = match src {
                UNKNOWN => [96, 96, 96],
                v if v >= 65 => [20, 20, 20],
                v if v == 0 => [235, 235, 235],
                _ => [150, 150, 150],
            };
            rgb[dst..dst + 3].copy_from_slice(&color);
        }
    }
    // Pose marker: a cross plus a short heading line, clipped to the image.
    if let Some((px, py, yaw)) = pose_xy {
        let (px, py) = (px as isize, (g.h as f64 - 1.0 - py) as isize);
        let (ix, iy) = (px as i64, py as i64);
        let plot = |rgb: &mut [u8], x: i64, y: i64| {
            if x >= 0 && y >= 0 && (x as usize) < g.w && (y as usize) < g.h {
                let dst = (y as usize * g.w + x as usize) * 3;
                rgb[dst..dst + 3].copy_from_slice(&[255, 60, 0]);
            }
        };
        for d in -2..=2 {
            plot(&mut rgb, ix + d, iy);
            plot(&mut rgb, ix, iy + d);
        }
        let (dx, dy) = (yaw.cos(), -yaw.sin());
        for step in 1..=4 {
            plot(
                &mut rgb,
                ix + (dx * step as f64).round() as i64,
                iy + (dy * step as f64).round() as i64,
            );
        }
    }
    let mut png_bytes = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut png_bytes, g.w as u32, g.h as u32);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        encoder
            .write_header()
            .map_err(|e| format!("png header: {e}"))?
            .write_image_data(&rgb)
            .map_err(|e| format!("png data: {e}"))?;
    }
    Ok(png_bytes)
}

/// Describe the grid for map clients: frame, origin, resolution, bounds and
/// the pixel-to-map transform for a `render_png` image (y-up, one pixel per
/// cell at `res` meters).
pub fn metadata(grid: &Value) -> Result<Value> {
    let g = Grid::parse(grid)?;
    Ok(json!({
        "frame": "map",
        "origin": [g.x_min, g.y_min],
        "resolution": g.res,
        "size": {"w": g.w, "h": g.h},
        "bounds": {
            "min": [g.x_min, g.y_min],
            "max": [g.x_min + g.w as f64 * g.res, g.y_min + g.h as f64 * g.res],
        },
        "png": {
            "width": g.w,
            "height": g.h,
            "pixel_size_m": g.res,
            "pixel_to_map": {
                "x": "x_min + col * res",
                "y": "y_min + (height - 1 - row) * res",
            },
        },
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    /// 4x2 grid, res 0.5 m, origin (1.0, 2.0): row-major, y grows with row.
    fn grid_4x2() -> Value {
        json!({
            "x_min": 1.0, "y_min": 2.0, "res": 0.5, "w": 4, "h": 2,
            "cells": "0,50,100,-1,100,0,-1,50"
        })
    }

    #[test]
    fn crop_inside_preserves_cells_and_origin() {
        // Circle around the map middle (2.0, 2.5), r = 0.5 m: covers columns
        // 1..=2 and both rows exactly, snapping outward to the cell grid.
        let out = crop_map(&grid_4x2(), (2.0, 2.5, 0.5)).unwrap();
        assert_eq!(out["x_min"], json!(1.5));
        assert_eq!(out["y_min"], json!(2.0));
        assert_eq!(out["res"], json!(0.5));
        assert_eq!(out["w"], json!(2));
        assert_eq!(out["h"], json!(2));
        assert_eq!(out["cells"], json!("50,100,0,-1"));
    }

    #[test]
    fn crop_partly_outside_marks_unknown() {
        // Circle centered on the left edge of the map: column -1 lies
        // outside the known area and must come back as unknown.
        let out = crop_map(&grid_4x2(), (1.0, 2.5, 0.5)).unwrap();
        assert_eq!(out["x_min"], json!(0.5));
        assert_eq!(out["w"], json!(2));
        assert_eq!(out["h"], json!(2));
        assert_eq!(out["cells"], json!("-1,0,-1,100"));
    }

    #[test]
    fn crop_disjoint_region_is_all_unknown() {
        let out = crop_map(&grid_4x2(), (50.0, 50.0, 1.0)).unwrap();
        assert!(out["w"].as_u64().unwrap() > 0);
        assert!(out["h"].as_u64().unwrap() > 0);
        let cells = out["cells"].as_str().unwrap();
        assert!(cells.split(',').all(|c| c == "-1"));
    }

    #[test]
    fn crop_rejects_bad_radius() {
        assert!(crop_map(&grid_4x2(), (2.0, 3.0, 0.0)).is_err());
        assert!(crop_map(&grid_4x2(), (2.0, 3.0, -1.0)).is_err());
        assert!(crop_map(&grid_4x2(), (f64::NAN, 3.0, 1.0)).is_err());
    }

    #[test]
    fn crop_enforces_cell_limit() {
        // r = 1000 m at res 0.05 would need ~1600x10^6 cells.
        let err = crop_map(&grid_4x2(), (2.0, 3.0, 1000.0)).unwrap_err();
        assert!(err.contains("limit"), "unexpected error: {err}");
    }

    #[test]
    fn downsample_merges_blocks_and_keeps_unknown() {
        // factor 2: each 2x1 block collapses (h=2 -> 1). A block holding an
        // unknown cell stays unknown; otherwise the max (worst obstacle) wins.
        let out = downsample(&grid_4x2(), 2).unwrap();
        assert_eq!(out["w"], json!(2));
        assert_eq!(out["h"], json!(1));
        assert_eq!(out["res"], json!(1.0));
        assert_eq!(out["x_min"], json!(1.0));
        assert_eq!(out["cells"], json!("100,-1"));
    }

    #[test]
    fn downsample_partial_edge_block() {
        // w=5 with factor 2 leaves a single-cell edge block.
        let grid = json!({
            "x_min": 0.0, "y_min": 0.0, "res": 0.1, "w": 5, "h": 1,
            "cells": "10,20,30,40,100"
        });
        let out = downsample(&grid, 2).unwrap();
        assert_eq!(out["w"], json!(3));
        assert_eq!(out["cells"], json!("20,40,100"));
    }

    #[test]
    fn downsample_factor_one_is_identity_and_zero_is_error() {
        let grid = grid_4x2();
        let out = downsample(&grid, 1).unwrap();
        assert_eq!(out["cells"], grid["cells"]);
        assert_eq!(out["w"], grid["w"]);
        assert!(downsample(&grid, 0).is_err());
    }

    #[test]
    fn hostile_dimensions_are_counted_before_allocation() {
        // w*h overflows the cell cap; cells claims far fewer entries, so the
        // parse must fail on metadata alone, never on the cells string.
        let hostile = json!({
            "x_min": 0.0, "y_min": 0.0, "res": 0.05,
            "w": 4_000_000_000_u64, "h": 4_000_000_000_u64,
            "cells": "1,2,3"
        });
        let err = crop_map(&hostile, (0.0, 0.0, 1.0)).unwrap_err();
        assert!(err.contains("w"), "unexpected error: {err}");
        // Same for a huge but non-overflowing product.
        let hostile = json!({
            "x_min": 0.0, "y_min": 0.0, "res": 0.05,
            "w": 100_000_u64, "h": 100_000_u64,
            "cells": "1"
        });
        assert!(crop_map(&hostile, (0.0, 0.0, 1.0)).is_err());
    }

    #[test]
    fn hostile_cells_length_must_match_wh() {
        let hostile = json!({
            "x_min": 0.0, "y_min": 0.0, "res": 0.05, "w": 2, "h": 1,
            "cells": "50,-1,70,80,90"
        });
        let err = downsample(&hostile, 1).unwrap_err();
        assert!(err.contains("cells"), "unexpected error: {err}");
        let hostile = json!({
            "x_min": 0.0, "y_min": 0.0, "res": 0.05, "w": 2, "h": 1,
            "cells": "50"
        });
        assert!(downsample(&hostile, 1).is_err());
    }

    #[test]
    fn hostile_fields_are_rejected() {
        let base = grid_4x2();
        for field in ["x_min", "y_min", "res", "w", "h", "cells"] {
            let mut bad = base.clone();
            bad[field] = json!("not-a-number");
            assert!(crop_map(&bad, (0.0, 0.0, 1.0)).is_err(), "field {field}");
        }
        let mut bad = base.clone();
        bad["res"] = json!(0.0);
        assert!(crop_map(&bad, (0.0, 0.0, 1.0)).is_err());
        let mut bad = base.clone();
        bad["cells"] = json!("0,101,0,-1,0,0,0,0");
        assert!(crop_map(&bad, (0.0, 0.0, 1.0)).is_err());
        assert!(crop_map(&json!("nope"), (0.0, 0.0, 1.0)).is_err());
    }

    #[test]
    fn not_modified_compares_versions() {
        assert!(not_modified(7, 7));
        assert!(!not_modified(7, 8));
        assert!(!not_modified(0, 7));
    }

    #[test]
    fn render_png_signature_and_dimensions() {
        let png = render_png(&grid_4x2(), None).unwrap();
        assert_eq!(
            &png[..8],
            &[0x89, b'P', b'N', b'G', b'\r', b'\n', 0x1a, b'\n']
        );
        // IHDR: length(4) + "IHDR"(4) then big-endian width/height.
        assert_eq!(&png[12..16], b"IHDR");
        let w = u32::from_be_bytes([png[16], png[17], png[18], png[19]]);
        let h = u32::from_be_bytes([png[20], png[21], png[22], png[23]]);
        assert_eq!((w, h), (4, 2));
    }

    #[test]
    fn render_png_pose_changes_output_and_stays_valid() {
        let plain = render_png(&grid_4x2(), None).unwrap();
        let posed = render_png(&grid_4x2(), Some((2.0, 3.0, 0.0))).unwrap();
        assert_eq!(&posed[..8], &plain[..8]);
        assert_ne!(plain, posed);
        // A pose far outside the grid is still renderable (clipped marker).
        assert!(render_png(&grid_4x2(), Some((99.0, 99.0, 1.0))).is_ok());
        assert!(render_png(&grid_4x2(), Some((f64::NAN, 0.0, 0.0))).is_err());
    }

    #[test]
    fn render_png_rejects_hostile_grid() {
        let hostile = json!({
            "x_min": 0.0, "y_min": 0.0, "res": 0.05,
            "w": 100_000_u64, "h": 100_000_u64, "cells": "1"
        });
        assert!(render_png(&hostile, None).is_err());
    }

    #[test]
    fn metadata_reports_frame_origin_bounds_and_pixel_transform() {
        let meta = metadata(&grid_4x2()).unwrap();
        assert_eq!(meta["frame"], json!("map"));
        assert_eq!(meta["origin"], json!([1.0, 2.0]));
        assert_eq!(meta["resolution"], json!(0.5));
        assert_eq!(
            meta["bounds"],
            json!({"min": [1.0, 2.0], "max": [3.0, 3.0]})
        );
        assert_eq!(meta["size"], json!({"w": 4, "h": 2}));
        assert_eq!(meta["png"]["width"], json!(4));
        assert_eq!(meta["png"]["height"], json!(2));
        assert_eq!(meta["png"]["pixel_size_m"], json!(0.5));
        assert_eq!(meta["png"]["pixel_to_map"]["x"], json!("x_min + col * res"));
        assert_eq!(
            meta["png"]["pixel_to_map"]["y"],
            json!("y_min + (height - 1 - row) * res")
        );
    }

    #[test]
    fn metadata_rejects_hostile_grid() {
        assert!(metadata(&json!({"w": 1})).is_err());
        let hostile = json!({
            "x_min": 0.0, "y_min": 0.0, "res": 0.05,
            "w": 100_000_u64, "h": 100_000_u64, "cells": "1"
        });
        assert!(metadata(&hostile).is_err());
    }
}
