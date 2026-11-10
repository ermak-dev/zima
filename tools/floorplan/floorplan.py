#!/usr/bin/env python3
"""Offline conversion of the SLAM occupancy grid into a polygonal floorplan.

Input: the grid object used by the MRPT adapter ({"op":"map"}) and by
robot-server map frames: {"x_min","y_min","res","w","h","cells"} where
cells is a CSV of occupancy percents (-1 unknown, 0 free, 100 wall).

Output structure (in metres, map coordinates):
  walls     [{"kind":"wall","a":[x,y],"b":[x,y]}]        axis-aligned after
            dominant-direction de-rotation
  rooms     [{"polygon":[(x,y)...],"area_m2","edges":[{"a","b","observed"}]}]
  doors     [{"a":[x,y],"b":[x,y],"width_m"}]            gaps in wall lines
            that separate two different rooms
  obstacles [{"polygon":[(x,y)...]}]                     unexplained occupied
            components (furniture), never walls or room boundaries
  report    coverage metrics (explained wall cells, distance percentiles,
            unobserved perimeter share)

Unknown cells never become walls or free space: wall gaps with unknown
territory behind stay unobserved room edges, never invented walls.

numpy/scipy only; no robot, no network, no writes unless requested.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy import ndimage

MAX_CELLS = 1 << 20
MAX_DIM = 1 << 16
OCC_TH = 65
FREE_TH = 35
WALL_MIN_RUN = 4            # cells; shorter runs are not walls
WALL_COLUMN_SHARE = 0.15    # column is a wall when sum >= share of the max
NOISE_COMPONENT = 4         # occupied components below this are noise
OBSTACLE_MIN_CELLS = 8
DOOR_MIN_M = 0.45
DOOR_MAX_M = 1.00
BAR_M = 0.45                # openings up to 2*BAR_M separate rooms
MIN_ROOM_M2 = 0.5
GROW_MAX_M = 0.5            # regrow cap from the core, blocks spill through doors
RDP_EPS_CELLS = 2.5
SNAP_MAX_M = 0.35


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def parse_grid(value):
    """Strict parse of the adapter grid with map_view.rs-sized limits."""
    for key in ('x_min', 'y_min', 'res', 'w', 'h', 'cells'):
        _require(key in value, f'grid field missing: {key}')
    try:
        x_min = float(value['x_min'])
        y_min = float(value['y_min'])
        res = float(value['res'])
        w = int(value['w'])
        h = int(value['h'])
    except (TypeError, ValueError):
        raise ValueError('grid geometry must be numeric') from None
    _require(math.isfinite(x_min) and math.isfinite(y_min), 'grid origin not finite')
    _require(math.isfinite(res) and res > 0, 'grid resolution must be positive')
    _require(1 <= w <= MAX_DIM and 1 <= h <= MAX_DIM, f'grid dimensions out of range: {w}x{h}')
    _require(w * h <= MAX_CELLS, f'grid too large: {w * h} cells')
    text = value['cells']
    _require(isinstance(text, str), 'cells must be a CSV string')
    parts = text.split(',')
    _require(len(parts) == w * h, f'cells length {len(parts)} != w*h {w * h}')
    try:
        data = np.array([int(p) for p in parts], dtype=np.int64)
    except ValueError:
        raise ValueError('cells must be integers') from None
    _require(bool(((data >= -1) & (data <= 100)).all()), 'cells out of -1..100')
    return x_min, y_min, res, data.reshape(h, w)


def _dominant_angle(occ):
    """Wall direction mod 90°: rotation that best collapses the occupied
    cells onto axis-parallel lines (minimal spread of coordinate histograms).

    Sobel-gradient means are biased on rasterized rotated lines; projecting
    the cells themselves is exact for thick raster walls.
    """
    if occ.sum() < 50:
        return 0.0
    ys, xs = np.nonzero(occ)
    wx = (xs + 0.5) * 1.0
    wy = (ys + 0.5) * 1.0
    best_theta = 0.0
    best_score = None
    for theta in np.arange(0.0, 90.0, 0.25):
        c = math.cos(math.radians(theta))
        s = math.sin(math.radians(theta))
        xr = wx * c + wy * s
        yr = -wx * s + wy * c
        score = _axis_peak(wx, wy, theta)
        if best_score is None or score > best_score:
            best_score = score
            best_theta = float(theta)
    # local refinement around the coarse peak
    for theta in np.arange(best_theta - 0.25, best_theta + 0.26, 0.05):
        score = _axis_peak(wx, wy, theta)
        if score > best_score:
            best_score = score
            best_theta = float(theta)
    return best_theta


def _axis_peak(wx, wy, theta):
    """Sum of squared top histogram peaks of occupied-cell coordinates in
    the frame rotated by theta; maximal when walls collapse onto axes."""
    c = math.cos(math.radians(theta))
    s = math.sin(math.radians(theta))
    xr = wx * c + wy * s
    yr = -wx * s + wy * c

    def peak(v):
        bins = np.floor(v).astype(np.int64)
        counts = np.bincount(bins - bins.min())
        top = np.sort(counts)[-8:]
        return float((top.astype(float) ** 2).sum())

    return peak(xr) + peak(yr)

def _axis_walls(occ, occ_major, res, x_min, y_min, theta):
    """Axis wall segments in the de-rotated frame.

    Returns (segments, bins_offset, bx_min, by_min) where segments are
    (axis, const_bin, lo_bin, hi_bin) — axis 0: vertical wall (constant X),
    axis 1: horizontal wall (constant Y).
    """
    H, W = occ.shape
    ys, xs = np.nonzero(occ_major)
    wx = x_min + (xs + 0.5) * res
    wy = y_min + (ys + 0.5) * res
    cos_t, sin_t = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    xr = wx * cos_t + wy * sin_t
    yr = -wx * sin_t + wy * cos_t
    bx1 = np.floor(xr / res).astype(np.int64)
    by1 = np.floor(yr / res).astype(np.int64)
    # Full rotated-coordinate grids so callers can index by (row, col).
    bx = np.zeros(occ.shape, dtype=np.int64)
    by = np.zeros(occ.shape, dtype=np.int64)
    bx[ys, xs] = bx1
    by[ys, xs] = by1
    segments = []
    # Only cells whose local gradient is perpendicular to the sought wall
    # direction: otherwise a horizontal wall fills every x-column and the
    # median merging collapses all columns into one line.
    gxf = ndimage.sobel(occ.astype(float), axis=1)
    gyf = ndimage.sobel(occ.astype(float), axis=0)
    for axis, bins, other in ((0, bx1, by1), (1, by1, bx1)):
        if axis == 0:
            sel = np.abs(gxf[ys, xs]) >= np.abs(gyf[ys, xs])
        else:
            sel = np.abs(gyf[ys, xs]) > np.abs(gxf[ys, xs])
        bins = bins[sel]
        other = other[sel]
        if not len(bins):
            continue
        # Walls can be several cells thick (lidar + resolution): merge
        # perpendicular bins within 3 cells into their median line first.
        uniq = np.unique(bins)
        groups = []
        group = [uniq[0]]
        for v in uniq[1:]:
            if v - group[-1] <= 3:
                group.append(v)
            else:
                groups.append(group)
                group = [v]
        groups.append(group)
        mapping = {}
        for g in groups:
            rep = int(np.median(g))
            for v in g:
                mapping[int(v)] = rep
        bins = np.array([mapping[int(v)] for v in bins], dtype=np.int64)
        lo = int(bins.min())
        counts = np.bincount(bins - lo)
        threshold = max(WALL_MIN_RUN, WALL_COLUMN_SHARE * counts.max())
        for c in np.nonzero(counts >= threshold)[0]:
            line = c + lo
            along = other[bins == line]
            along = np.sort(np.unique(along))
            # split into runs with a gap tolerance of 2 cells
            run_start = along[0]
            prev = along[0]
            run_vals = [along[0]]
            for v in along[1:]:
                if v - prev <= 3:  # gap of <=2 empty bins keeps the run
                    run_vals.append(v)
                else:
                    if len(run_vals) >= WALL_MIN_RUN:
                        segments.append((axis, line, run_vals[0], run_vals[-1]))
                    run_vals = [v]
                prev = v
            if len(run_vals) >= WALL_MIN_RUN:
                segments.append((axis, line, run_vals[0], run_vals[-1]))
    return segments, cos_t, sin_t, bx, by


def _rot_to_world(xr, yr, cos_t, sin_t):
    return (xr * cos_t - yr * sin_t, xr * sin_t + yr * cos_t)


def _segments_world(segments, cos_t, sin_t, res):
    """Wall segments as world-coordinate dicts with kind/observed."""
    out = []
    for axis, line, lo, hi in segments:
        const = (line + 0.5) * res
        a0 = lo * res
        b0 = (hi + 1) * res
        if axis == 0:
            ax, ay = _rot_to_world(const, a0, cos_t, sin_t)
            bx, by = _rot_to_world(const, b0, cos_t, sin_t)
        else:
            ax, ay = _rot_to_world(a0, const, cos_t, sin_t)
            bx, by = _rot_to_world(b0, const, cos_t, sin_t)
        out.append({'kind': 'wall', 'a': [ax, ay], 'b': [bx, by],
                    'observed': True})
    return out


def _explained_mask(occ_major, segments, bx, by, res):
    """Occupied cells close to an extracted wall segment (rotated frame)."""
    mask = np.zeros(occ_major.shape, dtype=bool)
    if not len(segments):
        return mask, np.full(occ_major.shape, np.inf)
    ys, xs = np.nonzero(occ_major)
    xr = (bx[ys, xs] + 0.5) * res
    yr = (by[ys, xs] + 0.5) * res
    dist = np.full(xr.shape, np.inf)
    for axis, line, lo, hi in segments:
        const = (line + 0.5) * res
        a0, b0 = lo * res, (hi + 1) * res
        if axis == 0:
            dx = np.abs(xr - const)
            dy = np.maximum(a0 - yr, yr - b0)
        else:
            dy = np.abs(yr - const)
            dx = np.maximum(a0 - xr, xr - b0)
        d = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
        dist = np.minimum(dist, d)
    near = dist <= res * 1.5
    mask[ys[near], xs[near]] = True
    return mask, dist


def _trace_loops(mask):
    """Closed boundary loops of a boolean mask as cell-lattice corner chains.

    Directed edges keep the region on the left; at vertices with several
    outgoing options the most clockwise turn is taken so each boundary is
    followed in one connected cycle (outer loop counter-clockwise in y-up).
    """
    m = mask
    top = m & ~np.pad(m, ((1, 0), (0, 0)))[:-1]
    bottom = m & ~np.pad(m, ((0, 1), (0, 0)))[1:]
    left = m & ~np.pad(m, ((0, 0), (1, 0)))[:, :-1]
    right = m & ~np.pad(m, ((0, 0), (0, 1)))[:, 1:]
    edges = {}

    def add(p, q):
        edges.setdefault(p, []).append(q)

    for y, x in zip(*np.nonzero(top)):
        # World bottom boundary of the cell: lattice row y, eastward
        # (region stays on the left in y-up coordinates).
        add((x, y), (x + 1, y))
    for y, x in zip(*np.nonzero(bottom)):
        # World top boundary: lattice row y+1, westward.
        add((x + 1, y + 1), (x, y + 1))
    for y, x in zip(*np.nonzero(left)):
        add((x, y + 1), (x, y))
    for y, x in zip(*np.nonzero(right)):
        add((x + 1, y), (x + 1, y + 1))

    # Vertex order for the clockwise choice: walk candidate outgoing edges
    # starting just after the reversed incoming direction.
    dirs = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]

    def next_edge(cur, inc, used):
        dx, dy = inc
        rev = (-dx, -dy)
        idx = dirs.index(rev)
        for k in range(8):
            d = dirs[(idx + 1 + k) % 8]
            q = (cur[0] + d[0], cur[1] + d[1])
            if q in edges.get(cur, ()) and (cur, q) not in used:
                return q
        return None

    loops = []
    used = set()
    for start in list(edges.keys()):
        if any((start, q) in used for q in edges.get(start, [])):
            continue
        loop = [start]
        cur = start
        inc = None
        local = set()
        closed = False
        while True:
            options = [
                q for q in edges.get(cur, [])
                if (cur, q) not in used and (cur, q) not in local
            ]
            if not options:
                break
            if inc is not None:
                nxt = next_edge(cur, inc, used | local)
                if nxt is None:
                    break
            else:
                nxt = options[0]
            local.add((cur, nxt))
            inc = (nxt[0] - cur[0], nxt[1] - cur[1])
            if nxt == start:
                closed = True
                break
            loop.append(nxt)
            cur = nxt
        # Only a closed cycle may consume its edges; a dead-end walk rolls
        # back so later starts can still trace the real boundary.
        if closed and len(loop) >= 4:
            used |= local
            loops.append(loop)
    return loops


def _signed_area(points):
    a = 0.0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return a / 2.0


def _rdp(points, eps):
    if len(points) < 3:
        return points
    (x1, y1), (x2, y2) = points[0], points[-1]
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy)
    best, idx = -1.0, -1
    for i in range(1, len(points) - 1):
        px, py = points[i]
        if L == 0:
            d = math.hypot(px - x1, py - y1)
        else:
            d = abs(dy * (px - x1) - dx * (py - y1)) / L
        if d > best:
            best, idx = d, i
    if best > eps:
        left = _rdp(points[:idx + 1], eps)
        right = _rdp(points[idx:], eps)
        return left[:-1] + right
    return [points[0], points[-1]]


def _simplify(points, res):
    """Closed-loop simplification of a lattice corner chain.

    Split the loop at its two farthest-apart vertices, simplify each half
    with Douglas-Peucker and stitch the halves back together. The first
    stage drops exact lattice-collinear midpoints so DP works on corners.
    """
    n = len(points)
    if n < 4:
        return points
    pts = np.asarray(points, dtype=float)
    # farthest pair by squared distance (grid-sized inputs, n^2 is fine)
    d2 = ((pts[:, None, :] - pts[None, :, :]) ** 2).sum(axis=2)
    i0, i1 = np.unravel_index(int(d2.argmax()), d2.shape)
    if i1 < i0:
        i0, i1 = i1, i0

    def collinear_prune(chain):
        kept = [chain[0]]
        for i in range(1, len(chain) - 1):
            (px, py) = chain[i - 1]
            (cx, cy) = chain[i]
            (nx, ny) = chain[i + 1]
            if (cx - px) * (ny - cy) != (cy - py) * (nx - cx):
                kept.append(chain[i])
        kept.append(chain[-1])
        return kept

    def rotate(chain, k):
        return chain[k:] + chain[:k]

    chain = rotate(points, i0)
    half1 = collinear_prune(chain[: i1 - i0 + 1])
    half2 = collinear_prune(chain[i1 - i0:] + [chain[0]])
    eps = res * RDP_EPS_CELLS
    out = _rdp(half1, eps)[:-1] + _rdp(half2, eps)[:-1]
    if len(out) < 3:
        return points
    return out


def _orthogonalize(points, segments, res, cos_t, sin_t):
    """Move polygon edges onto nearby wall lines and re-intersect corners.

    An edge whose rotated-frame direction is near-axis and lies within
    SNAP_MAX_M of an extracted wall line of the matching axis is replaced
    by that line; corners become intersections of adjacent lines. The
    area must stay within sanity bounds or the original polygon returns.
    """
    n = len(points)
    if n < 3 or not segments:
        return points
    to_r = lambda x, y: (x * cos_t + y * sin_t, -x * sin_t + y * cos_t)
    lines = [{(seg[1] + 0.5) * res for seg in segments if seg[0] == 0},
             {(seg[1] + 0.5) * res for seg in segments if seg[0] == 1}]
    if not any(lines[0] or lines[1] for _ in (0,)):
        if not lines[0] and not lines[1]:
            return points
    # Edge representation in world coords: (point, unit direction).
    edge_lines = []
    for i in range(n):
        p = points[i]
        q = points[(i + 1) % n]
        pr = to_r(*p)
        qr = to_r(*q)
        dxr, dyr = qr[0] - pr[0], qr[1] - pr[1]
        chosen = None
        if abs(dxr) <= res * 3 and lines[0]:  # near-vertical in rotated frame
            best = min(lines[0], key=lambda l: abs(l - pr[0]))
            if abs(best - pr[0]) <= SNAP_MAX_M and abs(best - qr[0]) <= SNAP_MAX_M:
                chosen = (0, best)
        elif abs(dyr) <= res * 3 and lines[1]:  # near-horizontal
            best = min(lines[1], key=lambda l: abs(l - pr[1]))
            if abs(best - pr[1]) <= SNAP_MAX_M and abs(best - qr[1]) <= SNAP_MAX_M:
                chosen = (1, best)
        if chosen is None:
            # original line through the two vertices
            d = (q[0] - p[0], q[1] - p[1])
            L = math.hypot(*d) or 1.0
            edge_lines.append((p, (d[0] / L, d[1] / L)))
        else:
            axis, const = chosen
            cr = (const, pr[1]) if axis == 0 else (pr[0], const)
            cp = (cr[0] * cos_t - cr[1] * sin_t, cr[0] * sin_t + cr[1] * cos_t)
            dr = (0.0, 1.0) if axis == 0 else (1.0, 0.0)
            cd = (dr[0] * cos_t - dr[1] * sin_t, dr[0] * sin_t + dr[1] * cos_t)
            edge_lines.append((cp, cd))
    # Intersect consecutive edge lines to form the new vertices.
    out = []
    for i in range(n):
        (p1, d1) = edge_lines[i]
        (p2, d2) = edge_lines[(i + 1) % n]
        det = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(det) < 1e-9:
            out.append(points[(i + 1) % n])
            continue
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / det
        out.append((p1[0] + d1[0] * t, p1[1] + d1[1] * t))
    old_a = _signed_area(points)
    new_a = _signed_area(out)
    if not all(math.isfinite(x) and math.isfinite(y) for x, y in out):
        return points
    if old_a > 0 and not (0.7 * old_a <= new_a <= 1.4 * old_a):
        return points
    return out


def _cell_at(grid, x, y, x_min, y_min, res):
    H, W = grid.shape
    ix = int(math.floor((x - x_min) / res))
    iy = int(math.floor((y - y_min) / res))
    if 0 <= ix < W and 0 <= iy < H:
        return int(grid[iy, ix])
    return -1


def build_floorplan(grid, occ_th=OCC_TH, free_th=FREE_TH,
                    door_min_m=DOOR_MIN_M, door_max_m=DOOR_MAX_M,
                    bar_m=BAR_M, min_room_m2=MIN_ROOM_M2, inside=None):
    """Convert an adapter grid dict into the floorplan structure."""
    del inside  # free space comes from the grid itself; kept for API clarity
    x_min, y_min, res, cells = parse_grid(grid)
    occ = cells >= occ_th
    free = (cells >= 0) & (cells <= free_th)
    theta = _dominant_angle(occ)
    cos_t, sin_t = math.cos(math.radians(theta)), math.sin(math.radians(theta))

    # Noise removal for structure analysis (kept for the report).
    labels_occ, n_occ = ndimage.label(occ, structure=np.ones((3, 3)))
    sizes = ndimage.sum_labels(np.ones_like(cells, dtype=float), labels_occ,
                               index=np.arange(1, n_occ + 1)) if n_occ else []
    occ_major = occ.copy()
    noise_cells = 0
    for i, s in enumerate(sizes, start=1):
        if s < NOISE_COMPONENT:
            occ_major[labels_occ == i] = False
            noise_cells += int(s)

    segments, cos_t, sin_t, bx, by = _axis_walls(
        occ, occ_major, res, x_min, y_min, theta)
    walls = _segments_world(segments, cos_t, sin_t, res)

    explained, dist = _explained_mask(occ_major, segments, bx, by, res)
    ys, xs = np.nonzero(occ_major)
    unexplained = occ_major & ~explained
    labels_un, n_un = ndimage.label(unexplained, structure=np.ones((3, 3)))
    obstacles = []
    for i in range(1, n_un + 1):
        comp = labels_un == i
        if comp.sum() < OBSTACLE_MIN_CELLS:
            continue
        loops = _trace_loops(comp)
        loops = [l for l in loops if _signed_area(l) > 0]
        if not loops:
            continue
        poly = _simplify(max(loops, key=_signed_area), res)
        obstacles.append({'polygon': [
            (x_min + px * res, y_min + py * res) for px, py in poly]})

    # Rooms: bar a ring along walls and unknown frontier, label the cores,
    # then regrow into the barred ring up to GROW_MAX_M.
    free_rooms = free | (occ & ~occ_major)
    dist_cells = ndimage.distance_transform_edt(free_rooms)
    bar = (dist_cells * res) < bar_m
    core = free_rooms & ~bar
    labels_room, n_rooms = ndimage.label(core)
    min_cells = min_room_m2 / (res * res)
    room_ids = [i for i in range(1, n_rooms + 1)
                if (labels_room == i).sum() >= min_cells]
    # Regrow each core into the bar by nearest-core labelling (Euclidean),
    # capped at GROW_MAX_M; iterative dilation would leave chamfered corners.
    grown = np.where(np.isin(labels_room, room_ids), labels_room, 0)
    grow_cells = int(round(GROW_MAX_M / res))
    dist_g, (iy, ix) = ndimage.distance_transform_edt(
        grown == 0, return_indices=True)
    near = (dist_g <= grow_cells) & free_rooms & (grown == 0) & (labels_room[iy, ix] != 0)
    grown[near] = labels_room[iy[near], ix[near]]
    rooms = []
    for rid in room_ids:
        mask = grown == rid
        loops = [l for l in _trace_loops(mask) if _signed_area(l) > 0]
        if not loops:
            continue
        loop = max(loops, key=_signed_area)
        poly = [(x_min + px * res, y_min + py * res) for px, py in loop]
        poly = _simplify(poly, res)
        poly = _orthogonalize(poly, segments, res, cos_t, sin_t)
        if _signed_area(poly) < 0:
            poly = poly[::-1]
        # The honest room area is the simplified polygon's, not the cell
        # count (occupancy cells include the half-open wall boundary).
        area = abs(_signed_area(poly))
        edges = []
        n = len(poly)
        unobserved = 0
        for i in range(n):
            ax, ay = poly[i]
            bxx, byy = poly[(i + 1) % n]
            mx, my = (ax + bxx) / 2, (ay + byy) / 2
            ex, ey = bxx - ax, byy - ay
            L = math.hypot(ex, ey) or 1.0
            nx, ny = ey / L, -ex / L  # outward for CCW
            ox, oy = mx + nx * res * 0.75, my + ny * res * 0.75
            val = _cell_at(cells, ox, oy, x_min, y_min, res)
            observed = val != -1
            edges.append({'a': [ax, ay], 'b': [bxx, byy], 'observed': observed})
            if not observed:
                unobserved += 1
        rooms.append({'polygon': poly, 'area_m2': area, 'edges': edges,
                      'unobserved_edge_share': (unobserved / n) if n else 0.0})
    rooms.sort(key=lambda r: -r['area_m2'])

    # Doors: gaps between collinear wall runs that separate two rooms.
    doors = []
    for axis in (0, 1):
        lines = {}
        for seg in segments:
            if seg[0] == axis:
                lines.setdefault(seg[1], []).append((seg[2], seg[3]))
        for line, runs in lines.items():
            runs.sort()
            for (lo1, hi1), (lo2, hi2) in zip(runs, runs[1:]):
                gap = (lo2 - hi1 - 1) * res
                if not (door_min_m <= gap <= door_max_m):
                    continue
                mid = ((hi1 + 1) + lo2) / 2 * res
                const = (line + 0.5) * res
                if axis == 0:
                    mxr, myr = const, mid
                    nxr, nyr = 1.0, 0.0
                else:
                    mxr, myr = mid, const
                    nxr, nyr = 0.0, 1.0
                side_rooms = set()
                for sign in (-1, 1):
                    sxr = mxr + nxr * sign * res * 3
                    syr = myr + nyr * sign * res * 3
                    wxp, wyp = _rot_to_world(sxr, syr, cos_t, sin_t)
                    ix = int(math.floor((wxp - x_min) / res))
                    iy = int(math.floor((wyp - y_min) / res))
                    if 0 <= ix < cells.shape[1] and 0 <= iy < cells.shape[0]:
                        side_rooms.add(int(grown[iy, ix]))
                if len([r for r in side_rooms if r]) >= 2:
                    if axis == 0:
                        a = _rot_to_world(const, (hi1 + 1) * res, cos_t, sin_t)
                        b = _rot_to_world(const, lo2 * res, cos_t, sin_t)
                    else:
                        a = _rot_to_world((hi1 + 1) * res, const, cos_t, sin_t)
                        b = _rot_to_world(lo2 * res, const, cos_t, sin_t)
                    doors.append({'a': list(a), 'b': list(b), 'width_m': gap})

    finite = dist[np.isfinite(dist)]
    report = {
        'theta_deg': theta,
        'occupied_cells': int(occ.sum()),
        'noise_cells': noise_cells,
        'wall_cells_explained_fraction': (
            float(explained.sum() / occ_major.sum()) if occ_major.any() else None),
        'occupied_distance_to_wall_p50_m': (
            float(np.percentile(finite, 50)) if len(finite) else None),
        'occupied_distance_to_wall_p95_m': (
            float(np.percentile(finite, 95)) if len(finite) else None),
        'unexplained_occupied_cells': int(unexplained.sum()),
        'rooms_unobserved_edge_share': [r['unobserved_edge_share'] for r in rooms],
    }
    return {'walls': walls, 'rooms': rooms, 'doors': doors,
            'obstacles': obstacles, 'report': report}


def _write_private(path):
    return open(path, 'x', encoding='utf-8')


def write_outputs(fp, geojson_path, svg_path, report_path):
    """Write GeoJSON, SVG and the report; 0600, never overwrite."""
    features = []
    for w in fp['walls']:
        features.append({'type': 'Feature', 'geometry': {
            'type': 'LineString', 'coordinates': [w['a'], w['b']]},
            'properties': {'kind': 'wall'}})
    for d in fp['doors']:
        features.append({'type': 'Feature', 'geometry': {
            'type': 'LineString', 'coordinates': [d['a'], d['b']]},
            'properties': {'kind': 'door', 'width_m': d['width_m']}})
    for room in fp['rooms']:
        ring = [list(p) for p in room['polygon']]
        ring.append(ring[0])
        features.append({'type': 'Feature', 'geometry': {
            'type': 'Polygon', 'coordinates': [ring]},
            'properties': {'kind': 'room', 'area_m2': room['area_m2'],
                           'edges': room['edges']}})
    for ob in fp['obstacles']:
        ring = [list(p) for p in ob['polygon']]
        ring.append(ring[0])
        features.append({'type': 'Feature', 'geometry': {
            'type': 'Polygon', 'coordinates': [ring]},
            'properties': {'kind': 'obstacle'}})
    geojson = {'type': 'FeatureCollection', 'features': features}

    def _bounds_x(coords):
        if coords and isinstance(coords[0], (int, float)):
            return [float(coords[0])]
        out = []
        for c in coords:
            out.extend(_bounds_x(c))
        return out

    xs = []
    for f in features:
        xs.extend(_bounds_x(f['geometry']['coordinates']))
    if not xs:
        xs = [0.0]
    pad = 0.5
    x0, x1 = min(xs) - pad, max(xs) + pad
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="'
           f'{x0} {x0} {x1 - x0} {x1 - x0}">']
    for room in fp['rooms']:
        pts = ' '.join(f'{p[0]},{p[1]}' for p in room['polygon'])
        svg.append(f'<polygon points="{pts}" fill="#e8f0e8" stroke="none"/>')
    for w in fp['walls']:
        svg.append(f'<line x1="{w["a"][0]}" y1="{w["a"][1]}" '
                   f'x2="{w["b"][0]}" y2="{w["b"][1]}" '
                   f'stroke="#404040" stroke-width="0.08"/>')
    for d in fp['doors']:
        svg.append(f'<line x1="{d["a"][0]}" y1="{d["a"][1]}" '
                   f'x2="{d["b"][0]}" y2="{d["b"][1]}" '
                   f'stroke="#208020" stroke-width="0.08"/>')
    svg.append('</svg>')

    for path, text in ((geojson_path, json.dumps(geojson, ensure_ascii=False)),
                       (svg_path, '\n'.join(svg)),
                       (report_path, json.dumps(
                           fp['report'], ensure_ascii=False, indent=1))):
        with _write_private(path) as fh:
            fh.write(text + '\n')
        os.chmod(path, 0o600)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_json', type=str, nargs='?',
                        help='adapter {"op":"map"} grid JSON file')
    parser.add_argument('--geojson')
    parser.add_argument('--svg')
    parser.add_argument('--report')
    parser.add_argument('--json', action='store_true',
                        help='read the grid from stdin, write the floorplan '
                             'structure as one JSON object to stdout (no files)')
    args = parser.parse_args()
    if args.json:
        try:
            grid = json.load(sys.stdin)
            fp = build_floorplan(grid)
        except (OSError, ValueError) as exc:
            parser.exit(1, f'floorplan: {exc}\n')
        sys.stdout.write(json.dumps(fp, allow_nan=False, default=list))
        sys.stdout.write('\n')
        return
    if not (args.geojson and args.svg and args.report and args.map_json):
        parser.error('file mode needs map_json, --geojson, --svg and --report')
    try:
        with open(args.map_json, encoding='utf-8') as fh:
            grid = json.load(fh)
        fp = build_floorplan(grid)
        write_outputs(fp, args.geojson, args.svg, args.report)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'floorplan: {exc}\n')
    print(json.dumps(fp['report'], ensure_ascii=False))


if __name__ == '__main__':
    main()
