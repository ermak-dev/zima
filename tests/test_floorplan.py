#!/usr/bin/env python3
# FEATURE (#70): offline occupancy-grid to polygonal floorplan conversion.
# SPEC: axis walls from projections, rooms via door cuts, obstacles as polygons,
#       unknown gaps never become walls or floor, bounded validation, 0600 outputs.
# IMPLEMENTATION: tools/floorplan/floorplan.py with numpy/scipy only.
# RESULT: PENDING
# STATUS: IN_PROGRESS
import importlib.util
import json
import math
from pathlib import Path
import stat
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools/floorplan/floorplan.py'


def load_module():
    # Loaded once at test-module import time: some audio tests purge
    # sys.modules['numpy'] while running, and re-importing numpy during
    # the run phase leaves its C core inconsistent.
    assert TOOL.is_file(), 'floorplan tool not implemented'
    spec = importlib.util.spec_from_file_location('floorplan', TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


FP = load_module()


def rasterize(walls, x0, y0, x1, y1, res=0.05, thickness=1):
    """Rasterize wall segments onto a grid; returns (grid_dict, W, H).

    A cell is occupied when the segment passes within half a cell diagonal of
    the cell centre, so walls stay exactly one cell thick on axis and diagonal
    walls cover their true path.
    """
    import numpy as np
    W = int(round((x1 - x0) / res))
    H = int(round((y1 - y0) / res))
    grid = np.full((H, W), -1, dtype=np.int64)
    for (ax, ay, bx, by) in walls:
        steps = max(2, int(math.hypot(bx - ax, by - ay) / res * 2) + 1)
        for s in range(steps + 1):
            t = s / steps
            px, py = ax + (bx - ax) * t, ay + (by - ay) * t
            cx = int(math.floor((px - x0) / res))
            cy = int(math.floor((py - y0) / res))
            for ox in (-1, 0, 1):
                for oy in (-1, 0, 1):
                    ix, iy = cx + ox, cy + oy
                    if 0 <= ix < W and 0 <= iy < H:
                        ccx, ccy = x0 + (ix + 0.5) * res, y0 + (iy + 0.5) * res
                        # distance from cell centre to the segment
                        dx, dy = bx - ax, by - ay
                        L2 = dx * dx + dy * dy
                        tt = 0.0 if L2 == 0 else ((ccx - ax) * dx + (ccy - ay) * dy) / L2
                        tt = max(0.0, min(1.0, tt))
                        ex = ccx - (ax + tt * dx)
                        ey = ccy - (ay + tt * dy)
                        if math.hypot(ex, ey) <= res * 0.55 + 1e-9:
                            grid[iy, ix] = 100
    return grid, W, H


def grid_value(walls, bounds, res=0.05, inside=None):
    grid, W, H = rasterize(walls, *bounds, res=res)
    # Observed free space: flood the component containing the inside point.
    from scipy import ndimage
    x0, y0, x1, y1 = bounds
    if inside is None:
        inside = ((x0 + x1) / 2, (y0 + y1) / 2)
    lab, n = ndimage.label(grid != 100)
    iy = int((inside[1] - y0) / res)
    ix = int((inside[0] - x0) / res)
    grid[lab == lab[iy, ix]] = 0
    return {
        'x_min': bounds[0], 'y_min': bounds[1], 'res': float(res),
        'w': W, 'h': H, 'cells': ','.join(str(int(v)) for v in grid.flatten()),
    }


def rect_room(x0, y0, x1, y1, door=None, res=0.05):
    """Walls of a rectangle; door=(cx,cy,horizontal?,width) removes a gap."""
    walls = [
        (x0, y0, x1, y0), (x0, y1, x1, y1),
        (x0, y0, x0, y1), (x1, y0, x1, y1),
    ]
    if door is not None:
        cx, cy, horiz, width = door
        if horiz:
            walls = [w for w in walls if not (
                abs(w[1] - w[3]) < 1e-9 and abs(w[1] - cy) < 1e-9)]
            hw = width / 2
            walls += [(w[0], cy, cx - hw, cy) for w in [(x0, 0, 0, 0)]] if False else []
            walls += [(x0, cy, cx - hw, cy), (cx + hw, cy, x1, cy)]
        else:
            walls = [w for w in walls if not (
                abs(w[0] - w[2]) < 1e-9 and abs(w[0] - cx) < 1e-9)]
            hw = width / 2
            walls += [(cx, y0, cx, cy - hw), (cx, cy + hw, cx, y1)]
    return walls


class FloorplanBasics(unittest.TestCase):
    fp = FP

    def build(self, walls, bounds, inside=None, **kw):
        gv = grid_value(walls, bounds, inside=inside)
        return self.fp.build_floorplan(gv, **kw)

    # SPEC: a plain rectangular room yields four walls and one room polygon.
    def test_rectangle_room(self):
        fp = self.build(rect_room(0, 0, 4, 3), (-0.2, -0.2, 4.2, 3.2))
        self.assertEqual(len([w for w in fp['walls'] if w['kind'] == 'wall']), 4)
        rooms = fp['rooms']
        self.assertEqual(len(rooms), 1)
        area = rooms[0]['area_m2']
        self.assertLess(abs(area - 12.0) / 12.0, 0.03, f'area {area}')
        # each wall must run close to its true line
        for wy in (0.0, 3.0):
            hs = [w for w in fp['walls'] if w['kind'] == 'wall'
                  and abs(w['a'][1] - wy) < 0.2 and abs(w['b'][1] - wy) < 0.2]
            self.assertTrue(hs, f'no wall near y={wy}')
        for wx in (0.0, 4.0):
            vs = [w for w in fp['walls'] if w['kind'] == 'wall'
                  and abs(w['a'][0] - wx) < 0.2 and abs(w['b'][0] - wx) < 0.2]
            self.assertTrue(vs, f'no wall near x={wx}')

    # SPEC: unknown gaps in a wall stay unknown: no wall segment there and
    # the adjacent room edge is marked unobserved.
    def test_unknown_gap_is_not_a_wall(self):
        # A wall gap with unobserved territory behind: gap cells are unknown
        # (as in a real SLAM grid), not free.
        import numpy as np
        walls = rect_room(0, 0, 4, 3)
        gv = grid_value(walls, (-0.2, -0.2, 4.2, 3.2))
        cells = np.array([int(c) for c in gv['cells'].split(',')]).reshape(gv['h'], gv['w'])
        res = gv['res']
        for iy in range(gv['h']):
            for ix in range(gv['w']):
                x = -0.2 + (ix + 0.5) * res
                y = -0.2 + (iy + 0.5) * res
                if 1.65 < x < 2.35 and y < 0.1:
                    cells[iy, ix] = -1
        gv['cells'] = ','.join(str(int(v)) for v in cells.flatten())
        fp = self.fp.build_floorplan(gv)
        segs = [w for w in fp['walls'] if w['kind'] == 'wall'
                and abs(w['a'][1]) < 0.15 and abs(w['b'][1]) < 0.15]
        for w in segs:
            mid = ((w['a'][0] + w['b'][0]) / 2, 0.0)
            self.assertFalse(1.7 < mid[0] < 2.3, f'wall invented in gap: {w}')
        room = fp['rooms'][0]
        edges = room['edges']
        gap_edges = [e for e in edges
                     if abs(e['a'][1]) < 0.2 and abs(e['b'][1]) < 0.2
                     and max(e['a'][0], e['b'][0]) > 1.65
                     and min(e['a'][0], e['b'][0]) < 2.35]
        self.assertTrue(gap_edges, 'room must have an edge bordering the gap')
        self.assertTrue(any(not e['observed'] for e in gap_edges),
                        'gap-adjacent edge must be observed:false')

    # SPEC: two rooms separated by a wall with a 0.8 m door.
    def test_two_rooms_with_door(self):
        walls = rect_room(0, 0, 8, 3)
        walls += [(4, 0, 4, 1.1), (4, 1.9, 4, 3)]  # divider with 0.8 m gap
        fp = self.build(walls, (-0.2, -0.2, 8.2, 3.2), inside=(2.0, 1.5))
        self.assertEqual(len(fp['rooms']), 2, 'two rooms expected')
        doors = fp['doors']
        self.assertTrue(doors, 'a door must be detected')
        d = doors[0]
        self.assertLess(abs(d['width_m'] - 0.8), 0.10, f"width {d['width_m']}")
        self.assertLess(abs((d['a'][0] + d['b'][0]) / 2 - 4.0), 0.10, 'door x')
        self.assertLess(abs((d['a'][1] + d['b'][1]) / 2 - 1.5), 0.10, 'door y')

    # SPEC: an L-shaped room keeps six corners all at right angles.
    def test_l_shaped_room(self):
        walls = [
            (0, 0, 4, 0), (4, 0, 4, 1), (4, 1, 2, 1), (2, 1, 2, 3),
            (2, 3, 0, 3), (0, 3, 0, 0),
        ]
        fp = self.build(walls, (-0.2, -0.2, 4.2, 3.2), inside=(1.0, 2.0))
        self.assertEqual(len(fp['rooms']), 1)
        room = fp['rooms'][0]
        self.assertEqual(len(room['polygon']), 6, f"L room corners: {room['polygon']}")
        for i in range(6):
            ax, ay = room['polygon'][i]
            bx, by = room['polygon'][(i - 1) % 6]
            cx, cy = room['polygon'][(i + 1) % 6]
            v1 = (ax - bx, ay - by)
            v2 = (cx - ax, cy - ay)
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            dot = v1[0] * v2[0] + v1[1] * v2[1]
            ang = math.degrees(math.atan2(abs(cross), dot))
            self.assertLess(abs(ang - 90), 5, f'corner {i} angle {ang}')

    # SPEC: salt-and-pepper noise keeps the clean-scene topology.
    def test_noise_keeps_topology(self):
        import numpy as np
        gv = grid_value(rect_room(0, 0, 4, 3), (-0.2, -0.2, 4.2, 3.2))
        cells = np.array([int(c) for c in gv['cells'].split(',')]).reshape(gv['h'], gv['w'])
        rng = np.random.default_rng(7)
        free = cells == 0
        occ = cells == 100
        # flip 2% of free cells to occupied and jitter occupied ones
        flip = rng.random(cells.shape) < 0.02
        cells[free & flip] = 100
        gv['cells'] = ','.join(str(int(v)) for v in cells.flatten())
        fp = self.fp.build_floorplan(gv)
        self.assertEqual(len(fp['rooms']), 1)
        area = fp['rooms'][0]['area_m2']
        self.assertLess(abs(area - 12.0) / 12.0, 0.05, f'noisy area {area}')

    # SPEC: a pillar inside a room is an obstacle, not a wall or boundary.
    def test_pillar_is_obstacle(self):
        walls = rect_room(0, 0, 4, 3)
        walls += [(1.9, 1.4, 2.1, 1.4), (1.9, 1.6, 2.1, 1.6),
                  (1.9, 1.4, 1.9, 1.6), (2.1, 1.4, 2.1, 1.6)]
        fp = self.build(walls, (-0.2, -0.2, 4.2, 3.2), inside=(1.0, 1.5))
        self.assertEqual(len(fp['rooms']), 1)
        obsts = fp['obstacles']
        self.assertTrue(obsts, 'pillar must become an obstacle polygon')
        p = obsts[0]['polygon']
        cx = sum(x for x, _ in p) / len(p)
        cy = sum(y for _, y in p) / len(p)
        self.assertLess(abs(cx - 2.0), 0.1)
        self.assertLess(abs(cy - 1.5), 0.1)
        for w in fp['walls']:
            if w['kind'] == 'wall':
                midx = (w['a'][0] + w['b'][0]) / 2
                midy = (w['a'][1] + w['b'][1]) / 2
                self.assertFalse(abs(midx - 2.0) < 0.12 and abs(midy - 1.5) < 0.12,
                                 'pillar leaked into walls')

    # SPEC: a rotated scene still resolves rooms after dominant-direction search.
    def test_rotated_scene(self):
        ang = math.radians(17)
        def rot(walls):
            out = []
            for (ax, ay, bx, by) in walls:
                rax = ax * math.cos(ang) - ay * math.sin(ang)
                ray = ax * math.sin(ang) + ay * math.cos(ang)
                rbx = bx * math.cos(ang) - by * math.sin(ang)
                rby = bx * math.sin(ang) + by * math.cos(ang)
                out.append((rax, ray, rbx, rby))
            return out
        walls = rot(rect_room(0, 0, 4, 3))
        # rotate around room centre to keep positive coords manageable
        cx, cy = 2.0, 1.5
        walls = [tuple(round(v, 6) for v in
                       (a[0] - (cx - cx * math.cos(ang) + cy * math.sin(ang)),
                        a[1] - (cy - cx * math.sin(ang) - cy * math.cos(ang)),
                        b[0] - (cx - cx * math.cos(ang) + cy * math.sin(ang)),
                        b[1] - (cy - cx * math.sin(ang) - cy * math.cos(ang))))
                 for a, b in [(w[:2], w[2:]) for w in walls]]
        x0 = min(min(w[0], w[2]) for w in walls) - 0.2
        y0 = min(min(w[1], w[3]) for w in walls) - 0.2
        x1 = max(max(w[0], w[2]) for w in walls) + 0.2
        y1 = max(max(w[1], w[3]) for w in walls) + 0.2
        inside_pt = (cx + (2.0 - cx) * math.cos(ang) - (1.5 - cy) * math.sin(ang),
                     cy + (2.0 - cx) * math.sin(ang) + (1.5 - cy) * math.cos(ang))
        fp = self.build(walls, (x0, y0, x1, y1), inside=inside_pt)
        self.assertEqual(len(fp['rooms']), 1)
        area = fp['rooms'][0]['area_m2']
        self.assertLess(abs(area - 12.0) / 12.0, 0.05, f'rotated area {area}')

    # SPEC: hostile grids are rejected before any output.
    def test_hostile_grid_rejected(self):
        bad = [
            {'x_min': 0, 'y_min': 0, 'res': 0.05, 'w': 0, 'h': 0, 'cells': ''},
            {'x_min': 0, 'y_min': 0, 'res': 0.05, 'w': 10, 'h': 10, 'cells': '1,2'},
            {'x_min': 0, 'y_min': 0, 'res': -0.05, 'w': 10, 'h': 10,
             'cells': ','.join(['0'] * 100)},
            {'x_min': 0, 'y_min': 0, 'res': 0.05, 'w': 2000, 'h': 600,
             'cells': ','.join(['0'] * (2000 * 600))},
        ]
        for g in bad:
            with self.assertRaises(ValueError, msg=f'accepted {g.get("w")}x{g.get("h")}'):
                self.fp.build_floorplan(g)
        empty = {'x_min': 0, 'y_min': 0, 'res': 0.05, 'w': 10, 'h': 10,
                 'cells': ','.join(['-1'] * 100)}
        fp = self.fp.build_floorplan(empty)
        self.assertEqual(fp['rooms'], [])
        self.assertEqual(fp['walls'], [])


class FloorplanOutput(unittest.TestCase):
    fp = FP

    @classmethod
    def setUpClass(cls):
        if not hasattr(cls.fp, 'write_outputs'):
            raise unittest.SkipTest('write_outputs not implemented')

    def test_outputs_written_0600_no_clobber(self):
        import tempfile
        gv = grid_value(rect_room(0, 0, 4, 3), (-0.2, -0.2, 4.2, 3.2))
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            geo = td / 'out.geojson'
            svg = td / 'out.svg'
            rep = td / 'report.json'
            self.fp.write_outputs(self.fp.build_floorplan(gv), geo, svg, rep)
            for p in (geo, svg, rep):
                self.assertTrue(p.is_file(), f'{p} missing')
                self.assertEqual(stat.S_IMODE(p.stat().st_mode), 0o600)
            data = json.loads(geo.read_text())
            self.assertEqual(data.get('type'), 'FeatureCollection')
            existing = (td / 'again.geojson')
            existing.write_text('keep')
            with self.assertRaises(OSError):
                self.fp.write_outputs(self.fp.build_floorplan(gv), existing, svg, rep)
            self.assertEqual(existing.read_text(), 'keep')


class FloorplanJsonMode(unittest.TestCase):
    """--json (#83): grid on stdin, one floorplan JSON on stdout; hostile
    input exits non-zero with no stdout payload."""

    def run_json(self, payload, timeout=30):
        import subprocess, sys
        return subprocess.run(
            [sys.executable, '-I', str(TOOL), '--json'],
            input=json.dumps(payload).encode(),
            capture_output=True, timeout=timeout)

    def test_json_mode_roundtrip(self):
        gv = grid_value(rect_room(0, 0, 4, 3), (-0.2, -0.2, 4.2, 3.2))
        proc = self.run_json(gv)
        self.assertEqual(proc.returncode, 0, proc.stderr.decode()[:400])
        out = json.loads(proc.stdout.decode())
        for key in ('walls', 'rooms', 'doors', 'obstacles', 'report'):
            self.assertIn(key, out)
        self.assertEqual(len(out['rooms']), 1)
        self.assertLess(abs(out['rooms'][0]['area_m2'] - 12.0) / 12.0, 0.05)

    def test_json_mode_hostile_input(self):
        for bad in ({}, {'cells': 'x'}, {'w': 1e9, 'h': 1, 'cells': '0'}):
            proc = self.run_json(bad)
            self.assertNotEqual(proc.returncode, 0, f'accepted {bad}')
            self.assertEqual(proc.stdout, b'', 'no partial stdout on error')


if __name__ == '__main__':
    unittest.main()
