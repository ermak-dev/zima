#!/usr/bin/env python3
# FEATURE: MRPT adapter map geometry and map reply semantics.
# SPEC: walls stay straight when beams drop out; the LDS lever arm is honored
# on rotation; `map` returns occupancy (0 free .. 100 occupied, -1 unknown)
# cropped to the observed area; hits beyond the insertion limit are ignored.
# IMPLEMENTATION: synthetic rooms scanned from the true LDS position, fed
# through the real adapter binary over its line-JSON protocol.
# RESULT: IMPLEMENTED — straightness, rotation, semantics and range limit.
# STATUS: DONE
import json
import math
import random
import subprocess
import unittest
from pathlib import Path

ADAPTER = Path(__file__).resolve().parent / 'adapter'
LDS_X = -0.1  # must match player_bridge::scan::LDS_X
BEAMS = 360
UNKNOWN = -1


def box(x0, y0, x1, y1):
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


ROOM = box(-3, -3, 3, 3)


def hit(px, py, dx, dy, seg):
    x1, y1, x2, y2 = seg
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((x1 - px) * ey - (y1 - py) * ex) / den
    u = ((x1 - px) * dy - (y1 - py) * dx) / den
    return t if t > 0 and 0 <= u <= 1 else None


def scan(walls, x, y, yaw, max_range=8.0, drop=0.0, rng=None):
    """Scan from the true LDS position; bearings in base_link."""
    lx, ly = x + LDS_X * math.cos(yaw), y + LDS_X * math.sin(yaw)
    angles, ranges = [], []
    for k in range(BEAMS):
        a = -math.pi + 2 * math.pi * k / BEAMS
        if rng and rng.random() < drop:
            continue
        c, s = math.cos(yaw + a), math.sin(yaw + a)
        ts = [t for t in (hit(lx, ly, c, s, w) for w in walls) if t]
        if not ts or min(ts) > max_range:
            continue
        angles.append(round(a, 5))
        ranges.append(round(min(ts), 5))
    return angles, ranges


def wall_distance(walls, x, y):
    def seg_dist(seg):
        x1, y1, x2, y2 = seg
        ex, ey = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * ex + (y - y1) * ey) / (ex * ex + ey * ey)))
        return math.hypot(x - (x1 + t * ex), y - (y1 + t * ey))
    return min(seg_dist(w) for w in walls)


class Adapter:
    def __init__(self):
        self.p = subprocess.Popen([str(ADAPTER)], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                  text=True, bufsize=1)

    def call(self, **req):
        self.p.stdin.write(json.dumps(req) + '\n')
        self.p.stdin.flush()
        return json.loads(self.p.stdout.readline())

    def close(self):
        self.p.stdin.close()
        self.p.wait(timeout=10)

    def drive(self, walls, start, steps, max_range=8.0, drop=0.0, seed=1):
        """Feed scans along `steps` of (dx, dyaw) with exact odometry."""
        rng = random.Random(seed)
        x, y, yaw = start
        for dx, dyaw in [(0.0, 0.0)] + steps:
            x, y, yaw = x + dx * math.cos(yaw), y + dx * math.sin(yaw), yaw + dyaw
            angles, ranges = scan(walls, x, y, yaw, max_range, drop, rng)
            reply = self.call(op='update', scan={'angles': angles, 'ranges': ranges},
                              range_max=max_range, sensor_x=LDS_X,
                              odom={'dx': dx, 'dy': 0.0, 'dyaw': dyaw})
            assert reply['ok'], reply
        return x, y, yaw

    def cells(self, start):
        """Map cells as (x, y, occupancy) in the room frame."""
        g = self.call(op='map')['grid']
        values = [int(v) for v in g['cells'].split(',')]
        sx, sy, syaw = start
        c, s = math.cos(syaw), math.sin(syaw)
        out = []
        for j in range(g['h']):
            for i in range(g['w']):
                mx = g['x_min'] + (i + 0.5) * g['res']
                my = g['y_min'] + (j + 0.5) * g['res']
                out.append((sx + c * mx - s * my, sy + s * mx + c * my, values[j * g['w'] + i]))
        return g, out


@unittest.skipUnless(ADAPTER.is_file(), 'adapter binary not built')
class AdapterMapTests(unittest.TestCase):
    def setUp(self):
        self.a = Adapter()
        self.addCleanup(self.a.close)
        self.assertTrue(self.a.call(op='start_mapping')['ok'])

    def walls_near_truth(self, walls, start, tolerance):
        _, cells = self.a.cells(start)
        occupied = [(x, y) for x, y, v in cells if v >= 65]
        self.assertGreater(len(occupied), 150, 'walls must be mapped at all')
        near = sum(1 for x, y in occupied if wall_distance(walls, x, y) <= tolerance)
        return near / len(occupied)

    def test_walls_stay_straight_when_beams_drop_out(self):
        start = (-1.0, 0.5, 0.0)
        self.a.drive(ROOM, start, [(0.1, 0.0)] * 12, drop=0.3)
        self.assertGreaterEqual(self.walls_near_truth(ROOM, start, 0.1), 0.9)

    def test_rotation_in_place_keeps_walls_sharp(self):
        start = (0.5, -0.5, 0.0)
        self.a.drive(ROOM, start, [(0.0, math.radians(15))] * 24)
        self.assertGreaterEqual(self.walls_near_truth(ROOM, start, 0.08), 0.9)

    def test_map_reply_is_cropped_occupancy_with_unknown(self):
        start = (-1.0, 0.5, 0.0)
        self.a.drive(ROOM, start, [(0.1, 0.0)] * 6)
        grid, cells = self.a.cells(start)
        self.assertLess(grid['w'] * grid['h'], 400 * 400 // 4, 'grid must be cropped')
        values = {v for _, _, v in cells}
        self.assertIn(UNKNOWN, values)
        center = [v for x, y, v in cells if abs(x) < 0.3 and abs(y) < 0.3]
        self.assertTrue(center and max(center) <= 35, 'room interior must read as free')
        self.assertGreaterEqual(max(values), 65, 'walls must read as occupied')

    def test_far_hits_are_not_inserted(self):
        corridor = box(-7, -1, 7, 1)
        start = (-6.0, 0.0, 0.0)
        self.a.drive(corridor, start, [(0.1, 0.0)] * 4, max_range=16.0)
        _, cells = self.a.cells(start)
        far = [(x, y, v) for x, y, v in cells if v != UNKNOWN and x > -0.5]
        self.assertEqual(far, [], 'nothing beyond the insertion limit may be mapped')


if __name__ == '__main__':
    unittest.main()
