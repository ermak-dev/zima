#!/usr/bin/env python3
# FEATURE: MRPT adapter map geometry and map reply semantics.
# SPEC: walls stay straight when beams drop out; the LDS lever arm is honored
# on rotation; `map` returns occupancy (0 free .. 100 occupied, -1 unknown)
# cropped to the observed area; hits beyond the insertion limit are ignored.
# IMPLEMENTATION: synthetic rooms scanned from the true LDS position, fed
# through the real adapter binary over its line-JSON protocol.
# RESULT: IMPLEMENTED — straightness, rotation, semantics and range limit.
# STATUS: DONE
import os
import tempfile
import json
import math
import random
import subprocess
import unittest
from pathlib import Path

ADAPTER = Path(os.environ.get('MRPT_ADAPTER_BIN', Path(__file__).resolve().parent / 'adapter'))
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
    def __init__(self, env=None):
        child_env = {k: v for k, v in os.environ.items() if not k.startswith('ROBOT_SLAM_')}
        child_env.update(env or {})
        self.p = subprocess.Popen([str(ADAPTER)], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                  text=True, bufsize=1, env=child_env)

    def call(self, **req):
        self.p.stdin.write(json.dumps(req) + '\n')
        self.p.stdin.flush()
        return json.loads(self.p.stdout.readline())

    def close(self):
        self.p.stdin.close()
        self.p.wait(timeout=10)
        self.p.stdout.close()

    def drive(self, walls, start, steps, max_range=8.0, drop=0.0, seed=1):
        """Feed scans along `steps` of (dx, dyaw) with exact odometry."""
        rng = random.Random(seed)
        x, y, yaw = start
        for dx, dyaw in [(0.0, 0.0)] + steps:
            x, y, yaw = x + dx * math.cos(yaw), y + dx * math.sin(yaw), yaw + dyaw
            angles, ranges = scan(walls, x, y, yaw, max_range, drop, rng)
            reply = self.call(op='update', scan={'angles': angles, 'ranges': ranges},
                              range_max=max_range, sensor_x=LDS_X,
                              odom={'dx': dx, 'dy': 0.0, 'dyaw': dyaw}, motion_compensated=True)
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


# FEATURE: Controlled XY ICP, honest diagnostics and incremental trusted occupancy.
# SPEC: Preserve isolated/nonuniform rays, correct wrong odometry, reject bad
# geometry without inserting it; bootstrap quality is null. Invalid requests do
# not mutate state; map reads do not replay observations. Build the selected source.
# IMPLEMENTATION: Explicit XY CICP + persistent trusted grid, strict JSON/config
# and source-time handling; MCL spread is separate from ICP quality.
# RESULT: IMPLEMENTED — real odometry correction and rejection/recovery validated.
# STATUS: DONE
class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.a = Adapter()
        self.addCleanup(self.a.close)
        self.assertTrue(self.a.call(op='start_mapping')['ok'])

    def update(self, x=0, y=0, yaw=0, dx=0, dy=0, dyaw=0, indices=None):
        angles, ranges = scan(ROOM, x, y, yaw)
        if indices is not None:
            angles, ranges = ([angles[i] for i in indices], [ranges[i] for i in indices])
        return self.a.call(op='update', scan={'angles': angles, 'ranges': ranges},
                           sensor_x=LDS_X, range_max=8,
                           odom={'dx': dx, 'dy': dy, 'dyaw': dyaw}, motion_compensated=True)

    def test_bootstrap_is_not_a_perfect_icp_match(self):
        r = self.update()
        self.assertIsNone(r['quality'])
        self.assertEqual(r['tracking'], 'initializing')
        d = r['diagnostics']
        self.assertFalse(d['accepted'])
        self.assertEqual(d['input_points'], 360)
        self.assertEqual(d['used_points'], 360)
        self.assertEqual(d['reference_points'], 0)
        for key in ('goodness', 'iterations', 'correction', 'covariance', 'residual_rms'):
            self.assertIsNone(d[key], key)
        self.assertGreaterEqual(d['duration_ms'], 0)

    def test_sparse_initial_geometry_can_finish_bootstrap(self):
        self.update(indices=[1, 120])
        r = self.update()
        self.assertEqual(r['tracking'], 'initializing')
        self.assertIsNone(r['quality'])
        self.assertEqual(r['diagnostics']['reference_points'], 2)
        self.assertTrue(self.update()['diagnostics']['accepted'])

    def test_wrong_odometry_is_actually_corrected(self):
        self.update()
        r = self.update(x=0.2, y=0.08, yaw=0.06, dx=0.38, dy=-0.06, dyaw=0.12)
        self.assertAlmostEqual(r['pose']['x'], 0.2, delta=0.035)
        self.assertAlmostEqual(r['pose']['y'], 0.08, delta=0.035)
        self.assertAlmostEqual(r['pose']['yaw'], 0.06, delta=0.015)
        d = r['diagnostics']
        self.assertTrue(d['accepted'])
        self.assertEqual(r['tracking'], 'tracking')
        self.assertEqual(r['quality'], d['goodness'])
        self.assertGreater(d['goodness'], 0.8)
        self.assertGreater(d['iterations'], 0)
        self.assertLess(d['correction']['x'], -0.1)
        self.assertEqual(len(d['covariance']), 9)
        self.assertTrue(all(math.isfinite(v) for v in d['covariance']))
        self.assertLess(d['residual_rms'], 0.08)

    def test_uncompensated_fast_motion_pauses_map_insertion_but_not_pose_tracking(self):
        self.update()
        before = self.a.call(op='map')['grid']
        angles, ranges = scan(ROOM, 0, 0, 0.36)
        request = dict(op='update', scan={'angles': angles, 'ranges': ranges},
                       sensor_x=LDS_X, range_max=8, odom={'dx': 0, 'dy': 0, 'dyaw': 0.36},
                       motion_compensated=False)
        result = self.a.call(**request)
        self.assertTrue(result['diagnostics']['accepted'])
        self.assertTrue(result['diagnostics'].get('mapping_paused'), 'large unverified motion must not paint new keyframes')
        self.assertEqual(result['tracking'], 'degraded')
        self.assertEqual(result['diagnostics']['reason'], 'uncompensated_motion')
        self.assertEqual(self.a.call(op='map')['grid'], before)
        request['odom']['dyaw'] = 0
        result = self.a.call(**request)
        self.assertFalse(result['diagnostics']['mapping_paused'])
        self.assertNotEqual(self.a.call(op='map')['grid'], before)

    def test_postfit_correspondences_are_measured_and_absent_during_bootstrap(self):
        bootstrap = self.update()['diagnostics']
        self.assertIn('correspondences', bootstrap)
        self.assertIsNone(bootstrap['correspondences'])
        matched = self.update()['diagnostics']
        self.assertEqual(matched['correspondences'], 360)
        self.assertGreater(matched['correspondence_threshold_m'], 0)
        self.assertLess(matched['residual_rms'], 0.001)

    def test_nonuniform_dropout_and_isolated_rays_are_not_filtered(self):
        indices = sorted(set(range(0, 360, 3)) | set(range(8, 45)) | {101, 220, 358})
        self.update(indices=indices)
        r = self.update(x=0.12, dx=0.25, indices=indices)
        d = r['diagnostics']
        self.assertEqual(d['used_points'], len(indices))
        self.assertEqual(d['reference_points'], len(indices))
        self.assertTrue(d['accepted'])
        self.assertAlmostEqual(r['pose']['x'], 0.12, delta=0.04)

    def test_rejected_geometry_is_not_inserted_and_tracking_recovers(self):
        self.update()
        before = self.a.call(op='map')['grid']
        r = self.a.call(op='update', scan={'angles': [0, 1, 2], 'ranges': [0.5]*3},
                        odom={'dx': 0.05, 'dy': 0, 'dyaw': 0})
        self.assertEqual(r['tracking'], 'degraded')
        self.assertFalse(r['diagnostics']['accepted'])
        self.assertEqual(self.a.call(op='map')['grid'], before)
        self.assertAlmostEqual(r['pose']['x'], 0.05, delta=0.001)
        self.assertTrue(self.update(x=0.05)['diagnostics']['accepted'])

    def test_low_goodness_rejection_does_not_pollute_map(self):
        self.update()
        before = self.a.call(op='map')['grid']
        r = self.a.call(op='update', scan={'angles': [-math.pi + i*.02 for i in range(300)],
                                         'ranges': [0.6]*300})
        self.assertFalse(r['diagnostics']['accepted'])
        self.assertEqual(r['tracking'], 'degraded')
        self.assertLess(r['quality'], 0.3)
        self.assertEqual(self.a.call(op='map')['grid'], before)

    def test_invalid_or_empty_scan_never_advances_pose(self):
        self.update()
        before = self.a.call(op='map')['grid']
        for scan_data in ({'angles': [], 'ranges': []},
                          {'angles': [0, 1], 'ranges': [2]},
                          {'angles': ['oops'], 'ranges': [2]},
                          {'angles': [None], 'ranges': [2]}):
            r = self.a.call(op='update', scan=scan_data, odom={'dx': 9})
            self.assertFalse(r['ok'], r)
        r = self.update()
        self.assertAlmostEqual(r['pose']['x'], 0, delta=0.001)
        self.assertEqual(self.a.call(op='map')['grid'], before)

    def test_unknown_angular_gap_stays_unknown(self):
        r = self.a.call(op='update', scan={'angles': [-1, 1], 'ranges': [2, 2]})
        self.assertTrue(r['ok'])
        g, cells = self.a.cells((0, 0, 0))
        middle = [v for x, y, v in cells if 0.5 < x < 1 and abs(y) < 0.1]
        self.assertTrue(middle)
        self.assertEqual(set(middle), {-1})
        self.assertEqual(self.a.call(op='map')['grid'], g)

    def test_save_load_and_reset_clears_mapping_state(self):
        self.update()
        expected = self.a.call(op='map')['grid']
        with tempfile.TemporaryDirectory(prefix='mrpt-map-') as d:
            path = str(Path(d) / 'quoted " map.simplemap')
            self.assertTrue(self.a.call(op='save_map', path=path)['ok'])
            self.assertTrue(Path(path).exists())
            self.assertTrue(self.a.call(op='load_map', path=path)['ok'])
            self.assertEqual(self.a.call(op='map')['grid'], expected)
            localized = self.update()
            self.assertTrue(localized['ok'])
            # The shared contract reserves quality for ICP goodness. Keep
            # the legacy MCL proxy explicitly separate, not as fake ICP quality.
            self.assertIsNone(localized['quality'])
            self.assertIsInstance(localized['localization_quality'], (int, float))
            self.assertGreaterEqual(localized['localization_quality'], 0)
            self.assertLessEqual(localized['localization_quality'], 1)
            self.assertIsNone(localized['diagnostics']['goodness'])
            self.assertIsNone(localized['diagnostics']['iterations'])
        self.a.call(op='start_mapping')
        self.assertEqual(self.a.call(op='map')['grid']['w'], 0)
        self.assertIsNone(self.update()['quality'])


class ConfigTests(unittest.TestCase):
    def test_invalid_configuration_is_rejected(self):
        bad = {
            'ICP_MIN_GOODNESS': '1.01', 'ICP_MAX_ITERATIONS': '2.5',
            'ICP_THRESHOLD_DIST': 'nan', 'ICP_DECIMATION': '0',
            'INSERTION_LINEAR': '-1', 'INSERTION_ANGULAR_DEG': 'inf',
            'GRID_RESOLUTION': '0', 'MAX_INSERT_RANGE': 'nope',
        }
        for key, value in bad.items():
            with self.subTest(key=key):
                a = Adapter({'ROBOT_SLAM_' + key: value})
                try:
                    self.assertFalse(a.call(op='start_mapping')['ok'])
                    self.assertEqual(a.call(op='ping')['mode'], 'idle')
                finally:
                    a.close()

    def test_grid_and_icp_configuration_are_applied(self):
        a = Adapter({'ROBOT_SLAM_GRID_RESOLUTION': '0.1',
                     'ROBOT_SLAM_ICP_MAX_ITERATIONS': '3',
                     'ROBOT_SLAM_ICP_DECIMATION': '2',
                     'ROBOT_SLAM_INSERTION_LINEAR': '10',
                     'ROBOT_SLAM_INSERTION_ANGULAR_DEG': '180'})
        self.addCleanup(a.close)
        self.assertTrue(a.call(op='start_mapping')['ok'])
        angles, ranges = scan(ROOM, 0, 0, 0)
        args = {'op': 'update', 'scan': {'angles': angles, 'ranges': ranges}, 'sensor_x': LDS_X}
        self.assertTrue(a.call(**args)['ok'])
        g = a.call(op='map')['grid']
        self.assertAlmostEqual(g['res'], 0.1)
        r = a.call(**args)
        self.assertTrue(r['diagnostics']['accepted'])
        self.assertGreater(r['diagnostics']['iterations'], 0)
        self.assertEqual(a.call(op='map')['grid'], g)


class BuildTests(unittest.TestCase):
    def test_test_build_never_overwrites_the_default_runtime_adapter(self):
        import shutil
        with tempfile.TemporaryDirectory(prefix='mrpt-build-default-') as temporary:
            root = Path(temporary)
            shutil.copy2(Path(__file__).resolve().parent / 'build.sh', root / 'build.sh')
            (root / 'third_party/install/include').mkdir(parents=True)
            (root / 'third_party/install/lib').mkdir(parents=True)
            (root / 'third_party/install/lib/libmrpt-fake.so').touch()
            runtime = root / 'adapter'
            runtime.write_text('runtime-binary')
            compiler = root / 'compiler'
            compiler.write_text('#!/usr/bin/env python3\nimport sys\nfrom pathlib import Path\nPath(sys.argv[sys.argv.index("-o")+1]).write_text("test-binary")\n')
            compiler.chmod(0o700)
            subprocess.run(['bash', str(root / 'build.sh'), '--test'],
                           env={**os.environ, 'CXX': str(compiler)}, check=True, capture_output=True)
            self.assertEqual(runtime.read_text(), 'runtime-binary')
            self.assertEqual((root / 'adapter-tests').read_text(), 'test-binary')

    def test_reproducible_build_to_selected_output(self):
        script = Path(__file__).resolve().parent / 'build.sh'
        self.assertTrue(script.is_file(), 'build.sh --output must build current source')
        with tempfile.TemporaryDirectory(prefix='mrpt-build-') as d:
            output = Path(d) / 'adapter'
            subprocess.run(['bash', str(script), '--output', str(output)], check=True,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            p = subprocess.run([str(output)], input='{"op":"ping"}\n', text=True,
                               capture_output=True, check=True)
            self.assertTrue(json.loads(p.stdout)['ok'])


if __name__ == '__main__':
    unittest.main()
