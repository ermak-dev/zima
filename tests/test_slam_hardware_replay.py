#!/usr/bin/env python3
# FEATURE: Strict, bounded offline replay of raw hardware Player recordings.
# SPEC: Preserve XDR scan order/u16/ID; reject corrupt/incomplete input; body-frame
# odometry from last usable update; explicit unverified association, no fake truth.
# IMPLEMENTATION: bounded capture/XDR parser, receive-order association, trusted stdio replay.
# RESULT: IMPLEMENTED — parser, body-frame deltas, diagnostics and transport limits verified.
# STATUS: DONE
import base64
import copy
import importlib.util
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools/slam-replay/hardware.py'
HEADER = dict(type='header', format='robot-player-raw-v1', max_file_bytes=67108864,
              max_payload_bytes=65536, queue_capacity=128, source_clock_verified=False,
              receive_clock='monotonic offset from recorder open in nanoseconds')


def laser(ranges=(1., 2., 3.), angles=(2., -1., 0.), intensities=(65535, 32768, 1), scan_id=91):
    data = struct.pack('>f', 6.)
    for values, code in ((ranges, 'f'), (angles, 'f'), (intensities, 'I')):
        data += struct.pack('>II', len(values), len(values))
        data += struct.pack('>' + code * len(values), *values)
    return data + struct.pack('>I', scan_id)


def frame(payload, interface=6, ns=100000000, stamp=50.):
    return dict(type='frame', interface=interface, index=0, msg_type=1,
                subtype=3 if interface == 6 else 1, source_stamp=stamp,
                source_seq=7, receive_offset_ns=ns,
                payload_base64=base64.b64encode(payload).decode())


def odom(x=0., y=0., yaw=0., ns=0):
    return frame(struct.pack('>6dI', x, y, yaw, 0., 0., 0., 0), 4, ns)


def rows(frames):
    return [copy.deepcopy(HEADER), *frames, dict(type='completion', complete=True,
        frames_written=len(frames), queue_dropped=0, oversized=0, size_limited=0)]


class HardwareReplayTests(unittest.TestCase):
    def module(self):
        self.assertTrue(TOOL.is_file(), 'hardware replay not implemented')
        spec = importlib.util.spec_from_file_location('hardware_replay', TOOL)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def load(self, m, data, **kwargs):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'raw.jsonl'
            path.write_text('\n'.join(json.dumps(r) for r in data) + '\n')
            return m.load(path, **kwargs)

    def test_xdr_order_unsigned_short_and_real_scan_id(self):
        m = self.module()
        scan = m.decode_laser(laser())
        self.assertEqual(scan, dict(range_max=6., ranges=[1., 2., 3.],
            angles=[2., -1., 0.], intensities=[65535, 32768, 1], scan_id=91))
        capture = self.load(m, rows([odom(), frame(laser())]))
        records = m.prepare(capture, lidar_yaw_deg=97.6)['records']
        self.assertEqual(records[0]['scan_id'], 91)
        self.assertEqual(records[0]['request']['scan']['ranges'], [1., 2., 3.])
        self.assertAlmostEqual(records[0]['request']['scan']['angles'][0], 2 + math.radians(97.6))
        self.assertEqual(records[0]['request']['sensor_x'], -.1)
        self.assertNotIn('stamp', records[0]['request'])
        self.assertNotIn('motion_compensated', records[0]['request'])
        self.assertFalse(records[0]['association']['source_time_verified'])
        self.assertEqual(records[0]['association']['method'], 'receive_order_unverified')
        self.assertEqual(records[0]['association']['age_ms'], 100.)
        self.assertEqual(records[0]['association']['scan_source_stamp'], 50.)

    # FEATURE: raw files from unified bundles share the session's monotonic origin.
    # SPEC: accept that declared origin without treating it as a verified source clock.
    def test_unified_session_clock_origin_is_accepted_without_verification(self):
        m = self.module()
        data = rows([odom(), frame(laser())])
        data[0]['receive_clock'] = 'monotonic offset from session start in nanoseconds'
        try:
            capture = self.load(m, data)
        except ValueError as error:
            self.fail(f'unified session raw capture rejected: {error}')
        record = m.prepare(capture, lidar_yaw_deg=0)['records'][0]
        self.assertEqual(record['association']['age_ms'], 100.)
        self.assertFalse(record['association']['source_time_verified'])
        data[0]['receive_clock'] = 'UTC measurement time'
        with self.assertRaises(ValueError):
            self.load(m, data)

    def test_body_frame_delta_and_skipped_scans_do_not_reset_absolute_baseline(self):
        m = self.module()
        frames = [odom(yaw=math.pi/2), frame(laser(), ns=10),
                  odom(y=1., yaw=math.pi/2, ns=20),
                  frame(laser(), ns=500000000),  # stale: not a new baseline
                  odom(y=2., yaw=-math.pi/2, ns=600000000),
                  frame(laser(scan_id=123), ns=600000010)]
        prepared = m.prepare(self.load(m, rows(frames)), lidar_yaw_deg=0)
        self.assertEqual(len(prepared['skipped']), 1)
        records = prepared['records']
        self.assertEqual(records[0]['request']['odom'], dict(dx=0., dy=0., dyaw=0.))
        delta = records[1]['request']['odom']
        self.assertAlmostEqual(delta['dx'], 2.)
        self.assertAlmostEqual(delta['dy'], 0.)
        self.assertAlmostEqual(abs(delta['dyaw']), math.pi)
        self.assertEqual(records[1]['scan_id'], 123)

    def test_missing_odom_and_empty_scans_are_explicitly_skipped(self):
        m = self.module()
        p = m.prepare(self.load(m, rows([frame(laser(), ns=1), odom(ns=2),
            frame(laser(ranges=(0., 0., 0.)), ns=3), frame(laser(), ns=4)])), lidar_yaw_deg=0)
        self.assertEqual([r['reason'] for r in p['skipped']], ['missing_odometry', 'no_usable_ranges'])
        self.assertEqual(len(p['records']), 1)

    def test_incomplete_footer_counts_header_order_and_types(self):
        m = self.module()
        good = rows([odom(), frame(laser())])
        bad = [[], good[:-1], [good[1], good[0], good[-1]], good + [good[1]]]
        for key, value in [('complete', False), ('queue_dropped', 1), ('frames_written', 3),
                           ('oversized', True), ('size_limited', -1)]:
            item = copy.deepcopy(good); item[-1][key] = value; bad.append(item)
        for key, value in [('source_clock_verified', True), ('max_payload_bytes', 10**30),
                           ('format', 'other')]:
            item = copy.deepcopy(good); item[0][key] = value; bad.append(item)
        for key, value in [('receive_offset_ns', -1), ('source_stamp', float('nan')),
                           ('source_seq', True), ('payload_base64', '!!!!'), ('index', 65536)]:
            item = copy.deepcopy(good); item[1][key] = value; bad.append(item)
        bad.append(rows([frame(laser(), ns=20), odom(ns=10)]))
        for data in bad:
            with self.subTest(data=str(data)[:150]), self.assertRaises(ValueError):
                self.load(m, data)
        self.assertFalse(self.load(m, good[:-1], allow_incomplete=True)['complete'])
        with patch.object(m, 'MAX_BYTES', 50), self.assertRaises(ValueError):
            self.load(m, good)
        with patch.object(m, 'MAX_FRAMES', 1), self.assertRaises(ValueError):
            self.load(m, good)

    def test_malformed_xdr_is_rejected_before_allocation(self):
        m = self.module()
        bad = [b'', laser()[:-1], laser()+b'\0'*4,
               laser(intensities=(65536, 1, 2)), laser(angles=(1.,)),
               laser(ranges=(float('nan'), 1., 2.)), laser(angles=(float('inf'), 1., 2.))]
        for offset, value in [(0, 0x7f800000), (4, 0xffffffff), (8, 2), (20, 0xffffffff)]:
            data = bytearray(laser()); struct.pack_into('>I', data, offset, value); bad.append(data)
        for data in bad:
            with self.subTest(data=bytes(data)[:25]), self.assertRaises(ValueError):
                m.decode_laser(data)
        for payload in (b'', struct.pack('>6dI', float('nan'), 0., 0., 0., 0., 0., 0)):
            with self.assertRaises(ValueError): self.load(m, rows([frame(payload, 4)]))

    def test_adapter_protocol_actual_diagnostics_no_truth_and_bounded_failure(self):
        m = self.module()
        capture = self.load(m, rows([odom(), frame(laser()), odom(x=1., ns=200000000),
                                      frame(laser(scan_id=92), ns=200000010)]))
        with tempfile.TemporaryDirectory() as tmp:
            adapter = Path(tmp) / 'adapter'
            adapter.write_text('''#!/usr/bin/python3 -I
import json,sys
for line in sys.stdin:
 r=json.loads(line)
 assert r['op'] in ('start_mapping','update')
 if r['op']=='update':
  assert 'stamp' not in r and 'motion_compensated' not in r
  print(json.dumps(dict(ok=True,pose=dict(x=1.,y=2.,yaw=.3), diagnostics=dict(accepted=False,used_points=3,mapping_paused=True,goodness=.4,reason='low_goodness'))),flush=True)
 else: print('{"ok":true}',flush=True)
''')
            adapter.chmod(0o700)
            result = m.run(adapter, capture, lidar_yaw_deg=0)
            self.assertEqual(result['updates'], 2)
            self.assertEqual(result['accepted_icp'], 0)
            self.assertEqual(result['map_insertions_paused'], 2)
            self.assertEqual(result['trajectory']['final_pose'], dict(x=1., y=2., yaw=.3))
            self.assertEqual(result['trajectory']['path_length_m'], 0.)
            self.assertEqual(result['results'][1]['diagnostics']['goodness'], .4)
            for key in ('final_xy_error_m', 'final_yaw_error_deg', 'wall_error_p95_m', 'walls_within_10cm'):
                self.assertIsNone(result[key])
            with patch.object(m, 'MAX_TOTAL_REPLY_BYTES', 300), self.assertRaises(ValueError):
                m.run(adapter, capture, lidar_yaw_deg=0)
            adapter.write_text('#!/bin/sh\nsleep 5\n')
            with self.assertRaises(TimeoutError):
                m.run(adapter, capture, lidar_yaw_deg=0, timeout_s=.1)
            adapter.write_text('#!/usr/bin/python3 -I\nimport sys\nsys.stdout.write("x"*5000000);sys.stdout.flush()\n')
            with self.assertRaises(ValueError): m.run(adapter, capture, lidar_yaw_deg=0)

    def test_map_out_saves_final_grid(self):
        """--map-out (#70): after replay the adapter {"op":"map"} grid is
        saved as JSON 0600 without overwriting an existing file."""
        import tempfile
        m = self.module()
        with tempfile.NamedTemporaryFile('w', suffix='.jsonl', delete=False) as fh:
            for row in rows([odom(), frame(laser())]):
                fh.write(json.dumps(row) + '\n')
            path = fh.name
        capture = m.load(path)
        reply = dict(ok=True, pose=dict(x=0.0, y=0.0, yaw=0.0), quality=0.9,
                     tracking='tracking',
                     diagnostics=dict(used_points=10, reference_points=10,
                                      accepted=True, reason='accepted',
                                      goodness=0.9, iterations=3, duration_ms=1.0,
                                      correspondences=10, residual_rms=0.01,
                                      mapping_paused=False))
        grid = dict(x_min=0.0, y_min=0.0, res=0.05, w=4, h=4,
                    cells=','.join(['0'] * 16))
        fake = (
            '#!/usr/bin/env python3\n'
            'import json, sys\n'
            f'reply = {reply!r}\n'
            f'grid = {grid!r}\n'
            'for line in sys.stdin:\n'
            '    req = json.loads(line)\n'
            '    if req.get("op") == "map":\n'
            '        print(json.dumps(dict(ok=True, grid=grid)), flush=True)\n'
            '    else:\n'
            '        print(json.dumps(reply), flush=True)\n')
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            adapter = td / 'adapter'
            adapter.write_text(fake)
            adapter.chmod(0o755)
            out = td / 'map.json'
            result = m.run(str(adapter), capture, lidar_yaw_deg=0., map_out=out)
            self.assertTrue(out.is_file(), 'map_out file missing')
            import stat as statmod
            self.assertEqual(statmod.S_IMODE(out.stat().st_mode), 0o600)
            data = json.loads(out.read_text())
            for key in ('x_min', 'y_min', 'res', 'w', 'h', 'cells'):
                self.assertIn(key, data)
            existing = td / 'again.json'
            existing.write_text('keep')
            with self.assertRaises(OSError):
                m.run(str(adapter), capture, lidar_yaw_deg=0., map_out=existing)
            self.assertEqual(existing.read_text(), 'keep')
            self.assertIsInstance(result, dict)

    def test_cli_requires_explicit_lidar_yaw(self):
        self.module()
        result = subprocess.run([sys.executable, '-I', str(TOOL), 'anything', '--adapter', '/trusted'], capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'--lidar-yaw-deg', result.stderr)


if __name__ == '__main__':
    unittest.main()
