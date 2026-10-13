#!/usr/bin/env python3
"""Unit tests for ros2/camera_mjpeg.py and ros2/web_panel.py (pure logic).

Same pattern as tests/test_sensor_overlay.py: the MJPEG frame parser and
the panel's JSON state serialization / command clamping are module-level
pure functions, so these tests run with a bare python3.
"""
import importlib.util
import math
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


cam = load("camera_mjpeg", ROOT / "ros2" / "camera_mjpeg.py")
panel = load("web_panel", ROOT / "ros2" / "web_panel.py")


def jpeg(payload=b"imgdata"):
    return b"\xff\xd8" + payload + b"\xff\xd9"


class MjpegParser(unittest.TestCase):
    def test_two_complete_frames(self):
        a, b = jpeg(b"AAA"), jpeg(b"BBB")
        frames, rest = cam.split_mjpeg_frames(a + b)
        self.assertEqual(frames, [a, b])
        self.assertEqual(rest, b"")

    def test_partial_frame_is_kept_as_remainder(self):
        a = jpeg(b"AAA")
        part = b"\xff\xd8BB\xff"  # SOI but no EOI yet
        frames, rest = cam.split_mjpeg_frames(a + part)
        self.assertEqual(frames, [a])
        self.assertEqual(rest, part)

    def test_leading_garbage_before_soi_is_dropped(self):
        a = jpeg(b"AAA")
        frames, rest = cam.split_mjpeg_frames(b"HTTP--boundary\r\n" + a)
        self.assertEqual(frames, [a])
        self.assertEqual(rest, b"")

    def test_empty_buffer(self):
        self.assertEqual(cam.split_mjpeg_frames(b""), ([], b""))

    def test_streamed_concatenation(self):
        # feed the buffer in two halves: nothing lost across chunks
        whole = jpeg(b"AA") + jpeg(b"BBBB")
        cut = len(jpeg(b"AA")) + 3  # lands mid-second-frame on purpose
        frames1, rest = cam.split_mjpeg_frames(whole[:cut])
        self.assertEqual(frames1, [jpeg(b"AA")])
        frames2, rest = cam.split_mjpeg_frames(rest + whole[cut:])
        self.assertEqual(frames2, [jpeg(b"BBBB")])
        self.assertEqual(rest, b"")


class PanelState(unittest.TestCase):
    def test_full_state_serializes(self):
        state = {
            "battery": (0.87, 20.06, True),
            "bumper": False,
            "wall": 0.055,
            "cliff": [0.0, 0.0, 1.5, 0.0],
            "drop": [2.25, 0.0],
            "sonar": 65.534,
            "odom": (1.25, -0.5, 0.1, 0.4),
        }
        js = panel.state_to_json(state)
        self.assertEqual(js["battery"]["percent"], 87)
        self.assertEqual(js["battery"]["volts"], 20.1)
        self.assertTrue(js["battery"]["charging"])
        self.assertEqual(js["bumper"], "clear")
        self.assertEqual(js["wall_ir"], 0.055)
        self.assertEqual(js["cliff_ir"], [0.0, 0.0, 1.5, 0.0])
        self.assertEqual(js["drop_ir"], [2.25, 0.0])
        self.assertEqual(js["sonar"], 65.534)
        self.assertAlmostEqual(js["odom"]["x"], 1.25)
        # yaw comes from the quaternion, not the tuple; here it stays 0
        self.assertAlmostEqual(js["odom"]["yaw"], 0.0)
        self.assertAlmostEqual(js["odom"]["yaw_rate"], 0.4)

    def test_empty_state_gives_empty_sections(self):
        js = panel.state_to_json({})
        self.assertNotIn("battery", js)
        self.assertNotIn("odom", js)

    def test_bumper_pressed(self):
        self.assertEqual(panel.state_to_json({"bumper": True})["bumper"],
                         "PRESSED")

    def test_scan_serializes_to_robot_frame_points(self):
        angles, points = panel.scan_to_points(
            angle_min=-math.pi / 2, angle_increment=math.pi / 2,
            ranges=[1.0, 2.0, float("inf")], range_max=6.0)
        # out-of-range beams are dropped; robot frame x forward, y left
        self.assertEqual(len(points), 2)
        self.assertEqual(angles[0], -90)
        self.assertAlmostEqual(points[0][0], 0.0, places=5)   # cos(-90)
        self.assertAlmostEqual(points[0][1], -1.0, places=5)  # sin(-90)


class CmdClamp(unittest.TestCase):
    def test_clamps_to_project_limits(self):
        self.assertEqual(panel.clamp_cmd(0.5, 2.0), (0.1, 0.4))
        self.assertEqual(panel.clamp_cmd(-0.5, -2.0), (-0.1, -0.4))

    def test_passes_through_within_limits(self):
        self.assertEqual(panel.clamp_cmd(0.05, -0.2), (0.05, -0.2))

    def test_rejects_non_finite(self):
        self.assertEqual(panel.clamp_cmd(float("nan"), 0.0), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
