#!/usr/bin/env python3
"""Unit tests for ros2/sensor_overlay.py (pure logic, no rclpy needed).

The overlay node keeps its formatting and layout in module-level pure
functions so this test runs with a bare python3, exactly like
tests/test_robot_bug.py does for bin/bug.
"""
import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "sensor_overlay", ROOT / "ros2" / "sensor_overlay.py")
overlay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(overlay)


class FormatState(unittest.TestCase):
    def test_battery_line(self):
        lines = overlay.format_state(
            {"battery": (0.87, 20.1, True)})
        self.assertEqual(lines, ["battery 87% 20.1V charging"])

    def test_battery_not_charging(self):
        lines = overlay.format_state({"battery": (1.0, 19.9, False)})
        self.assertEqual(lines, ["battery 100% 19.9V discharging"])

    def test_bumper_states(self):
        self.assertEqual(overlay.format_state({"bumper": True}),
                         [("bumper PRESSED", True)])
        self.assertEqual(overlay.format_state({"bumper": False}),
                         ["bumper clear"])

    def test_sonar_and_wall_ranges(self):
        lines = overlay.format_state({"sonar": 65.534, "wall": 0.055})
        self.assertIn("sonar 65.534 m", lines)
        self.assertIn("wall IR 0.055 m", lines)

    def test_voltages_arrays(self):
        lines = overlay.format_state(
            {"cliff": [0.0, 0.0, 1.5, 0.0], "drop": [2.25, 0.0]})
        self.assertIn("cliff IR 0.00 0.00 1.50 0.00", lines)
        self.assertIn("drop IR 2.25 0.00", lines)

    def test_empty_and_missing_values_are_skipped(self):
        self.assertEqual(overlay.format_state({}), [])
        # absent / empty sensors never add lines
        self.assertEqual(
            overlay.format_state({"cliff": [], "drop": None, "wall": None}),
            [])

    def test_odometry_line(self):
        lines = overlay.format_state({"odom": (1.25, -0.5, 0.1, 0.4)})
        self.assertIn("odom x1.25 y-0.50  v 0.10 m/s 0.40 rad/s", lines)


class Layout(unittest.TestCase):
    def test_line_z_spacing(self):
        layout = overlay.label_layout(["a", ("b", True), "c"])
        # one slot per line, stacked upward from the base height
        self.assertEqual([slot[0] for slot in layout], [0, 1, 2])
        self.assertAlmostEqual(layout[0][2], overlay.BASE_HEIGHT)
        self.assertAlmostEqual(layout[1][2],
                               overlay.BASE_HEIGHT + overlay.LINE_HEIGHT)
        # plain strings are neutral, tuples carry an alert flag
        self.assertFalse(layout[0][1])
        self.assertTrue(layout[1][1])


if __name__ == "__main__":
    unittest.main()
