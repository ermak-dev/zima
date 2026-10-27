#!/usr/bin/env python3
"""FEATURE: Explicit speaker mute and timezone-aware quiet hours do not mute input."""
from datetime import datetime, timezone
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))


class OutputPolicyTests(unittest.TestCase):
    def setUp(self):
        try:
            from robot_audio.policy import OutputPolicy
        except ImportError:
            self.fail('speaker mute/quiet-hours policy is not implemented')
        self.Policy = OutputPolicy

    def test_daily_window_crosses_midnight_in_the_configured_timezone(self):
        now = [datetime(2026, 10, 18, 21, 30, tzinfo=timezone.utc)]
        policy = self.Policy(quiet_hours=['22:00-08:00'], timezone_name='UTC', clock=lambda: now[0])
        self.assertFalse(policy.inhibited)
        now[0] = datetime(2026, 10, 18, 22, 0, tzinfo=timezone.utc)
        self.assertTrue(policy.inhibited)
        now[0] = datetime(2026, 10, 19, 7, 59, tzinfo=timezone.utc)
        self.assertTrue(policy.inhibited)
        now[0] = datetime(2026, 10, 19, 8, 0, tzinfo=timezone.utc)
        self.assertFalse(policy.inhibited)

    def test_timezone_is_not_assumed_to_be_the_host_wall_clock(self):
        clock = lambda: datetime(2026, 10, 18, 20, 0, tzinfo=timezone.utc)
        policy = self.Policy(quiet_hours=['22:00-08:00'], timezone_name='Europe/Moscow', clock=clock)
        self.assertTrue(policy.inhibited)

    def test_explicit_mute_is_independent_of_the_schedule(self):
        policy = self.Policy()
        self.assertFalse(policy.inhibited)
        policy.set_muted(True); self.assertTrue(policy.inhibited)
        policy.set_muted(False); self.assertFalse(policy.inhibited)
        with self.assertRaises(ValueError): policy.set_muted('false')

    def test_invalid_or_ambiguous_schedules_are_rejected(self):
        for value in (['25:00-08:00'], ['22:70-08:00'], ['22:00-22:00'], '22:00-08:00', ['bad']):
            with self.subTest(value=value):
                with self.assertRaises(ValueError): self.Policy(quiet_hours=value)
        with self.assertRaises(ValueError): self.Policy(timezone_name='invalid/timezone')


if __name__ == '__main__': unittest.main()
