#!/usr/bin/env python3
"""FEATURE: read-only Russian battery answers, no digits or invented freshness."""
import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from robot_audio.commands import BatterySnapshot, BatteryProvider, battery_response, handle_command, snapshot_from_json


class CommandsTests(unittest.TestCase):
    def test_normalized_intents_and_no_other_commands(self):
        calls = []
        def reader():
            calls.append(True)
            return BatterySnapshot(percent=42, freshness='fresh')
        for text in ['Какой у тебя заряд?', '  СКОЛЬКО заря́да?! ', 'какой заряд', 'сколько у тебя заряда', 'какое тебя заряд', 'какой тебя заряд']:
            self.assertEqual(handle_command(text, reader), 'У меня сорок два процента заряда.')
        count = len(calls)
        for text in ['', 'езжай вперед', 'скажи какой у тебя заряд и езжай', None]:
            answer = handle_command(text, reader)
            self.assertIn('Неизвестная команда', answer)
            self.assertLess(len(answer), 160)
        self.assertEqual(len(calls), count)

    def test_every_integer_percent_has_words_and_correct_suffix(self):
        expected = {0:'ноль процентов',1:'один процент',2:'два процента',11:'одиннадцать процентов',21:'двадцать один процент',42:'сорок два процента',100:'сто процентов'}
        for value in range(101):
            answer = battery_response(BatterySnapshot(percent=value, freshness='fresh'))
            self.assertFalse(any(c.isdigit() for c in answer))
            self.assertLess(len(answer), 160)
            suffix = 'процентов' if 11 <= value % 100 <= 14 else ('процент' if value % 10 == 1 else 'процента' if value % 10 in (2,3,4) else 'процентов')
            self.assertTrue(answer.endswith(f'{suffix} заряда.'), answer)
            if value in expected: self.assertEqual(answer, f'У меня {expected[value]} заряда.')

    def test_unknown_invalid_stale_and_exceptions_are_honest(self):
        for value in [None, -1, 101, 10**1000, float('nan'), float('inf'), True, '42']:
            answer = battery_response(BatterySnapshot(percent=value, freshness='fresh'))
            self.assertIn('неизвестен', answer)
        for freshness in ['unknown','stale',None,'bogus']:
            answer = battery_response(BatterySnapshot(percent=42, freshness=freshness))
            self.assertNotIn('сорок',answer)
            self.assertIn('неизвестен',answer)
        def broken(): raise RuntimeError('secret and arbitrary user data')
        self.assertIn('неизвестен',handle_command('сколько заряда',broken))
        self.assertNotIn('secret',handle_command('сколько заряда',broken))
        self.assertIn('неизвестен',handle_command('сколько заряда',lambda: object()))
        fixed = BatteryProvider(BatterySnapshot(percent=0,freshness='fresh'))
        self.assertEqual(fixed(), BatterySnapshot(percent=0,freshness='fresh'))

    def test_json_has_bounded_input_validity_and_no_invented_age(self):
        raw = {'valid':25,'percent':42,'voltage':19.5,'charging':True,'freshness':'unknown','measurement_time':None}
        snapshot = snapshot_from_json(json.dumps(raw))
        self.assertEqual(snapshot.percent,42)
        self.assertEqual(snapshot.voltage,19.5)
        self.assertIs(snapshot.charging,True)
        self.assertEqual(snapshot.freshness,'unknown')
        # Transport receipt and caller-supplied JSON freshness are not sensor evidence.
        raw.update(valid=0, freshness='fresh', measurement_time=9999999999)
        snapshot = snapshot_from_json(json.dumps(raw))
        self.assertIsNone(snapshot.percent)
        self.assertIsNone(snapshot.voltage)
        self.assertIsNone(snapshot.charging)
        self.assertEqual(snapshot.freshness,'unknown')
        for text in ['not json','[]','x'*513,'{"valid":25,"percent":NaN}', '{"valid":true,"percent":42}']:
            self.assertEqual(snapshot_from_json(text),BatterySnapshot())

    def test_reported_value_is_explicitly_qualified_not_upgraded_to_current(self):
        snapshot=BatterySnapshot(percent=100,freshness='unknown')
        answer=handle_command('какой заряд',lambda:snapshot,allow_reported=True)
        self.assertIn('сто процентов',answer)
        self.assertIn('давность',answer.lower())
        self.assertNotIn('У меня',answer)
        self.assertEqual(snapshot.freshness,'unknown')
        self.assertNotIn('сто',handle_command('какой заряд',lambda:snapshot))

    def test_fractional_percent_is_explicitly_approximate(self):
        self.assertEqual(battery_response(BatterySnapshot(percent=42.4,freshness='fresh')), 'Примерно сорок два процента заряда.')

if __name__ == '__main__': unittest.main()
