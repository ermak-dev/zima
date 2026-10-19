#!/usr/bin/env python3
"""Lexical wake matching and pronunciation metadata, without audio or models."""
from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import sys
import unicodedata
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
from robot_audio.phrases import PhraseSpec, normalize_phrase, strip_wake_prefix


class NormalizePhraseTests(unittest.TestCase):
    def test_case_punctuation_whitespace_and_yo(self):
        self.assertEqual(normalize_phrase(' «ПРИВЕТ,\t Ёлка!» '), 'привет елка')
        self.assertEqual(normalize_phrase('эй—робот_7'), 'эй робот 7')
        self.assertEqual(normalize_phrase('...'), '')

    def test_canonical_unicode_and_acute_only(self):
        for text in ('Привет, Зи́ма', 'Привет, Зима́', 'ЭЙ, Ёлка', 'Café'):
            with self.subTest(text=text):
                self.assertEqual(normalize_phrase(unicodedata.normalize('NFC', text)),
                                 normalize_phrase(unicodedata.normalize('NFD', text)))
        self.assertEqual(normalize_phrase('ЭЙ, Ёлка'), 'эй елка')
        self.assertEqual(normalize_phrase('Café'), 'cafe')
        self.assertEqual(normalize_phrase('Зи́ма'), normalize_phrase('Зима́'))

    def test_no_stemming_or_fuzzy_match(self):
        self.assertNotEqual(normalize_phrase('Зимарь'), normalize_phrase('Зима'))
        self.assertNotEqual(normalize_phrase('приветзима'), normalize_phrase('Привет, Зима'))


class StripWakePrefixTests(unittest.TestCase):
    def test_full_prefix_preserves_original_command(self):
        self.assertEqual(strip_wake_prefix(' «ПРИВЕТ, Зи́ма!» — Включи Ёлку, на 5%! ',
                                          'Привет, Зима'), 'Включи Ёлку, на 5%!')
        self.assertEqual(strip_wake_prefix('ЭЙ, Ёлка: Скажи «Да!»', 'эй ёлка'),
                         'Скажи «Да!»')
        command = unicodedata.normalize('NFD', 'Включи зелёный свет!')
        self.assertEqual(strip_wake_prefix('Привет Зима, ' + command, 'Привет, Зима'), command)

    def test_asr_cleanup_is_explicit_and_only_one_final_word_substitution(self):
        text = 'Привет, Сима. Какое тебя заряд?'
        self.assertEqual(strip_wake_prefix(text, 'Привет, Зима'), text)
        self.assertEqual(strip_wake_prefix(text, 'Привет, Зима', allow_asr_substitution=True),
                         'Какое тебя заряд?')
        self.assertEqual(strip_wake_prefix('Привет Сима, НЕ включай свет!', 'Привет Зима',
                                          allow_asr_substitution=True), 'НЕ включай свет!')
        self.assertEqual(strip_wake_prefix('Привет Сима', 'Привет Зима',
                                          allow_asr_substitution=True), '')
        for text, phrase in (('Привет Зима2, команда', 'Привет Зима'),
                             ('Привет Зимарь, команда', 'Привет Зима'),
                             ('Привет Лира, команда', 'Привет Зима'),
                             ('Прибет Сима, команда', 'Привет Зима'),
                             ('Эй Привет Сима, команда', 'Привет Зима'),
                             ('Сима, команда', 'Зима'),
                             ('Привет ты, команда', 'Привет мы'),
                             ('Привет Зим4, команда', 'Привет Зима')):
            with self.subTest(text=text):
                self.assertEqual(strip_wake_prefix(text, phrase, allow_asr_substitution=True), text)

    def test_wake_only(self):
        for text in ('Привет Зима', '«Привет, Зима!»', 'привет, зима́...'):
            with self.subTest(text=text):
                self.assertEqual(strip_wake_prefix(text, 'Привет, Зи́ма'), '')

    def test_never_removes_partial_or_nonprefix_occurrence(self):
        for text in ('Привет, Зимарь, включи свет', 'Привет Зима2, свет',
                     'Приветзима, свет', 'Эй, Привет Зима, свет',
                     'Скажи: Привет Зима', 'Привет, свет', ''):
            with self.subTest(text=text):
                self.assertEqual(strip_wake_prefix(text, 'Привет, Зима'), text)

    def test_empty_phrase_is_noop_and_only_one_prefix_is_removed(self):
        self.assertEqual(strip_wake_prefix('  Команда!  ', '...'), 'Команда!')
        self.assertEqual(strip_wake_prefix('Привет Зима, Привет Зима!', 'Привет Зима'),
                         'Привет Зима!')


class PhraseSpecTests(unittest.TestCase):
    def test_explicit_positive_and_negative_stress_are_preserved(self):
        spec = PhraseSpec('Привет, Зима', 'Привет, Зи́ма', ('Привет, Зима́',))
        self.assertEqual(spec.text, 'Привет, Зима')
        self.assertEqual(spec.pronunciation, 'Привет, Зи́ма')
        self.assertEqual(spec.negative_pronunciations, ('Привет, Зима́',))
        self.assertEqual(spec.language, 'ru')
        with self.assertRaises(FrozenInstanceError):
            spec.text = 'Другое имя'

    def test_configurable_phrase_without_library_name_default(self):
        spec = PhraseSpec('Эй, Ёлка', unicodedata.normalize('NFD', 'Эй, Ёлка'))
        self.assertEqual(spec.negative_pronunciations, ())
        self.assertEqual(normalize_phrase(spec.pronunciation), normalize_phrase(spec.text))
        self.assertEqual(PhraseSpec('Hello robot', 'Hello robot', language='en').language, 'en')
        with self.assertRaises(TypeError):
            PhraseSpec()

    def test_rejects_empty_or_lexically_mismatched_metadata(self):
        for args in (('', ''), ('...', '...'), ('Привет Зима', ''),
                     ('Привет Зима', 'Привет Зимарь'),
                     ('Привет Зима', 'Привет Зи́ма', ('Пока Зима́',)),
                     ('Привет Зима', 'Привет Зи́ма', ('',)),
                     ('Привет Зима', 'Привет Зи́ма', ('привет, зи́ма!',))):
            with self.subTest(args=args), self.assertRaises(ValueError):
                PhraseSpec(*args)
        with self.assertRaises(ValueError):
            PhraseSpec('Hey', 'Hey', language=' ')

    def test_negative_collection_is_immutable_snapshot(self):
        negatives = ['Привет, Зима́']
        spec = PhraseSpec('Привет Зима', 'Привет Зи́ма', negatives)
        negatives.clear()
        self.assertEqual(spec.negative_pronunciations, ('Привет, Зима́',))
        with self.assertRaises(ValueError):
            PhraseSpec('Hey', 'Hey', 'Hey')

    def test_tts_dict_is_structured_json_data_not_spoken_instructions(self):
        spec = PhraseSpec('Привет, Зима', 'Привет, Зи́ма', ('Привет, Зима́',))
        expected = {'text': 'Привет, Зима', 'pronunciation': 'Привет, Зи́ма',
                    'negative_pronunciations': ['Привет, Зима́'], 'language': 'ru'}
        self.assertEqual(spec.to_tts_dict(), expected)
        self.assertEqual(json.loads(json.dumps(spec.to_tts_dict(), ensure_ascii=False)), expected)
        payload = spec.to_tts_dict()
        payload['negative_pronunciations'].clear()
        self.assertEqual(spec.to_tts_dict(), expected)


if __name__ == '__main__':
    unittest.main()
