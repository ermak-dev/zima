#!/usr/bin/env python3
"""FEATURE: speech chunking splits text so playback starts before synthesis ends.

SPEC: ported from pireal voice/chunking; property, not implementation.
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from robot_audio.chunking import split_for_speech


def words(text):
    return ' '.join(text.split())


class ChunkingTest(unittest.TestCase):
    def assert_lossless(self, text, chunks):
        self.assertEqual(words(' '.join(chunks)), words(text),
                         'chunking must not drop or reorder any word')

    def test_empty_text_yields_nothing(self):
        self.assertEqual(split_for_speech(''), [])
        self.assertEqual(split_for_speech('   '), [])

    def test_short_text_stays_one_chunk(self):
        self.assertEqual(split_for_speech('Привет, Вася.'), ['Привет, Вася.'])

    def test_first_chunk_is_short_so_audio_starts_early(self):
        text = ('Он нам, когда мы были в Сезоне, очень смешно рассказал про мотор, '
                'а потом мы поехали домой без тебя.')
        chunks = split_for_speech(text, first_max=48)
        self.assertLessEqual(len(chunks[0]), 48)
        self.assertGreater(len(chunks), 1)
        self.assert_lossless(text, chunks)

    def test_later_chunks_may_be_longer_than_the_first(self):
        text = ' '.join(['Фраза номер %d, достаточно длинная.' % i for i in range(1, 8)])
        chunks = split_for_speech(text, first_max=40, rest_max=140)
        self.assertLessEqual(len(chunks[0]), 40)
        self.assertTrue(any(len(c) > 40 for c in chunks[1:]),
                        'later chunks should use the larger budget')
        self.assert_lossless(text, chunks)

    def test_a_needed_split_lands_on_a_sentence_boundary(self):
        text = 'Первое предложение довольно длинное. Второе предложение тоже.'
        chunks = split_for_speech(text, first_max=40, rest_max=40)
        self.assertEqual(chunks[0], 'Первое предложение довольно длинное.')
        self.assert_lossless(text, chunks)

    def test_short_sentences_pack_together_while_they_fit(self):
        chunks = split_for_speech('Да. Нет. Может быть.', first_max=48)
        self.assertEqual(chunks, ['Да. Нет. Может быть.'])

    def test_no_chunk_splits_a_word(self):
        text = 'Привет ' + 'длинноесловобезпробелов ' * 6
        chunks = split_for_speech(text, first_max=20, rest_max=30)
        for chunk in chunks:
            for word in chunk.split():
                self.assertIn(word, text.split(),
                              f'{word!r} is not a whole word from the input')
        self.assert_lossless(text, chunks)

    def test_a_single_overlong_word_is_not_lost(self):
        word = 'а' * 200
        chunks = split_for_speech(word, first_max=48)
        self.assert_lossless(word, chunks)

    def test_trailing_scrap_is_folded_back(self):
        chunks = split_for_speech('Это основная фраза здесь. Да.', first_max=48, rest_max=160)
        self.assertNotIn('Да.', chunks[1:], 'a tiny tail should merge, not stand alone')

    def test_whitespace_is_normalised(self):
        chunks = split_for_speech('  Привет,\n\n   Вася.  ')
        self.assertEqual(chunks, ['Привет, Вася.'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
