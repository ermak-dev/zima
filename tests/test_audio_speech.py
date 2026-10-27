#!/usr/bin/env python3
"""FEATURE: One bounded, cancelable voice-command session with local recognition.
SPEC: No replay after gaps/restart; actual playback gates manual start; no motion authority.
STATUS: Wake activation, wake-prefix stripping and barge-in were removed; sessions start only by begin_utterance.
"""
from concurrent.futures import Future
from pathlib import Path
import struct
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from robot_audio.speech import (BoundedRecognizer, FasterWhisperASR, RecognizerBusy,
                                RmsVad, SpeechSession)


def block(seq=1, *, stream='one', speech=True, discontinuity=False):
    return SimpleNamespace(stream_id=stream, sequence=seq, stamp_ns=seq*20_000_000,
                           pcm=struct.pack('<h', 3000 if speech else 0)*320,
                           sample_rate=16000, channels=1, sample_format='s16le',
                           discontinuity=discontinuity)


class ManualRecognizer:
    def __init__(self): self.future = None; self.calls=[]
    def submit(self, pcm):
        self.calls.append(pcm); self.future=Future(); return self.future
    def close(self): pass


class SpeechTests(unittest.TestCase):
    def session(self, **kwargs):
        recognizer = ManualRecognizer()
        return SpeechSession(recognizer, vad=RmsVad(-40),
                             silence_s=.2, wait_s=1, max_command_s=2,
                             asr_timeout_s=1, echo_tail_s=.2, **kwargs), recognizer

    def spoken(self, session, recognizer):
        """Start a manual utterance, hear speech, then silence: ASR is submitted."""
        self.assertTrue(session.begin_utterance(now=0))
        session.feed(block(1), now=.05)
        session.feed(block(2, speech=False), now=.3)
        self.assertEqual(session.state, 'transcribing')

    def test_removed_wake_api_is_absent(self):
        session, _ = self.session()
        for name in ('on_wake', 'barge_in', '_strip_wake', 'wake_phrase'):
            self.assertFalse(hasattr(session, name), name)
        with self.assertRaises(TypeError):
            SpeechSession(ManualRecognizer(), wake_phrase='x')

    def test_transcript_is_returned_verbatim_without_prefix_removal(self):
        session, recognizer = self.session()
        self.spoken(session, recognizer)
        recognizer.future.set_result('Эй, робот, осмотрись в коридоре')
        transcript, = session.tick(now=.4)
        self.assertEqual(transcript.text, 'Эй, робот, осмотрись в коридоре')
        self.assertTrue(transcript.final)
        self.assertEqual(transcript.stream_id, 'one')
        self.assertEqual(session.state, 'processing')
        session.response_finished(transcript.session_id)
        self.assertEqual(session.state, 'waiting')

    def test_manual_audio_has_no_preroll(self):
        session, recognizer = self.session()
        self.spoken(session, recognizer)
        self.assertEqual(recognizer.calls[0], block(1).pcm + block(2, speech=False).pcm)

    def test_second_start_and_source_gap_do_not_start_two_commands(self):
        session, recognizer = self.session()
        self.assertTrue(session.begin_utterance(now=0))
        self.assertFalse(session.begin_utterance(now=.01))
        session.feed(block(1), now=.02)
        session.feed(block(4), now=.03)
        self.assertEqual(session.state, 'fault')
        self.assertEqual(session.last_error, 'audio_gap')
        self.assertEqual(recognizer.calls, [])

    def test_source_change_and_discontinuity_cancel_current_command(self):
        for changed in (block(2, stream='two'), block(2, discontinuity=True)):
            session, _ = self.session(); session.begin_utterance(now=0)
            session.feed(block(1), now=.01)
            session.feed(changed, now=.02)
            self.assertEqual(session.state, 'fault')

    def test_no_speech_and_maximum_duration_are_bounded(self):
        session, _ = self.session()
        session.begin_utterance(now=0)
        session.feed(block(1, speech=False), now=.01)
        session.tick(now=1.1)
        self.assertEqual(session.last_error, 'no_speech')
        session, recognizer = self.session(); session.begin_utterance(now=0)
        session.feed(block(1), now=0)
        session.tick(now=2.1)
        self.assertEqual(session.state, 'transcribing')
        self.assertEqual(len(recognizer.calls), 1)

    def test_asr_timeout_and_cancel_discard_late_result(self):
        for cancel in (False, True):
            session, recognizer = self.session(); self.spoken(session, recognizer)
            old = recognizer.future
            if cancel: session.cancel()
            else: session.tick(now=1.4)
            if not old.cancelled(): old.set_result('опасная старая команда')
            self.assertEqual(session.tick(now=1.5), [])
            self.assertEqual(session.transcripts, [])

    def test_only_actual_playback_blocks_start_and_tail_is_respected(self):
        session, _ = self.session()
        session.playback_started('p1', now=0)
        self.assertFalse(session.begin_utterance(now=.1))
        session.playback_finished('other', now=.2)
        self.assertTrue(session.playback_active)
        session.playback_finished('p1', now=.3)
        self.assertFalse(session.begin_utterance(now=.4))
        self.assertTrue(session.begin_utterance(now=.6))

    def test_fault_can_be_reset_and_a_new_utterance_started(self):
        session, _ = self.session(); session.begin_utterance(now=0)
        session.source_lost()
        self.assertEqual(session.state, 'fault')
        self.assertFalse(session.begin_utterance(now=1))
        session.reset()
        self.assertTrue(session.begin_utterance(now=1))

    def test_empty_transcript_does_not_become_command(self):
        session, recognizer = self.session(); self.spoken(session, recognizer)
        recognizer.future.set_result('  ')
        self.assertEqual(session.tick(now=.4), [])
        self.assertEqual(session.last_error, 'empty_command')


class RecognitionTests(unittest.TestCase):
    def test_bounded_worker_never_queues_a_second_job(self):
        release=threading.Event(); entered=threading.Event()
        def recognize(pcm): entered.set(); release.wait(2); return 'тест'
        recognizer=BoundedRecognizer(recognize)
        self.addCleanup(release.set)
        first=recognizer.submit(b'\x00\x00'*320)
        self.assertTrue(entered.wait(1))
        with self.assertRaises(RecognizerBusy): recognizer.submit(b'\x00\x00')
        release.set(); self.assertEqual(first.result(1), 'тест')
        recognizer.close()
        with self.assertRaises(RecognizerBusy): recognizer.submit(b'\x00\x00')

    def test_asr_uses_only_existing_local_model_and_russian(self):
        with tempfile.TemporaryDirectory() as d:
            calls={}
            class Model:
                def __init__(self,path,**kwargs): calls.update(path=path,kwargs=kwargs)
                def transcribe(self,audio,**kwargs):
                    calls['transcribe']=kwargs
                    self.audio=audio
                    return iter([SimpleNamespace(text=' привет'),SimpleNamespace(text=' мир')]), None
            with patch.dict(sys.modules, {'faster_whisper':SimpleNamespace(WhisperModel=Model)}):
                asr=FasterWhisperASR(d)
                self.assertEqual(asr(b'\x00\x00'*320), 'привет мир')
            self.assertTrue(calls['kwargs']['local_files_only'])
            self.assertEqual(calls['transcribe']['language'], 'ru')
            self.assertEqual(calls['kwargs']['device'], 'cpu')
        with self.assertRaises(ValueError): FasterWhisperASR('/does/not/exist')

    def test_vad_distinguishes_zero_and_loud_samples(self):
        vad=RmsVad(-40)
        self.assertFalse(vad(b'\x00\x00'*320))
        self.assertTrue(vad(struct.pack('<h',3000)*320))
        with self.assertRaises(ValueError): RmsVad(float('nan'))


if __name__ == '__main__': unittest.main()
