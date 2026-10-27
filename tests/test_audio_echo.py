#!/usr/bin/env python3
"""FEATURE: Real SpeexDSP echo cancellation with guarded playback alignment.
SPEC: 20 ms mono 16 kHz, explicit calibration, no unreferenced barge-in.
"""
import array
import ctypes.util
import math
from pathlib import Path
import random
import struct
import sys
from types import SimpleNamespace
import threading
from unittest.mock import patch
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from robot_audio.echo import EchoController, SpeexEchoCanceller


def block(seq, value=100, stream='input', stamp_ns=None):
    return SimpleNamespace(stream_id=stream, sequence=seq,
                           stamp_ns=seq*20_000_000 if stamp_ns is None else stamp_ns,
                           pcm=struct.pack('<h',value)*320, sample_rate=16000,
                           channels=1, sample_format='s16le', discontinuity=False)


class FakeEcho:
    def __init__(self): self.resets=0; self.calls=[]
    def process(self, mic, ref): self.calls.append((mic,ref)); return mic
    def reset(self): self.resets+=1
    def close(self): pass


class AlignmentTests(unittest.TestCase):
    def test_unverified_configuration_falls_back_to_half_duplex(self):
        controller=EchoController(FakeEcho(), delay_ms=100, calibrated=False)
        result=controller.process_capture(block(5), playback_active=True)
        self.assertEqual(result.mode,'half_duplex')
        self.assertFalse(result.allow_barge_in)
        self.assertEqual(result.reason,'calibration_required')

    def test_actual_aligned_reference_allows_full_duplex(self):
        engine=FakeEcho();controller=EchoController(engine, delay_ms=100, calibrated=True)
        controller.push_reference(block(0,stream='p1'), playback_id='play1', actual_playback=True)
        result=controller.process_capture(block(5), playback_active=True)
        self.assertEqual(result.mode,'full_duplex')
        self.assertTrue(result.allow_barge_in)
        self.assertEqual(len(engine.calls),1)

    def test_prepared_not_playing_audio_is_not_accepted_as_reference(self):
        controller=EchoController(FakeEcho(), delay_ms=0, calibrated=True)
        with self.assertRaises(ValueError):
            controller.push_reference(block(0), playback_id='p1', actual_playback=False)

    def test_missing_and_drifting_reference_fail_closed(self):
        engine=FakeEcho();controller=EchoController(engine,delay_ms=100,calibrated=True,max_drift_ms=5)
        self.assertFalse(controller.process_capture(block(5),playback_active=True).allow_barge_in)
        controller.push_reference(block(0,stream='p1'),playback_id='p1',actual_playback=True)
        result=controller.process_capture(block(7),playback_active=True)
        self.assertEqual(result.mode,'half_duplex')
        self.assertFalse(result.allow_barge_in)
        self.assertGreater(engine.resets,0)

    def test_capture_gap_invalidates_reference_and_engine(self):
        engine=FakeEcho();controller=EchoController(engine,delay_ms=0,calibrated=True)
        controller.push_reference(block(0,stream='p1'),playback_id='p1',actual_playback=True)
        controller.process_capture(block(0),playback_active=True)
        controller.push_reference(block(2,stream='p1'),playback_id='p1',actual_playback=True)
        result=controller.process_capture(block(2),playback_active=True)
        self.assertEqual(result.mode,'half_duplex')
        self.assertEqual(result.reason,'audio_gap')

    def test_no_playback_preserves_near_end_and_cancel_clears_reference(self):
        engine=FakeEcho();controller=EchoController(engine,delay_ms=0,calibrated=True)
        original=block(0,333)
        result=controller.process_capture(original,playback_active=False)
        self.assertEqual(result.pcm,original.pcm)
        self.assertFalse(result.allow_barge_in)
        controller.push_reference(block(0,stream='p1'),playback_id='p1',actual_playback=True)
        controller.playback_finished('p1')
        self.assertFalse(controller.process_capture(block(1),playback_active=True).allow_barge_in)

    def test_reference_storage_bounded(self):
        controller=EchoController(FakeEcho(),delay_ms=0,calibrated=True,max_reference_s=1)
        for i in range(200):
            controller.push_reference(block(i,stream='p1'),playback_id='p1',actual_playback=True)
        self.assertLessEqual(controller.reference_frames,50)


class SpeexTests(unittest.TestCase):
    def test_close_waits_for_in_flight_native_processing(self):
        entered=threading.Event(); release=threading.Event(); closed=threading.Event()
        destroyed=[]
        class Function:
            def __init__(self, fn): self.fn=fn
            def __call__(self,*args): return self.fn(*args)
        def cancel(*args): entered.set(); release.wait(1)
        fake=SimpleNamespace(speex_echo_state_init=Function(lambda *a:1),
             speex_echo_state_destroy=Function(lambda *a:destroyed.append(True)),
             speex_echo_state_reset=Function(lambda *a:None),
             speex_echo_ctl=Function(lambda *a:0), speex_echo_cancellation=Function(cancel))
        with patch('robot_audio.echo.ctypes.CDLL',return_value=fake):
            echo=SpeexEchoCanceller(library='offline-fake')
        worker=threading.Thread(target=lambda:echo.process(b'\0'*640,b'\0'*640))
        worker.start(); self.assertTrue(entered.wait(1))
        closer=threading.Thread(target=lambda:(echo.close(),closed.set()))
        closer.start()
        try: self.assertFalse(closed.wait(.03))
        finally:
            release.set(); worker.join(1); closer.join(1)
        self.assertTrue(closed.is_set()); self.assertEqual(destroyed,[True])

    def test_frame_size_validation_and_reset(self):
        with SpeexEchoCanceller() as echo:
            self.assertEqual(len(echo.process(b'\0'*640,b'\0'*640)),640)
            with self.assertRaises(ValueError): echo.process(b'\0'*638,b'\0'*640)
            echo.reset()
        with self.assertRaises(RuntimeError): echo.process(b'\0'*640,b'\0'*640)

    def test_real_library_attenuates_synthetic_echo_and_preserves_double_talk(self):
        rng=random.Random(824)
        reference=[rng.randint(-7000,7000) for _ in range(320*300)]
        mic=[int(.55*reference[max(0,i-64)]) for i in range(len(reference))]
        output=[]
        with SpeexEchoCanceller(filter_length=3200) as echo:
            for i in range(0,len(reference),320):
                ref=struct.pack('<320h',*reference[i:i+320])
                rec=struct.pack('<320h',*mic[i:i+320])
                values=array.array('h',echo.process(rec,ref));output.extend(values)
            tail=output[-16000:]; baseline=mic[-16000:]
            power=lambda a:sum(v*v for v in a)/len(a)
            attenuation=10*math.log10(power(baseline)/max(1,power(tail)))
            self.assertGreater(attenuation,8)
            near=[];clean=[]
            for frame in range(80):
                index=frame*320
                local=[int(3000*math.sin(2*math.pi*431*(index+j)/16000)) for j in range(320)]
                far=[rng.randint(-7000,7000) for _ in range(320)]
                combined=[int(.55*v)+n for v,n in zip(far,local)]
                clean.extend(array.array('h',echo.process(struct.pack('<320h',*combined),struct.pack('<320h',*far))))
                near.extend(local)
            dot=sum(a*b for a,b in zip(near[-16000:],clean[-16000:]))
            denom=math.sqrt(power(near[-16000:])*power(clean[-16000:]))*16000
            self.assertGreater(dot/denom,.55)


if __name__ == '__main__': unittest.main()
