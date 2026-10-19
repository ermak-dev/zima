#!/usr/bin/env python3
"""FEATURE: Authenticated, bounded OpenIPC audio capture without live hardware.

SPEC: Decode one HTTP Ogg/Opus stream to 16 kHz mono s16le; mark gaps,
reject unsafe credentials/URLs, and never replay queued stale audio.
"""
import array
import base64
import importlib
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2'))


class CaptureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.capture = importlib.import_module('robot_audio.capture')

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.netrc = Path(self.temp.name) / 'camera.netrc'
        self.netrc.write_text('machine 127.0.0.1 login offline-user password offline-secret\n')
        self.netrc.chmod(0o600)

    def test_credentials_fail_closed_without_leaking_secret(self):
        self.netrc.chmod(0o644)
        with self.assertRaises(self.capture.CaptureError) as error:
            self.capture.open_audio_stream('http://127.0.0.1/audio.opus', self.netrc)
        self.assertEqual(error.exception.code, 'credentials')
        self.assertNotIn('offline-secret', str(error.exception))
        self.netrc.chmod(0o600)
        link = self.netrc.with_name('link.netrc')
        link.symlink_to(self.netrc)
        with self.assertRaises(self.capture.CaptureError):
            self.capture.open_audio_stream('http://127.0.0.1/audio.opus', link)
        for url in ('http://user:secret@127.0.0.1/audio.opus', 'file:///etc/passwd',
                    'http://127.0.0.1/audio.opus#secret'):
            with self.subTest(url=url), self.assertRaises(self.capture.CaptureError) as error:
                self.capture.open_audio_stream(url, self.netrc)
            self.assertEqual(error.exception.code, 'invalid_url')

    def test_missing_host_and_file_are_not_anonymous_fallbacks(self):
        for path, url in ((self.netrc.with_name('missing'), 'http://127.0.0.1/audio.opus'),
                          (self.netrc, 'http://localhost/audio.opus')):
            with self.subTest(path=path), self.assertRaises(self.capture.CaptureError) as error:
                self.capture.open_audio_stream(url, path)
            self.assertEqual(error.exception.code, 'credentials')

    def test_default_netrc_entry_cannot_authorize_an_unmatched_host(self):
        self.netrc.write_text('default login offline-user password offline-secret\n')
        with self.assertRaises(self.capture.CaptureError) as error:
            self.capture.open_audio_stream('http://localhost/audio.opus', self.netrc)
        self.assertEqual(error.exception.code, 'credentials')

    def test_framer_handles_split_samples_and_uses_sample_timestamps(self):
        framer = self.capture.PCMFramer('session', frame_ms=20, stamp_ns=1_000_000_000)
        self.assertEqual(framer.feed(b'\x01'), [])
        chunks = framer.feed(b'\x00' * 1279)
        self.assertEqual(len(chunks), 2)
        self.assertEqual([x.sequence for x in chunks], [0, 1])
        self.assertEqual([x.stamp_ns for x in chunks], [1_000_000_000, 1_020_000_000])
        self.assertEqual([len(x.pcm) for x in chunks], [640, 640])
        self.assertTrue(chunks[0].discontinuity)
        self.assertFalse(chunks[1].discontinuity)
        self.assertTrue(chunks[0].estimated_timestamp)
        self.assertEqual(chunks[0].pcm[:2], b'\x01\x00')
        with self.assertRaises(ValueError):
            framer.feed(b'\0' * 100000)

    def test_queue_keeps_latest_and_marks_dropped_and_stale_audio(self):
        framer = self.capture.PCMFramer('session', stamp_ns=1)
        chunks = framer.feed(b'\0' * 640 * 4)
        q = self.capture.LatestAudioQueue(2, clock=lambda: 10.0)
        for chunk in chunks:
            q.put(chunk)
        first = q.get(timeout=0)
        self.assertEqual(first.sequence, 2)
        self.assertTrue(first.discontinuity)
        self.assertEqual(q.dropped, 2)
        self.assertEqual(q.get(timeout=0).sequence, 3)
        with self.assertRaises(queue.Empty):
            q.get(timeout=0)
        now = [1.0]
        q = self.capture.LatestAudioQueue(2, clock=lambda: now[0])
        q.put(chunks[1])
        now[0] = 3.0
        with self.assertRaises(queue.Empty):
            q.get(timeout=0, max_age_s=.1)
        q.put(chunks[2])
        self.assertTrue(q.get(timeout=0).discontinuity)
        self.assertEqual(q.dropped, 1)

    def test_configuration_rejects_unbounded_or_invalid_values(self):
        for kwargs in ({'queue_size': 0}, {'frame_ms': 0}, {'stall_timeout': 0},
                       {'connect_timeout': float('nan')}, {'max_reconnects': -1},
                       {'backoff_s': -1}, {'max_frame_age_s': float('inf')}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.capture.CaptureConfig(**kwargs)

    def server(self, body, *, redirect=False, unauthorized=False, hold=False):
        test = self
        requests = []

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                requests.append(self.headers.get('Authorization'))
                if redirect:
                    self.send_response(302)
                    self.send_header('Location', 'http://127.0.0.1:1/credentials-leak')
                    self.end_headers()
                    return
                if unauthorized:
                    self.send_response(401)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.end_headers()
                try:
                    self.wfile.write(body)
                    self.wfile.flush()
                    if hold:
                        test.stop_server.wait(4)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.stop_server = threading.Event()
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(thread.join, 2)
        self.addCleanup(server.shutdown)
        self.addCleanup(self.stop_server.set)
        return f'http://127.0.0.1:{server.server_port}/audio.opus', requests

    def test_authenticated_http_and_redirect_refusal(self):
        url, requests = self.server(b'local-fake')
        with self.capture.open_audio_stream(url, self.netrc, timeout=1) as response:
            self.assertEqual(response.read(), b'local-fake')
        expected = 'Basic ' + base64.b64encode(b'offline-user:offline-secret').decode()
        self.assertEqual(requests, [expected])
        url, _ = self.server(b'', redirect=True)
        with self.assertRaises(self.capture.CaptureError) as error:
            self.capture.open_audio_stream(url, self.netrc, timeout=1)
        self.assertEqual(error.exception.code, 'http_error')
        self.assertNotIn('offline-secret', str(error.exception))

    def test_authentication_error_is_sanitized(self):
        url, _ = self.server(b'', unauthorized=True)
        with self.assertRaises(self.capture.CaptureError) as error:
            self.capture.open_audio_stream(url, self.netrc, timeout=1)
        self.assertEqual(error.exception.code, 'unauthorized')
        self.assertNotIn('offline-secret', str(error.exception))

    @unittest.skipUnless(shutil.which('ffmpeg'), 'ffmpeg is needed for local decoder test')
    def test_real_http_opus_decodes_frames_and_reconnects_with_new_epoch(self):
        generated = subprocess.run([
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'lavfi', '-i',
            'sine=frequency=880:sample_rate=48000:duration=0.36', '-ac', '1',
            '-c:a', 'libopus', '-f', 'ogg', 'pipe:1'], capture_output=True, check=True)
        url, requests = self.server(generated.stdout)
        source = self.capture.CameraAudioSource(self.capture.CaptureConfig(
            url=url, netrc_path=str(self.netrc), queue_size=64, frame_ms=20,
            max_reconnects=1, backoff_s=.01, connect_timeout=1, stall_timeout=1,
            max_frame_age_s=5))
        self.addCleanup(source.stop)
        source.start()
        chunks = []
        deadline = time.monotonic() + 7
        while time.monotonic() < deadline:
            try:
                chunks.append(source.get_chunk(timeout=.1))
            except queue.Empty:
                if source.state()['state'] == 'error':
                    break
        self.assertGreater(len(chunks), 10)
        self.assertTrue(all(len(x.pcm) == 640 for x in chunks))
        self.assertTrue(all((x.sample_rate, x.channels, x.sample_format) == (16000, 1, 's16le') for x in chunks))
        epochs = {x.stream_id for x in chunks}
        self.assertEqual(len(epochs), 2)
        for epoch in epochs:
            first = next(x for x in chunks if x.stream_id == epoch)
            self.assertEqual(first.sequence, 0)
            self.assertTrue(first.discontinuity)
        values = array.array('h', b''.join(x.pcm for x in chunks))
        self.assertGreater(max(map(abs, values)), 500)
        self.assertEqual(len(requests), 2)
        self.assertEqual(source.state()['error'], 'reconnect_exhausted')
        source.stop()
        self.assertEqual(source.state()['state'], 'stopped')
        self.assertFalse(source.state()['microphone_verified'])

    @unittest.skipUnless(shutil.which('ffmpeg'), 'ffmpeg is needed for live-stream fixture')
    def test_decoder_delivers_pcm_before_http_eof(self):
        generated = subprocess.run([
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'lavfi', '-i',
            'sine=frequency=880:sample_rate=48000:duration=2', '-ac', '1',
            '-c:a', 'libopus', '-f', 'ogg', 'pipe:1'], capture_output=True, check=True)
        url, _ = self.server(generated.stdout, hold=True)
        source = self.capture.CameraAudioSource(self.capture.CaptureConfig(
            url=url, netrc_path=str(self.netrc), queue_size=128,
            connect_timeout=2, stall_timeout=2, max_reconnects=0))
        self.addCleanup(source.stop)
        started = time.monotonic()
        source.start()
        chunk = source.get_chunk(timeout=1.5)
        self.assertLess(time.monotonic() - started, 1.5)
        self.assertEqual(len(chunk.pcm), 640)
        self.assertEqual(source.state()['state'], 'streaming')
        source.stop()

    @unittest.skipUnless(shutil.which('ffmpeg'), 'ffmpeg is needed for corrupt input test')
    def test_corrupt_ogg_reports_decoder_failure_not_quiet_audio(self):
        url, _ = self.server(b'not-an-ogg-stream')
        source = self.capture.CameraAudioSource(self.capture.CaptureConfig(
            url=url, netrc_path=str(self.netrc), max_reconnects=0,
            connect_timeout=.3, stall_timeout=1))
        self.addCleanup(source.stop)
        source.start()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and source.state()['state'] != 'error':
            threading.Event().wait(.02)
        self.assertEqual(source.state()['state'], 'error')
        self.assertEqual(source.state()['last_failure'], 'decoder_error')
        with self.assertRaises(queue.Empty):
            source.get_chunk(timeout=0)

    @unittest.skipUnless(shutil.which('ffmpeg'), 'ffmpeg is needed for cancellation test')
    def test_stop_interrupts_an_active_network_read(self):
        url, _ = self.server(b'', hold=True)
        source = self.capture.CameraAudioSource(self.capture.CaptureConfig(
            url=url, netrc_path=str(self.netrc), max_reconnects=0,
            connect_timeout=.3, stall_timeout=2, max_frame_age_s=.5))
        self.addCleanup(source.stop)
        source.start()
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not source.state()['stream_id']:
            threading.Event().wait(.01)
        self.assertIsNotNone(source.state()['stream_id'])
        started = time.monotonic()
        source.stop()
        self.assertLess(time.monotonic() - started, 1.5)
        self.assertEqual(source.state()['state'], 'stopped')
        self.assertFalse(source._thread.is_alive())

    @unittest.skipUnless(shutil.which('ffmpeg'), 'ffmpeg is needed for stalled decoder test')
    def test_empty_stream_stalls_and_stop_is_bounded(self):
        url, _ = self.server(b'', hold=True)
        source = self.capture.CameraAudioSource(self.capture.CaptureConfig(
            url=url, netrc_path=str(self.netrc), max_reconnects=0,
            connect_timeout=.3, stall_timeout=.3, max_frame_age_s=.5))
        self.addCleanup(source.stop)
        source.start()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and source.state()['state'] != 'error':
            threading.Event().wait(.02)
        self.assertEqual(source.state()['state'], 'error')
        self.assertEqual(source.state()['last_failure'], 'stalled')
        started = time.monotonic()
        source.stop()
        self.assertLess(time.monotonic() - started, 1.5)
        with self.assertRaises(queue.Empty):
            source.get_chunk(timeout=0)


if __name__ == '__main__':
    unittest.main()
