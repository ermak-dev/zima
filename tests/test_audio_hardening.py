#!/usr/bin/env python3
"""FEATURE: Cancellation and credentials remain safe at protocol boundaries."""
import io
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
import wave

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from robot_audio.playback import AssetStore, AudioError, PlaybackQueue, TransportResult
from robot_audio.tts import OpenRouterTTS


def wav():
    data = io.BytesIO()
    with wave.open(data, 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
        w.writeframes(b'\x00\x00' * 1600)
    return data.getvalue()


class AudioHardeningTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.store = AssetStore(self.tmp.name)
        self.asset = self.store.put(wav())

    def test_cancel_during_remote_start_waits_for_acknowledgement(self):
        entered, release = threading.Event(), threading.Event()
        class Transport:
            def play(self, asset, volume, cancel_event, on_started=None, on_reference=None):
                entered.set(); release.wait(2)
                return TransportResult('cancelled' if cancel_event.is_set() else 'error')
        queue = PlaybackQueue(self.store, Transport())
        job = queue.submit('starting', self.asset.asset_id)
        worker = threading.Thread(target=queue.run_next)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.assertTrue(queue.cancel('starting'))
            self.assertNotEqual(job.state, 'cancelled', 'remote stop has not been acknowledged yet')
        finally:
            release.set(); worker.join(2)
        self.assertEqual(job.state, 'cancelled')

    def test_existing_assets_are_validated_without_unbounded_reads(self):
        with patch.object(Path, 'read_bytes', side_effect=AssertionError('unbounded asset read')):
            self.assertEqual(self.store.put(wav()).asset_id, self.asset.asset_id)

    def test_fifo_asset_is_rejected_without_blocking(self):
        import os
        self.asset.path.unlink()
        os.mkfifo(self.asset.path, 0o600)
        code = ('import sys;sys.path.insert(0,' + repr(str(ROOT / 'python')) + ');'
                'from robot_audio.playback import AssetStore,AudioError;'
                's=AssetStore(' + repr(self.tmp.name) + ');'
                '\ntry:s.get(' + repr(self.asset.asset_id) + ')\nexcept AudioError:sys.exit(0)\nsys.exit(1)')
        process = subprocess.Popen([sys.executable, '-I', '-c', code], stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
        try:
            try:
                result = process.wait(timeout=.7)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
                self.fail('asset lookup blocked on a FIFO')
            self.assertEqual(result, 0)
        finally:
            if process.poll() is None: process.kill(); process.wait()

    def test_token_fifo_is_rejected_without_blocking(self):
        import os
        path = Path(self.tmp.name) / 'token.fifo'
        os.mkfifo(path, 0o600)
        code = ('import sys;sys.path.insert(0,' + repr(str(ROOT / 'python')) + ');'
                'from robot_audio.tts import OpenRouterTTS;from robot_audio.playback import AudioError;'
                '\ntry:OpenRouterTTS(token_file=' + repr(str(path)) + ')._token()\nexcept AudioError:sys.exit(0)\nsys.exit(1)')
        process = subprocess.Popen([sys.executable, '-I', '-c', code], stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
        try:
            try:
                result = process.wait(timeout=.7)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
                self.fail('token lookup blocked on a FIFO')
            self.assertEqual(result, 0)
        finally:
            if process.poll() is None: process.kill(); process.wait()

    def test_provider_redirect_does_not_forward_bearer_credentials(self):
        calls = []
        class Destination(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def do_GET(self):
                calls.append(self.headers.get('Authorization'))
                self.send_response(200); self.end_headers()
                self.wfile.write(b'data: [DONE]\n\n')
        destination = ThreadingHTTPServer(('127.0.0.1', 0), Destination)
        class Redirect(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def do_POST(self):
                self.rfile.read(int(self.headers.get('Content-Length', '0')))
                self.send_response(302)
                self.send_header('Location', f'http://127.0.0.1:{destination.server_port}/audio')
                self.end_headers()
        redirect = ThreadingHTTPServer(('127.0.0.1', 0), Redirect)
        servers = (destination, redirect)
        threads = [threading.Thread(target=s.serve_forever, daemon=True) for s in servers]
        for thread in threads: thread.start()
        try:
            with patch.object(OpenRouterTTS, 'ENDPOINT', f'http://127.0.0.1:{redirect.server_port}/tts'):
                with self.assertRaises(AudioError):
                    OpenRouterTTS(model='test/model', voice='test', api_key='test-key').synthesize('test')
            self.assertEqual(calls, [], 'provider redirects must not leak the Authorization header')
        finally:
            for server in servers: server.shutdown(); server.server_close()
            for thread in threads: thread.join(2)


if __name__ == '__main__': unittest.main()
