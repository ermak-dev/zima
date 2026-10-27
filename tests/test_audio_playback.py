#!/usr/bin/env python3
"""FEATURE: bounded playback with safe assets and explicit completion.
SPEC: fake transports only; queue acceptance is not audible completion.
"""
import io
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import wave

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from robot_audio.playback import AudioError, AssetStore, PlaybackQueue, TransportResult, SSHPlaybackTransport


def wav_data(seconds=.1, rate=16000, channels=1):
    b = io.BytesIO()
    with wave.open(b, 'wb') as f:
        f.setnchannels(channels); f.setsampwidth(2); f.setframerate(rate)
        f.writeframes(b'\0\0' * int(seconds * rate) * channels)
    return b.getvalue()


class PlaybackTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.now = 100.
        self.store = AssetStore(self.tmp.name, clock=lambda: self.now)
        self.asset = self.store.put(wav_data())
        self.calls = []
        self.result = TransportResult('completed')
        parent = self
        class Transport:
            def play(self, asset, volume, cancel_event, on_started=None, on_reference=None):
                parent.calls.append((asset.asset_id, volume))
                if on_started: on_started()
                if on_reference: on_reference(asset.pcm, asset.sample_rate, parent.now)
                return parent.result
        self.queue = PlaybackQueue(self.store, Transport(), clock=lambda: self.now)

    def test_acceptance_then_explicit_completion_and_reference(self):
        states, refs = [], []
        self.queue.on_state = lambda job: states.append(job.state)
        self.queue.on_reference = lambda *args: refs.append(args)
        job = self.queue.submit('one', self.asset.asset_id)
        self.assertEqual(job.state, 'queued'); self.assertEqual(self.calls, [])
        self.queue.run_next()
        self.assertEqual(job.state, 'completed')
        self.assertEqual(states, ['queued', 'playing', 'completed'])
        self.assertEqual(len(refs), 1)
        self.assertEqual(self.calls[0][1], .35)

    def test_priority_fifo_expiry_and_bounded_queue(self):
        low = self.queue.submit('low', self.asset.asset_id, priority=0)
        high = self.queue.submit('high', self.asset.asset_id, priority=2)
        self.assertEqual(self.queue.run_next().task_id, 'high')
        self.now += 6
        self.queue.run_next(); self.assertEqual(low.state, 'expired')
        q = PlaybackQueue(self.store, object(), max_queue=1, clock=lambda: self.now)
        q.submit('first', self.asset.asset_id)
        with self.assertRaises(AudioError): q.submit('second', self.asset.asset_id)

    def test_cancel_and_idempotency(self):
        a = self.queue.submit('one', self.asset.asset_id)
        self.assertIs(a, self.queue.submit('one', self.asset.asset_id))
        with self.assertRaises(AudioError): self.queue.submit('one', self.asset.asset_id, volume=.7)
        self.assertTrue(self.queue.cancel('one')); self.queue.run_next()
        self.assertEqual(a.state, 'cancelled'); self.assertEqual(self.calls, [])

    def test_unknown_and_busy_are_not_success_or_retry(self):
        for i, outcome in enumerate(('unknown', 'busy', 'error')):
            self.result = TransportResult(outcome)
            job = self.queue.submit(str(i), self.asset.asset_id)
            self.queue.run_next()
            self.assertNotEqual(job.state, 'completed')
        self.assertEqual(len(self.calls), 3)

    def test_reject_invalid_volume_and_assets(self):
        for volume in (-1, 1.1, float('nan'), float('inf')):
            with self.assertRaises(AudioError): self.queue.submit(str(volume), self.asset.asset_id, volume=volume)
        for value in ('../secret', 'file:///tmp/x', 'https://example.com/x', ''):
            with self.assertRaises(AudioError): self.store.get(value)
        with self.assertRaises(AudioError): self.store.put(b'not wav')
        with self.assertRaises(AudioError): self.store.put(wav_data(rate=8000))
        with self.assertRaises(AudioError): self.store.put(wav_data(channels=2))

    def test_private_files_expire_and_symlinks_rejected(self):
        p = Path(self.tmp.name) / (self.asset.asset_id + '.wav')
        self.assertEqual(p.stat().st_mode & 0o777, 0o600)
        p.unlink(); p.symlink_to('/etc/passwd')
        with self.assertRaises(AudioError): self.store.get(self.asset.asset_id)
        second = self.store.put(wav_data(seconds=.2))
        self.now += 3601
        with self.assertRaises(AudioError): self.store.get(second.asset_id)

    def test_expired_assets_are_removed_not_only_rejected(self):
        path = self.asset.path
        self.now += 3601
        self.assertEqual(self.store.purge(), 1)
        self.assertFalse(path.exists())
        with self.assertRaises(AudioError): self.store.get(self.asset.asset_id)

    def test_bounded_history_expires_without_permanently_disabling_speech(self):
        queue = PlaybackQueue(self.store, self.queue.transport, clock=lambda: self.now,
                              history_limit=2, history_ttl_s=10)
        queue.submit('first', self.asset.asset_id); queue.run_next()
        queue.submit('second', self.asset.asset_id); queue.run_next()
        with self.assertRaises(AudioError): queue.submit('recent-overflow', self.asset.asset_id)
        self.now += 11
        job = queue.submit('third', self.asset.asset_id)
        self.assertEqual(queue.run_next().task_id, 'third')
        self.assertEqual(job.state, 'completed')
        self.assertIsNone(queue.get('first'))

    def test_transport_cancel_requires_remote_acknowledgement(self):
        marker = Path(self.tmp.name) / 'cancelled'
        script = Path(self.tmp.name) / 'ssh-fake'
        script.write_text('#!/usr/bin/env python3\nimport pathlib,sys,time\np=pathlib.Path(' + repr(str(marker)) + ')\nif " cancel " in sys.argv[-1]:\n p.touch(); print("CANCELLED"); sys.exit(0)\nsys.stdin.buffer.read(); print("STARTED",flush=True)\nfor _ in range(100):\n if p.exists(): print("CANCELLED",flush=True); sys.exit(130)\n time.sleep(.02)\nsys.exit(1)\n')
        script.chmod(0o700)
        cancel = threading.Event()
        result = SSHPlaybackTransport(ssh=str(script)).play(self.asset, .35, cancel, on_started=cancel.set)
        self.assertEqual(result.outcome, 'cancelled')
        self.assertTrue(marker.exists())

    def test_transport_arguments_never_accept_shell_host_or_helper(self):
        for host in ('robot; touch /tmp/x', '-oProxyCommand=anything', 'robot host'):
            with self.assertRaises(AudioError): SSHPlaybackTransport(host=host)
        with self.assertRaises(AudioError): SSHPlaybackTransport(helper_path='/tmp/helper;echo x')
        transport = SSHPlaybackTransport()
        self.assertEqual(transport.host, 'robot')


if __name__ == '__main__': unittest.main()
