"""FEATURE: explicit bounded camera-power/gain lease, offline fakes only.
SPEC: refuse disabled controller; restore gain then stock even on failures;
credentials never in argv/env; camera-local guard owns gain rollback.
"""
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from robot_audio.camera_session import (CameraSession, CameraGainLease, CameraSSH,
                                        CameraSessionError)


class Clock:
    def __init__(self): self.now = 0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class Process:
    pid = 987654
    code = None
    def poll(self): return self.code
    def wait(self, timeout):
        if self.code is None: raise subprocess.TimeoutExpired('hold', timeout)
        return self.code


class RobotRunner:
    def __init__(self, state='enabled'):
        self.state = state
        self.calls = []
        self.process = Process()
        self.fail = None
        self.marker = True
    def run(self, args, **kw):
        self.calls.append((args, kw))
        cmd = tuple(args[1:])
        if cmd == self.fail: raise OSError('failure')
        if cmd == ('disable',): self.state = 'disabled'
        if cmd == ('enable',): self.state = 'enabled'
        return subprocess.CompletedProcess(args, 0, self.state if cmd == ('status',) else '')
    def Popen(self, args, **kw):
        self.calls.append((args, kw))
        if self.marker:
            kw['stderr'].write(b'mask-owner: holding; send SIGTERM to stop\n')
            kw['stderr'].flush()  # Popen writes the descriptor, not Python's buffer.
        return self.process


class Gain:
    def __init__(self, fail=False): self.events = []; self.fail = fail
    def __enter__(self): self.events.append('enter'); return self
    def __exit__(self, *args):
        self.events.append('exit')
        if self.fail: raise CameraSessionError('gain_restore_unverified')
    def check(self): self.events.append('check')


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(); self.runner = RobotRunner(); self.signals = []
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.lock_path = Path(self.temp.name) / 'session.lock'
    def session(self, **kw):
        kw.setdefault('lock_path', self.lock_path)
        return CameraSession(explicit=True, runner=self.runner, clock=self.clock,
                             sleep=self.clock.sleep, killpg=self.killpg, **kw)
    def killpg(self, pid, sig):
        self.signals.append((pid, sig)); self.runner.process.code = -sig
    def commands(self): return [tuple(a[1:]) for a, _ in self.runner.calls]
    def test_requires_explicit_consent_and_bounded_integer_ttl(self):
        for kw in ({}, {'explicit': False}, {'explicit': True, 'ttl_s': 3601},
                   {'explicit': True, 'ttl_s': True}, {'explicit': True, 'ttl_s': 0}):
            with self.assertRaises(ValueError): CameraSession(**kw)
    def test_disabled_or_ambiguous_controller_never_touched(self):
        for state in ('disabled', 'something enabled', ''):
            self.runner = RobotRunner(state)
            with self.assertRaises(CameraSessionError):
                with self.session(): pass
            self.assertEqual(self.commands(), [('status',)])
    def test_concurrent_local_session_is_rejected_before_robot_access(self):
        with tempfile.TemporaryDirectory() as directory:
            lock=Path(directory)/'session.lock'
            other=RobotRunner()
            with self.session(lock_path=lock):
                with self.assertRaises(CameraSessionError):
                    with CameraSession(explicit=True,runner=other,lock_path=lock,
                        clock=self.clock,sleep=self.clock.sleep,killpg=self.killpg):pass
                self.assertEqual(other.calls,[])
                self.assertEqual(self.runner.state,'disabled')
            self.assertEqual(self.runner.state,'enabled')

    def test_hold_is_separate_group_long_ssh_budget_no_motion_then_restore(self):
        gain = Gain()
        with self.session(gain=gain) as session:
            self.assertGreater(session.remaining_s, 0)
            session.check()
        self.assertEqual(self.commands(), [('status',), ('disable',),
                         ('mask', 'hold', '--mask', '10f8ff'),
                         ('mask', 'restore'), ('enable',), ('status',)])
        _, kw = self.runner.calls[2]
        self.assertTrue(kw['start_new_session'])
        self.assertGreater(int(kw['env']['ROBOT_SSH_TIMEOUT_S']), 600)
        self.assertEqual(gain.events, ['enter', 'check', 'exit'])
        self.assertEqual(self.signals[0], (987654, signal.SIGTERM))
    def test_hold_log_poll_does_not_rewind_child_file_offset(self):
        with tempfile.TemporaryFile() as log:
            wrapped = Mock(wraps=log)
            with patch('robot_audio.camera_session.tempfile.TemporaryFile', return_value=wrapped):
                with self.session(): pass
            self.assertEqual(wrapped.seek.call_count, 0)
    def test_body_failure_restores_and_does_not_swallow(self):
        with self.assertRaisesRegex(RuntimeError, 'audio failed'):
            with self.session(): raise RuntimeError('audio failed')
        self.assertEqual(self.commands()[-3:], [('mask', 'restore'), ('enable',), ('status',)])
    def test_disable_unknown_outcome_still_restores(self):
        self.runner.fail = ('disable',)
        with self.assertRaises(CameraSessionError):
            with self.session(): pass
        self.assertEqual(self.commands()[-3:], [('mask', 'restore'), ('enable',), ('status',)])
    def test_hold_crash_and_ttl_fail_closed(self):
        for fail in ('crash', 'ttl'):
            self.runner = RobotRunner()
            with self.assertRaises(CameraSessionError):
                with self.session(ttl_s=10) as session:
                    if fail == 'crash': self.runner.process.code = 1
                    else: self.clock.now += 11
                    session.check()
            self.assertIn(('enable',), self.commands())
    def test_missing_holding_ack_has_startup_deadline(self):
        self.runner.marker = False
        with self.assertRaises(CameraSessionError):
            with self.session(startup_s=2): pass
        self.assertLess(self.clock.now, 4)
        self.assertIn(('enable',), self.commands())
    def test_gain_cleanup_failure_still_restores_stock_and_is_error(self):
        with self.assertRaisesRegex(CameraSessionError, 'restor'):
            with self.session(gain=Gain(True)): pass
        self.assertEqual(self.commands()[-1], ('status',))
    def test_nested_gain_cannot_reenable_signals_during_stock_cleanup(self):
        class NestedGain(Gain):
            def __exit__(inner, *args):
                signal.signal(signal.SIGTERM, lambda *args: None)
        original_run = self.runner.run
        def check_cleanup(args, **kw):
            if args[1:] == ['mask', 'restore']:
                self.assertEqual(signal.getsignal(signal.SIGTERM), signal.SIG_IGN)
            return original_run(args, **kw)
        self.runner.run = check_cleanup
        with self.session(gain=NestedGain()): pass
    def test_restore_error_does_not_skip_enable_or_verification(self):
        self.runner.fail = ('mask', 'restore')
        with self.assertRaises(CameraSessionError):
            with self.session(): pass
        self.assertEqual(self.commands()[-2:], [('enable',), ('status',)])
    def test_final_disabled_status_is_not_success(self):
        original_run = self.runner.run
        def never_enabled(args, **kw):
            result = original_run(args, **kw)
            if args[1:] == ['enable']: self.runner.state = 'disabled'
            return result
        self.runner.run = never_enabled
        with self.assertRaisesRegex(CameraSessionError, 'restore'):
            with self.session(): pass
    def test_hold_timeout_escalates_only_our_process_group(self):
        def ignore_term(pid, sig):
            self.signals.append((pid, sig))
            if sig == signal.SIGKILL: self.runner.process.code = -sig
        with CameraSession(explicit=True, runner=self.runner, killpg=ignore_term,
                           lock_path=self.lock_path): pass
        self.assertEqual(self.signals, [(987654, signal.SIGTERM), (987654, signal.SIGKILL)])
    def test_gain_health_check_respects_session_overall_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            credentials = Path(directory) / 'netrc'
            credentials.write_text('machine cam login admin password fake\n'); credentials.chmod(0o600)
            ssh_runner = SSHRunner()
            gain = CameraGainLease(CameraSSH(host='cam', netrc_path=credentials, runner=ssh_runner),
                                   readiness=lambda timeout: True, clock=self.clock, sleep=self.clock.sleep)
            ssh_runner.outputs = ['40\n', '40 90\n', '40 90\n', '40 40\n']
            with self.session(ttl_s=5, gain=gain) as session:
                self.clock.now += 4
                session.check()
                self.assertLessEqual(ssh_runner.calls[2][1]['timeout'], 1)
    # BUG: failed SSH restore cuts camera power before its TTL rollback.
    # HYPOTHESIS: gain cleanup needs a bounded powered grace with hold checks.
    # SOLUTION: retry verification through the remote TTL + 10s grace while
    # checking hold health, then always attempt bounded stock cleanup.
    # RESULT: FIXED in offline fake-clock tests; hardware remains unverified.
    def test_failed_restore_keeps_power_until_guard_ttl_and_retries(self):
        self._guard_recovery_case(recover=True)

    def test_unreachable_restore_waits_bounded_then_reports_unverified(self):
        self._guard_recovery_case(recover=False)

    def _guard_recovery_case(self, recover):
        attempts = []
        def restore(lease_id, **kw):
            attempts.append(self.clock.now)
            self.assertIsNone(self.runner.process.poll())
            self.assertEqual(self.runner.state, 'disabled')
            if recover and self.clock.now >= 6: return '40 40'
            raise CameraSessionError('camera_ssh_failed')
        ssh = SimpleNamespace(http_ready=lambda timeout: True,
            start=lambda *a, **kw: 40, ready=lambda *a, **kw: '40 90', restore=restore)
        gain = CameraGainLease(ssh, ttl_s=6, clock=self.clock, sleep=self.clock.sleep)
        def run():
            with self.session(gain=gain, ttl_s=3): pass
        if recover: run()
        else:
            with self.assertRaisesRegex(CameraSessionError, 'restore_unverified'): run()
        self.assertGreaterEqual(self.clock.now, 6)
        self.assertLessEqual(self.clock.now, 30)
        self.assertGreater(len(attempts), 1)
        self.assertEqual(self.runner.state, 'enabled')

    def test_dead_hold_aborts_guard_wait_but_still_enables_stock(self):
        def restore(*a, **kw):
            self.runner.process.code = 1
            raise CameraSessionError('camera_ssh_failed')
        ssh = SimpleNamespace(http_ready=lambda timeout: True,
            start=lambda *a, **kw: 40, ready=lambda *a, **kw: '40 90', restore=restore)
        gain = CameraGainLease(ssh, ttl_s=600, clock=self.clock, sleep=self.clock.sleep)
        with self.assertRaisesRegex(CameraSessionError, 'restore_unverified'):
            with self.session(gain=gain): pass
        self.assertLess(self.clock.now, 2)
        self.assertEqual(self.runner.state, 'enabled')

    # BUG: robot disable's legitimate 30s wait is killed by a 10s SSH cap.
    # HYPOTHESIS: robot operations should receive the remaining outer budget.
    # SOLUTION/RESULT: FIXED by separating robot allowance from camera SSH caps.
    def test_robot_disable_uses_remaining_startup_allowance(self):
        original = self.runner.run
        def slow_disable(args, **kw):
            if args[1:] == ['disable']:
                self.assertGreaterEqual(kw['timeout'], 30)
                self.clock.sleep(30)
            return original(args, **kw)
        self.runner.run = slow_disable
        with self.session(startup_s=120): pass
        self.assertEqual(self.runner.state, 'enabled')

    def test_sigterm_raises_and_restores_original_handler(self):
        before = signal.getsignal(signal.SIGTERM)
        with self.assertRaises(CameraSessionError):
            with self.session(): signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        self.assertEqual(signal.getsignal(signal.SIGTERM), before)
        self.assertIn(('enable',), self.commands())


class SSHRunner:
    def __init__(self): self.calls = []; self.outputs = ['40\n', '40 90\n', '40 40\n']
    def run(self, args, **kw):
        password = os.read(kw['pass_fds'][0], 4096)
        self.calls.append((args, kw.copy(), password))
        return subprocess.CompletedProcess(args, 0, self.outputs.pop(0), '')


class SSHTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.netrc = Path(self.temp.name) / 'camera.netrc'
        self.netrc.write_text('machine cam login admin password private-test-password\n')
        self.netrc.chmod(0o600); self.runner = SSHRunner()
    def ssh(self, **kw):
        return CameraSSH(host='cam', netrc_path=self.netrc, runner=self.runner, **kw)
    def test_secure_credentials_and_fixed_guard_scripts(self):
        ssh = self.ssh()
        with CameraGainLease(ssh, ttl_s=10, readiness=lambda timeout: True) as lease:
            self.assertEqual(lease.original_volume, 40)
        self.assertEqual(len(self.runner.calls), 3)
        scripts = []
        for args, kw, password in self.runner.calls:
            self.assertEqual(password, b'private-test-password\n')
            self.assertNotIn('private-test-password', repr(args) + repr(kw))
            self.assertIn('StrictHostKeyChecking=yes', args)
            self.assertIn('root@cam', args)
            self.assertEqual(args[-2:], ['sh', '-s'])
            self.assertNotIn('env', kw)
            scripts.append(kw['input'].decode())
        start = scripts[0]
        self.assertIn('yaml-cli -g .audio.volume', start)
        self.assertIn('yaml-cli -s .audio.volume 90', start)
        self.assertIn('pidof majestic', start)
        self.assertIn('nohup', start)
        self.assertIn('/tmp/robot-audio-camera-', start)
        self.assertIn('restored', start)
        self.assertNotIn('/api/v1/set', ''.join(scripts))
        self.assertNotIn('kill -9', ''.join(scripts))
    def test_no_default_netrc_no_symlinks_wrong_mode_or_owner(self):
        for contents in ('default login admin password secret\n',
                         'machine cam login root password secret\n'):
            self.netrc.write_text(contents)
            with self.assertRaises(CameraSessionError): self.ssh()
        self.netrc.write_text('machine cam login admin password secret\n')
        self.netrc.chmod(0o644)
        with self.assertRaises(CameraSessionError): self.ssh()
        self.netrc.chmod(0o600)
        link = Path(self.temp.name) / 'link'; link.symlink_to(self.netrc)
        with self.assertRaises(CameraSessionError):
            CameraSSH(host='cam', netrc_path=link)
        with patch('robot_audio.camera_session.os.getuid', return_value=os.getuid() + 1):
            with self.assertRaises(CameraSessionError): self.ssh()
    def test_invalid_hosts_volumes_ttl_and_uuid_rejected_without_execution(self):
        for host in ('-option', 'cam;id', 'root@cam', 'cam/foo'):
            with self.assertRaises(ValueError): CameraSSH(host=host, netrc_path=self.netrc)
        for volume in (True, -1, 101, '90;id'):
            with self.assertRaises(ValueError): CameraGainLease(self.ssh(), volume=volume)
        with self.assertRaises(ValueError): self.ssh().restore('../foreign')
        self.assertEqual(self.runner.calls, [])
    def test_readiness_bounded_before_any_gain_change(self):
        clock = Clock()
        lease = CameraGainLease(self.ssh(), startup_s=2, clock=clock,
                                sleep=clock.sleep, readiness=lambda timeout: False)
        with self.assertRaises(CameraSessionError):
            with lease: pass
        self.assertEqual(self.runner.calls, [])
        self.assertLess(clock.now, 4)
    def test_gain_readback_failure_attempts_restore(self):
        self.runner.outputs = ['40\n', '40 41\n', '40 40\n']
        with self.assertRaises(CameraSessionError):
            with CameraGainLease(self.ssh(), readiness=lambda timeout: True): pass
        self.assertEqual(len(self.runner.calls), 3)
    def test_unconfirmed_restore_is_error_even_if_body_failed(self):
        self.runner.outputs = ['40\n', '40 90\n', '40 90\n']
        with self.assertRaisesRegex(CameraSessionError, 'restor'):
            with CameraGainLease(self.ssh(), readiness=lambda timeout: True):
                raise RuntimeError('audio failed')
    def test_password_pipe_write_is_bounded_before_ssh(self):
        self.netrc.write_text('machine cam login admin password ' + 'x' * 8192 + '\n')
        with self.assertRaises(CameraSessionError): self.ssh()
        self.assertEqual(self.runner.calls, [])
    def test_check_detects_lost_gain_readback(self):
        self.runner.outputs = ['40\n', '40 90\n', '40 41\n', '40 40\n']
        with self.assertRaisesRegex(CameraSessionError, 'readback'):
            with CameraGainLease(self.ssh(), readiness=lambda timeout: True) as lease:
                lease.check()
        self.assertEqual(len(self.runner.calls), 4)
    def test_guard_uses_wall_deadline_and_scripts_parse_as_shell(self):
        with CameraGainLease(self.ssh(), readiness=lambda timeout: True): pass
        start = self.runner.calls[0][1]['input'].decode()
        self.assertIn('date +%s', start)
        self.assertIn('deadline=', start)
        for _, kw, _ in self.runner.calls:
            result = subprocess.run(['sh', '-n'], input=kw['input'], capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
    def test_ssh_failure_does_not_disclose_password_or_remote_output(self):
        def fail(args, **kw):
            raise subprocess.CalledProcessError(1, args, output='private-test-password')
        self.runner.run = fail
        with self.assertRaises(CameraSessionError) as error:
            self.ssh().start('a' * 32, 90, 10)
        self.assertNotIn('private-test-password', str(error.exception))
    def test_http_readiness_uses_authenticated_non_image_fixed_endpoint(self):
        response = io.BytesIO()
        with patch('robot_audio.camera_session.open_audio_stream', return_value=response) as open_:
            self.assertTrue(self.ssh().http_ready(2))
        args, kw = open_.call_args
        self.assertEqual(args, ('http://cam/', self.netrc))
        self.assertEqual(kw, {'timeout': 2})
        self.assertTrue(response.closed)


class GuardShellTests(unittest.TestCase):
    """Execute generated guard locally, with fake YAML, pidof and HUP only.

    SSH runner redirects every remote /tmp path into a fresh test directory.
    The shell kill builtin is overridden in BOTH launcher and detached guard;
    no signal can reach a real process. No network or devices are accessed.
    """
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.volume = self.root / 'volume'; self.volume.write_text('40\n')
        self.hups = self.root / 'hups'
        netrc = self.root / 'camera.netrc'
        netrc.write_text('machine cam login admin password fake\n'); netrc.chmod(0o600)
        tools = self.root / 'bin'; tools.mkdir()
        for name, script in {
            'yaml-cli': '#!/bin/sh\ncase "$1 $2" in\n'
                        '"-g .audio.volume") cat "$FAKE_CAMERA_VOLUME";;\n'
                        '"-s .audio.volume") printf "%s\\n" "$3" > "$FAKE_CAMERA_VOLUME";;\n'
                        '*) exit 1;; esac\n',
            'pidof': '#!/bin/sh\n[ "$1" = majestic ] || exit 1\nprintf "234567\\n"\n',
        }.items():
            executable = tools / name; executable.write_text(script); executable.chmod(0o700)
        self.env = dict(os.environ, PATH=str(tools) + os.pathsep + os.environ['PATH'],
                        FAKE_CAMERA_VOLUME=str(self.volume), FAKE_CAMERA_HUPS=str(self.hups))
        import robot_audio.camera_session as module
        self.module = module
        self.override = '\nkill() { printf "%s\\n" "$*" >> "$FAKE_CAMERA_HUPS"; }\n'
        patcher = patch.object(module, '_SHELL_VOLUME', module._SHELL_VOLUME + self.override)
        patcher.start(); self.addCleanup(patcher.stop)
        self.ssh = CameraSSH(host='cam', netrc_path=netrc, runner=SimpleNamespace(run=self.run_script))
    def run_script(self, args, **kw):
        # Fake SSH transport, not a command sent to ssh or any API.
        os.read(kw['pass_fds'][0], 4096)
        script = kw['input'].decode().replace('/tmp/robot-audio-camera', str(self.root / 'camera'))
        self.assertIn(self.override, script)
        return subprocess.run(['sh', '-s'], input=script.encode(), stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL, env=self.env, timeout=kw['timeout'], check=True)
    def wait_for(self, predicate):
        end = time.monotonic() + 4
        while not predicate():
            if time.monotonic() >= end: self.fail('local fake guard deadline exceeded')
            time.sleep(.02)
    def start(self, ttl):
        lease_id = 'a' * 32
        self.assertEqual(self.ssh.start(lease_id, 90, ttl), 40)
        self.wait_for(lambda: (self.root / ('camera-' + lease_id) / 'ready').exists())
        self.assertEqual(self.volume.read_text(), '90\n')
        return lease_id
    def test_done_restores_only_volume_and_confirms_marker(self):
        lease_id = self.start(10)
        self.assertEqual(self.ssh.ready(lease_id), '40 90')
        self.assertEqual(self.ssh.restore(lease_id), '40 40')
        self.assertEqual(self.volume.read_text(), '40\n')
        self.assertEqual(self.hups.read_text().splitlines(), ['-HUP 234567', '-HUP 234567'])
        self.assertFalse((self.root / 'camera.lock').exists())
    def test_detached_ttl_restores_without_host_done(self):
        lease_id = self.start(1)
        directory = self.root / ('camera-' + lease_id)
        self.wait_for(lambda: (directory / 'restored').exists())
        self.assertFalse((directory / 'done').exists())
        self.assertEqual(self.volume.read_text(), '40\n')
        with self.assertRaises(CameraSessionError): self.ssh.ready(lease_id)
        self.assertEqual(self.ssh.restore(lease_id), '40 40')
    def test_second_lease_refuses_existing_guard_without_mutation(self):
        lease_id = self.start(10)
        try:
            with self.assertRaises(CameraSessionError): self.ssh.start('b' * 32, 60, 10)
            self.assertEqual(self.volume.read_text(), '90\n')
        finally:
            self.assertEqual(self.ssh.restore(lease_id), '40 40')
    # BUG: pre-launch failures strand the global camera gain lock.
    # HYPOTHESIS: launcher must roll back only its own pre-guard artifacts.
    # SOLUTION/RESULT: FIXED via pre-launch traps, disarmed before uncertain
    # guard launch; executable fake shell tests preserve all foreign ownership.
    def test_prelaunch_failures_release_owned_lock_and_data(self):
        for failure in ('read', 'invalid', 'write', 'guard_write'):
            with self.subTest(failure=failure):
                lease_id = 'd' * 32
                directory = self.root / ('camera-' + lease_id)
                self.volume.write_text('invalid\n' if failure == 'invalid' else '40\n')
                yaml = self.root / 'bin/yaml-cli'
                original = yaml.read_text()
                if failure == 'read': yaml.write_text('#!/bin/sh\nexit 1\n')
                real_run = subprocess.run
                def injected_run(args, **kw):
                    if failure in ('write', 'guard_write'):
                        artifact = 'original' if failure == 'write' else 'guard.sh'
                        kw['input'] = kw['input'].replace(b'original=$(read_volume)',
                            f'mkdir "$d/{artifact}"\noriginal=$(read_volume)'.encode(), 1)
                    return real_run(args, **kw)
                try:
                    with patch('robot_audio.camera_session.subprocess.run', side_effect=injected_run):
                        with self.assertRaises(CameraSessionError): self.ssh.start(lease_id, 90, 10)
                    self.assertFalse((self.root / 'camera.lock').exists())
                    self.assertFalse(directory.exists())
                finally: yaml.write_text(original)

    def test_uncertain_launch_conservatively_keeps_lock_and_data(self):
        # The background launch was attempted but no guard actually ran. Do not
        # infer safe ownership from success/failure of the launcher response.
        nohup = self.root / 'bin/nohup'
        nohup.write_text('#!/bin/sh\nexit 1\n'); nohup.chmod(0o700)
        lease_id = 'f' * 32
        self.assertEqual(self.ssh.start(lease_id, 90, 10), 40)
        directory = self.root / ('camera-' + lease_id)
        self.assertTrue((self.root / 'camera.lock').exists())
        self.assertTrue((directory / 'guard.sh').exists())
        self.assertEqual(self.volume.read_text(), '40\n')

    def test_existing_lease_directory_is_never_removed(self):
        directory = self.root / ('camera-' + 'e' * 32)
        directory.mkdir(); (directory / 'foreign').write_text('keep')
        with self.assertRaises(CameraSessionError): self.ssh.start('e' * 32, 90, 10)
        self.assertEqual((directory / 'foreign').read_text(), 'keep')
        self.assertFalse((self.root / 'camera.lock').exists())

    def test_invalid_saved_volume_never_executed_or_applied(self):
        self.volume.write_text('40; touch injected\n')
        with self.assertRaises(CameraSessionError): self.ssh.start('c' * 32, 90, 10)
        self.assertEqual(self.volume.read_text(), '40; touch injected\n')
        self.assertFalse(self.hups.exists())


if __name__ == '__main__': unittest.main()
