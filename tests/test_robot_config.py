#!/usr/bin/env python3
# FEATURE: One private robot TOML configuration shared by the CLI and its children.
# SPEC: Repo-relative default; explicit --config; CLI > env > TOML > defaults;
#       owner-only 0600 regular files; no secret diagnostics or shell evaluation;
#       preserve audio JSON --config semantics; validate before any robot access.
# IMPLEMENTATION: Secure descriptor-based TOML loader and data-only environment re-exec.
# RESULT: IMPLEMENTED — defaults, precedence, private files, consent and CLI lifecycle covered.
# STATUS: DONE
"""Offline contract tests. All configuration and credentials below are synthetic."""

import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
import threading
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "python" / "robot_config.py"
FAKE_KEY = "synthetic-test-key-not-a-credential"
TOML = '''[robot]
ssh_host = "vacuum-test"
player_host = "192.0.2.23"
player_port = 6666
[server]
listen = "127.0.0.2"
port = 9085
[supervisor]
host = "192.0.2.24"
port = 9765
timeout_ms = 8000
[run]
slam_adapter = ""
yolo_adapter = ""
[camera]
netrc = "camera.netrc"
snapshot_url = "http://camera.test/image.jpg"
mjpeg_url = "http://camera.test/mjpeg"
audio_url = "http://camera.test/audio.opus"
[audio]
profile = "audio.json"
endpoint = "http://127.0.0.1:9086"
volume = 0.45
[yandex]
api_key = "synthetic-test-key-not-a-credential"
url = "https://tts.example.test/synthesize"
voice = "alena"
folder_id = "test-folder"
'''


def private_file(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    path.chmod(0o600)
    return path


class ConfigLoaderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = private_file(self.root / "etc" / "robot.toml", TOML)

    def module(self):
        if hasattr(self, "_module"):
            return self._module
        # Assert instead of an import error: RED must identify missing feature,
        # not suggest a missing external dependency or service.
        self.assertTrue(MODULE.is_file(), "shared python/robot_config.py is not implemented")
        spec = importlib.util.spec_from_file_location("robot_config", MODULE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self._module = module
        return module

    def load(self, path=None, environ=None):
        return self.module().load_config(path, root=self.root,
                                         environ={} if environ is None else environ)

    def test_default_path_is_repository_etc_not_current_directory(self):
        with patch("os.getcwd", return_value="/unrelated"):
            config = self.load()
        self.assertEqual(config["server"]["port"], 9085)
        self.assertEqual(self.module().DEFAULT_CONFIG, ROOT / "etc" / "robot.toml")

    def test_missing_implicit_config_keeps_legacy_defaults(self):
        self.path.unlink()
        config = self.load()
        self.assertEqual(config["server"], {"listen": "127.0.0.1", "port": 8085})
        self.assertEqual(config["supervisor"]["port"], 7665)
        self.assertEqual(config["supervisor"]["timeout_ms"], 5000)
        self.assertEqual(config["robot"]["ssh_host"], "robot")
        self.assertEqual(config["audio"]["volume"], 0.35)
        self.assertEqual(config["yandex"]["voice"], "filipp")
        self.assertTrue(config["run"]["slam_adapter"].endswith("tools/mrpt-adapter/adapter"))
        self.assertTrue(config["run"]["yolo_adapter"].endswith("tools/yolo/adapter"))

    def test_speech_endpoint_follows_server_port_unless_explicitly_overridden(self):
        self.path.write_text('[server]\nlisten = "0.0.0.0"\nport = 9185\n')
        self.assertEqual(self.load()["audio"]["endpoint"], "http://127.0.0.1:9185")
        self.assertEqual(self.load(environ={"ROBOT_API_ENDPOINT":"http://remote.test:8085"})["audio"]["endpoint"], "http://remote.test:8085")
        self.path.write_text('[server]\nlisten = "::"\nport = 9185\n')
        self.assertEqual(self.load()["audio"]["endpoint"], "http://[::1]:9185")

    def test_missing_explicit_config_is_an_error_not_fallback(self):
        module = self.module()
        with self.assertRaises(module.ConfigError):
            module.load_config(self.root / "missing.toml", environ={})

    def test_empty_private_file_uses_defaults(self):
        self.path.write_text("")
        self.assertEqual(self.load()["server"]["port"], 8085)

    def test_relative_paths_are_relative_to_toml_directory(self):
        config = self.load(self.path)
        self.assertEqual(config["camera"]["netrc"], str(self.path.parent / "camera.netrc"))
        self.assertEqual(config["audio"]["profile"], str(self.path.parent / "audio.json"))

    def test_environment_overrides_toml_without_mutating_environment(self):
        environ = {"ROBOT_YANDEX_VOICE": "filipp", "ROBOT_AUDIO_VOLUME": "0.2",
                   "ROBOT_SLAM_ADAPTER": "/env/slam", "ROBOT_API_ENDPOINT": "http://127.0.0.1:8088"}
        before = dict(environ)
        config = self.load(environ=environ)
        self.assertEqual(config["yandex"]["voice"], "filipp")
        self.assertEqual(config["audio"]["volume"], 0.2)
        self.assertEqual(config["run"]["slam_adapter"], "/env/slam")
        self.assertEqual(config["audio"]["endpoint"], "http://127.0.0.1:8088")
        self.assertEqual(environ, before)

    def test_supervisor_combined_address_environment_overrides_config(self):
        config = self.load(environ={'ROBOT_SUPERVISOR_ADDR': '127.0.0.3:9988'})
        self.assertEqual(config['supervisor']['host'], '127.0.0.3')
        self.assertEqual(config['supervisor']['port'], 9988)
        env = self.module().export_env(config, environ={})
        self.assertEqual(env['ROBOT_SUPERVISOR_ADDR'], '127.0.0.3:9988')

    def test_export_env_preserves_explicit_environment_over_toml(self):
        # BUG: export_env overwrote explicit ROBOT_SLAM_* flags with TOML defaults,
        # silently disabling verified deskew at runtime.
        # HYPOTHESIS: export must keep caller-provided environment values.
        env = self.module().export_env(self.load(), environ={
            'ROBOT_SLAM_SOURCE_TIME_VERIFIED': 'true',
            'ROBOT_SLAM_DESKEW_ENABLED': 'true',
            'ROBOT_SLAM_BEAM_TIME_VERIFIED': 'true',
        })
        self.assertEqual(env['ROBOT_SLAM_SOURCE_TIME_VERIFIED'], 'true')
        self.assertEqual(env['ROBOT_SLAM_DESKEW_ENABLED'], 'true')
        self.assertEqual(env['ROBOT_SLAM_BEAM_TIME_VERIFIED'], 'true')
        self.assertEqual(env['ROBOT_SLAM_ICP_MIN_GOODNESS'], '0.3')

    def test_export_contract_for_rust_audio_and_camera_children(self):
        module = self.module()
        env = module.export_env(self.load(), environ={})
        expected = {
            "ROBOT_YANDEX_URL": "https://tts.example.test/synthesize",
            "ROBOT_YANDEX_VOICE": "alena", "ROBOT_YANDEX_FOLDER_ID": "test-folder",
            "ROBOT_SUPERVISOR_ADDR": "192.0.2.24:9765",
            "ROBOT_AUDIO_VOLUME": "0.45", "ROBOT_API_ENDPOINT": "http://127.0.0.1:9086",
            "ROBOT_CAMERA_NETRC": str(self.path.parent / "camera.netrc"),
            "ROBOT_CAMERA_URL": "http://camera.test/mjpeg",
            "ROBOT_CAMERA_SNAPSHOT_URL": "http://camera.test/image.jpg",
            "ROBOT_CAMERA_AUDIO_URL": "http://camera.test/audio.opus",
            "ROBOT_AUDIO_CONFIG": str(self.path.parent / "audio.json"),
            "ROBOT_SSH_HOST": "vacuum-test", "ROBOT_PLAYER_HOST": "192.0.2.23",
            "ROBOT_PLAYER_PORT": "6666",
        }
        for name, value in expected.items():
            with self.subTest(name=name):
                self.assertEqual(env.get(name), value)
        self.assertTrue(env.get("ROBOT_YANDEX_API_KEY") == FAKE_KEY)
        self.assertFalse(any("ALLOW_EXTERNAL" in name for name in env),
                         "presence of a key must not implicitly consent to cloud audio")

    def test_refuse_group_or_world_permissions_even_without_secrets(self):
        module = self.module()
        self.path.write_text("[server]\nport = 8085\n")
        for mode in (0o644, 0o640, 0o660, 0o400):
            with self.subTest(mode=oct(mode)):
                self.path.chmod(mode)
                with self.assertRaises(module.ConfigError):
                    self.load()

    def test_refuse_symlinks_and_nonregular_files(self):
        module = self.module()
        link = self.root / "link.toml"
        link.symlink_to(self.path)
        for path in (link, self.root):
            with self.subTest(path=path.name):
                with self.assertRaises(module.ConfigError):
                    module.load_config(path, environ={})

    def test_refuse_wrong_owner(self):
        module = self.module()
        with patch.object(module.os, "getuid", return_value=os.getuid() + 1):
            with self.assertRaises(module.ConfigError):
                module.load_config(self.path, environ={})

    def test_schema_rejects_unknown_keys_wrong_types_and_invalid_ranges(self):
        module = self.module()
        cases = ('[server]\nport = 0', '[server]\nport = 65536',
                 '[server]\nport = true', '[server]\nport = "8085"',
                 '[supervisor]\ntimeout_ms = 99', '[supervisor]\ntimeout_ms = 300001',
                 '[audio]\nvolume = -0.1', '[audio]\nvolume = 1.1',
                 '[audio]\nvolume = nan', '[server]\nprot = 8085',
                 '[unknown]\nvalue = 1')
        for text in cases:
            with self.subTest(text=text):
                self.path.write_text(text)
                with self.assertRaises(module.ConfigError):
                    self.load()

    def test_parse_and_validation_errors_never_echo_secret_values(self):
        module = self.module()
        for text in ('[yandex]\napi_key = "' + FAKE_KEY,
                     '[server]\nport = "' + FAKE_KEY + '"'):
            self.path.write_text(text)
            with self.assertRaises(module.ConfigError) as caught:
                self.load()
            self.assertNotIn(FAKE_KEY, str(caught.exception))

    def test_consent_supervisor_and_provider_settings(self):
        self.path.write_text('[supervisor]\ntoken = "synthetic-token"\nbinary = "supervisor"\nlisten = "0.0.0.0"\n[audio]\nmax_volume = 0.9\n[yandex]\nenabled = true\n[providers]\nopenrouter_api_key = "synthetic-provider"\n')
        config = self.load()
        env = self.module().export_env(config, environ={})
        self.assertEqual(env['ROBOT_YANDEX_ENABLED'], 'true')
        self.assertEqual(env['ROBOT_SUPERVISOR_TOKEN'], 'synthetic-token')
        self.assertEqual(env['ROBOT_AUDIO_MAX_VOLUME'], '0.9')
        self.assertEqual(env['ROBOT_PROVIDER_OPENROUTER_API_KEY'], 'synthetic-provider')
        self.assertEqual(config['supervisor']['binary'], str(self.path.parent / 'supervisor'))

    def test_cloud_disabled_and_shared_endpoint_by_default(self):
        self.path.write_text('')
        config = self.load()
        self.assertFalse(config['yandex']['enabled'])
        self.assertEqual(config['audio']['endpoint'], 'http://127.0.0.1:8085')
        self.assertTrue(config['camera']['netrc'].endswith('/robot/camera.netrc'))

    def test_values_are_data_not_shell_code(self):
        marker = self.root / "executed"
        self.path.write_text('[yandex]\napi_key = "$(touch ' + str(marker) + ')"\n')
        config = self.load()
        self.assertTrue(config["yandex"]["api_key"].startswith("$(touch "))
        self.assertFalse(marker.exists())


class ConfigCliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "repo"
        (self.root / "bin").mkdir(parents=True)
        # Copy only public source files; never read a real .env, private TOML,
        # netrc, user audio profile, or any host credential file.
        shutil.copy2(ROOT / "bin" / "robot", self.root / "bin" / "robot")
        if MODULE.is_file():
            (self.root / "python").mkdir()
            shutil.copy2(MODULE, self.root / "python" / "robot_config.py")
        config_cli = ROOT / "bin" / "config.py"
        if config_cli.is_file():
            shutil.copy2(config_cli, self.root / "bin" / "config.py")
        restore = self.root / "crates" / "mask-owner" / "controller-restore.sh"
        restore.parent.mkdir(parents=True)
        shutil.copy2(ROOT / "crates" / "mask-owner" / "controller-restore.sh", restore)
        self.config = private_file(self.root / "etc" / "robot.toml", TOML)
        self.outside = Path(self.temp.name) / "elsewhere"
        self.outside.mkdir()
        self.capture = Path(self.temp.name) / "child.json"
        self.ssh_log = Path(self.temp.name) / "ssh.log"
        fake = self.root / "bin" / "fake-child"
        fake.write_text('''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
names = ('ROBOT_YANDEX_VOICE', 'ROBOT_AUDIO_VOLUME', 'ROBOT_API_ENDPOINT',
         'ROBOT_SUPERVISOR_ADDR', 'ROBOT_AUDIO_CONFIG', 'ROBOT_CAMERA_NETRC', 'ROBOTCTL_PORT')
Path(os.environ['TEST_CAPTURE']).write_text(json.dumps({
    'argv': sys.argv[1:], 'env': {name: os.environ.get(name) for name in names},
    'has_key': bool(os.environ.get('ROBOT_YANDEX_API_KEY'))}))
''')
        fake.chmod(0o700)
        ssh = self.root / "bin" / "fake-ssh"
        ssh.write_text('''#!/usr/bin/env python3
import os, sys
from pathlib import Path
with Path(os.environ['TEST_SSH_LOG']).open('a') as log:
    log.write(' '.join(sys.argv[1:]) + '\\n')
if sys.argv[1] == '-G':
    print('hostname 192.0.2.99')
elif sys.argv[-2:] == ['bash', '-s']:
    script = sys.stdin.read()
    if 'ROBOT_SUPERVISOR_' in script:
        Path(os.environ['TEST_SCRIPT']).write_text(script)
    if 'ROBOT_STATUS_COMMANDS' in script:
        print('enabled')
''')
        ssh.chmod(0o700)
        self.env = {"PATH": os.environ["PATH"], "HOME": str(self.outside),
                    "ROBOT_SSH": str(ssh), "ROBOT_CAMERA_BIN": str(fake),
                    "ROBOT_AUDIO_BIN": str(fake), "ROBOT_SERVER_BIN": str(fake),
                    "ROBOT_SUPERVISOR_BIN": str(fake),
                    "ROBOT_SESSION_LOCK": str(self.root / "session.lock"),
                    "TEST_CAPTURE": str(self.capture), "TEST_SSH_LOG": str(self.ssh_log),
                    "TEST_SCRIPT": str(Path(self.temp.name) / 'supervisor-script')}

    def run_cli(self, *args):
        return subprocess.run([str(self.root / "bin" / "robot"), *map(str, args)],
                              cwd=self.outside, env=self.env, capture_output=True,
                              text=True, timeout=10)

    def child(self, result):
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.capture.is_file(), "configured child was not dispatched")
        self.assertNotIn(FAKE_KEY, result.stdout + result.stderr)
        return json.loads(self.capture.read_text())

    def test_persistent_supervisor_reuse_and_secret_transport(self):
        self.env['ROBOT_SUPERVISOR_TOKEN'] = 'synthetic-supervisor-token'
        self.child(self.run_cli('run', '--no-slam', '--no-yolo'))
        script = Path(self.env['TEST_SCRIPT']).read_text()
        self.assertIn('PING', script)
        self.assertIn('AUTH', script)
        self.assertNotIn('pkill', script)
        self.assertNotIn('synthetic-supervisor-token', self.ssh_log.read_text())

    # SPEC: PONG IDLE alone cannot identify the expected supervisor ELF.
    def test_real_probe_refuses_unidentified_idle_or_active_listener_without_side_effects(self):
        self.env['ROBOT_SUPERVISOR_TOKEN'] = 'synthetic-probe-token'
        self.child(self.run_cli('run', '--no-slam', '--no-yolo'))
        template = Path(self.env['TEST_SCRIPT']).read_text()
        for reply, expected in ((b'PONG IDLE\n', 4), (b'PONG ACTIVE\n', 2), (b'INVALID\n', 4)):
            with self.subTest(reply=reply):
                requests = []
                with socket.socket() as listener:
                    listener.bind(('127.0.0.1', 0))
                    listener.listen(1)
                    listener.settimeout(5)
                    port = listener.getsockname()[1]
                    def daemon():
                        conn, _ = listener.accept()
                        with conn:
                            conn.settimeout(5)
                            stream = conn.makefile('rb')
                            requests.append(stream.readline())
                            conn.sendall(b'AUTHENTICATED\n')
                            requests.append(stream.readline())
                            conn.sendall(reply)
                    worker = threading.Thread(target=daemon, daemon=True)
                    worker.start()
                    script = template.replace('port=9765\n', f'port={port}\n', 1)
                    result = subprocess.run(['bash', '-s'], input=script,
                                            capture_output=True, text=True, timeout=10)
                    worker.join(timeout=5)
                self.assertEqual(requests, [b'AUTH synthetic-probe-token\n', b'PING\n'])
                self.assertEqual(result.returncode, expected, result.stderr)
                self.assertNotIn('synthetic-probe-token', result.stdout + result.stderr)

    def test_real_probe_reports_absence_before_any_start_or_deploy(self):
        self.child(self.run_cli('run', '--no-slam', '--no-yolo'))
        template = Path(self.env['TEST_SCRIPT']).read_text()
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        script = template.replace('port=9765\n', f'port={port}\n', 1)
        result = subprocess.run(['bash', '-s'], input=script,
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 3, result.stderr)

    def test_run_supervisor_cli_override_reaches_all_children(self):
        child = self.child(self.run_cli('run', '--supervisor', '192.0.2.66:9988'))
        self.assertEqual(child['env']['ROBOT_SUPERVISOR_ADDR'], '192.0.2.66:9988')
        script = Path(self.env['TEST_SCRIPT']).read_text()
        self.assertIn('port=9988', script)

    def test_daemon_start_explicitly_deploys_without_controller_handoff(self):
        result = self.run_cli('daemon', 'start')
        self.assertEqual(result.returncode, 0, result.stderr)
        script = Path(self.env['TEST_SCRIPT']).read_text()
        self.assertIn('ROBOT_SUPERVISOR_START', script)
        self.assertNotIn('ROBOT_DISABLE_CONTROLLER', self.ssh_log.read_text())
        self.assertFalse(self.capture.exists())

    def test_direct_battery_command_uses_shared_player_host_and_port(self):
        self.env['ROBOT_ROBOTCTL_BIN'] = str(self.root / 'bin/fake-child')
        child = self.child(self.run_cli('battery'))
        self.assertEqual(child['argv'], ['192.0.2.23', 'battery'])
        self.assertEqual(child['env']['ROBOTCTL_PORT'], '6666')
        self.assertFalse(self.ssh_log.exists())

    def test_serve_resolves_ssh_alias_locally_when_no_tcp_host_is_configured(self):
        self.config.write_text(TOML.replace('player_host = "192.0.2.23"','player_host = ""').replace('host = "192.0.2.24"','host = ""'))
        child = self.child(self.run_cli('serve'))
        self.assertEqual(child['env']['ROBOT_SUPERVISOR_ADDR'], '192.0.2.99:9765')
        self.assertEqual(self.ssh_log.read_text().splitlines(), ['-G vacuum-test'])

    def test_serve_launches_audio_only_without_ssh(self):
        child = self.child(self.run_cli('serve'))
        self.assertIn('--audio-only', child['argv'])
        self.assertFalse(self.ssh_log.exists())

    def test_default_configuration_reaches_snapshot_from_unrelated_cwd(self):
        child = self.child(self.run_cli("snapshot", "frame.jpg"))
        self.assertEqual(child["argv"], ["frame.jpg"])
        self.assertEqual(child["env"]["ROBOT_CAMERA_NETRC"], str(self.config.parent / "camera.netrc"))
        self.assertFalse(self.ssh_log.exists())

    def test_environment_can_select_private_config_without_changing_cwd(self):
        alternate = private_file(self.outside / 'env.toml', TOML.replace('voice = "alena"', 'voice = "filipp"'))
        self.env['ROBOT_CONFIG'] = str(alternate)
        child = self.child(self.run_cli('audio', 'status'))
        self.assertEqual(child['env']['ROBOT_YANDEX_VOICE'], 'filipp')

    def test_global_alternate_config_is_consumed_not_forwarded(self):
        alternate = private_file(self.outside / "alternate.toml", TOML.replace('voice = "alena"', 'voice = "filipp"'))
        child = self.child(self.run_cli("--config", alternate, "audio", "status"))
        self.assertEqual(child["argv"], ["status"])
        self.assertEqual(child["env"]["ROBOT_YANDEX_VOICE"], "filipp")
        self.assertTrue(child["has_key"])

    def test_legacy_audio_json_config_and_flags_are_preserved(self):
        child = self.child(self.run_cli("--config", self.config, "audio", "run",
                                        "--config", "legacy-audio.json", "--port", "12345"))
        self.assertEqual(child["argv"], ["run", "--config", "legacy-audio.json", "--port", "12345"])
        self.assertEqual(child["env"]["ROBOT_AUDIO_CONFIG"], str(self.config.parent / "audio.json"))

    def test_record_alias_receives_shared_settings_without_hardware(self):
        child = self.child(self.run_cli("record", "recording.wav", "--gain", "80"))
        self.assertEqual(child["argv"], ["capture", "recording.wav", "--gain", "80"])
        self.assertEqual(child["env"]["ROBOT_AUDIO_VOLUME"], "0.45")
        self.assertFalse(self.ssh_log.exists())

    def test_run_cli_overrides_config_and_env_exports_reach_server(self):
        self.env["ROBOT_YANDEX_VOICE"] = "filipp"
        child = self.child(self.run_cli("run", "--config", self.config,
                                        "--port", "10085", "--no-slam", "--no-yolo"))
        args = child["argv"]
        self.assertEqual(args[args.index("--port") + 1], "10085")
        self.assertEqual(args[args.index("--listen") + 1], "127.0.0.2")
        self.assertEqual(args[args.index("--player-host") + 1], "192.0.2.23")
        self.assertEqual(args[args.index("--player-port") + 1], "6666")
        self.assertNotIn("--slam-adapter", args)
        self.assertNotIn("--yolo-adapter", args)
        self.assertEqual(child["env"]["ROBOT_YANDEX_VOICE"], "filipp")
        self.assertEqual(child["env"]["ROBOT_SUPERVISOR_ADDR"], "192.0.2.24:9765")
        self.assertIn("vacuum-test", self.ssh_log.read_text())
        script = Path(self.env['TEST_SCRIPT']).read_text()
        self.assertIn('port=9765', script)
        self.assertIn('timeout_ms=8000', script)
        self.assertIn('--heartbeat-port "$port" --timeout-ms "$timeout_ms"', script)

    def test_run_uses_config_values_when_not_overridden(self):
        child = self.child(self.run_cli("--config", self.config, "run"))
        self.assertEqual(child["argv"][child["argv"].index("--port") + 1], "9085")
        self.assertNotIn("--slam-adapter", child["argv"])
        self.assertNotIn("--yolo-adapter", child["argv"])

    def test_bad_private_config_is_rejected_before_any_child_or_ssh(self):
        for text, mode in ((TOML, 0o644), ('[server]\nport = 0\n', 0o600),
                           ('[yandex]\napi_key = "' + FAKE_KEY, 0o600)):
            with self.subTest(mode=oct(mode), malformed=text != TOML):
                self.config.write_text(text)
                self.config.chmod(mode)
                result = self.run_cli("--config", self.config, "run")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("config", result.stderr.lower())
                self.assertNotIn(FAKE_KEY, result.stdout + result.stderr)
                self.assertFalse(self.capture.exists())
                self.assertFalse(self.ssh_log.exists())

    def test_explicit_missing_file_is_not_silently_ignored(self):
        result = self.run_cli("--config", self.outside / "missing.toml", "snapshot")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("config", result.stderr.lower())
        self.assertFalse(self.capture.exists())
        self.assertFalse(self.ssh_log.exists())


if __name__ == "__main__":
    unittest.main()
