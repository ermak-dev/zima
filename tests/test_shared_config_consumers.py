#!/usr/bin/env python3
# FEATURE: Camera and legacy audio operations consume the common configuration handoff.
# SPEC: No live devices; environment defaults are applied and explicit audio profiles win.
# IMPLEMENTATION: shared environment defaults connect TOML settings to existing consumers.
# RESULT: IMPLEMENTED — camera, audio profiles and alternate provider credentials verified.
# STATUS: DONE
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from robot_audio.config import load_config

class ConsumerTests(unittest.TestCase):
    def test_openrouter_reads_its_shared_provider_key_without_a_second_secret_file(self):
        from robot_audio.tts import OpenRouterTTS
        with patch.dict(os.environ, {'ROBOT_PROVIDER_OPENROUTER_API_KEY':'synthetic-provider-key'}, clear=True):
            client = OpenRouterTTS(token_file='/not-used/shared-key-replaces-this-file')
            self.assertEqual(client._token(), 'synthetic-provider-key')
            explicit = OpenRouterTTS(api_key='explicit-key', token_file='/not-used')
            self.assertEqual(explicit._token(), 'explicit-key')

    def test_audio_uses_shared_camera_and_output_parameters(self):
        with patch.dict(os.environ, {'ROBOT_CAMERA_AUDIO_URL':'http://configured-cam/audio.opus',
                'ROBOT_CAMERA_NETRC':'/private/test.netrc','ROBOT_AUDIO_VOLUME':'0.45',
                'ROBOT_AUDIO_MAX_VOLUME':'0.8','ROBOT_SSH_HOST':'configured-robot'}, clear=True):
            config = load_config()
        self.assertEqual(config['input']['url'],'http://configured-cam/audio.opus')
        self.assertEqual(config['input']['netrc'],'/private/test.netrc')
        self.assertEqual(config['output']['host'],'configured-robot')
        self.assertEqual(config['output']['volume'],.45)
        self.assertEqual(config['output']['max_volume'],.8)
        self.assertFalse(config['input']['enabled'])
        self.assertFalse(config['output']['enabled'])

    def test_shared_profile_is_used_but_explicit_profile_wins(self):
        with tempfile.TemporaryDirectory() as tmp:
            a,b=Path(tmp)/'a.json',Path(tmp)/'b.json'
            a.write_text(json.dumps({'input':{'url':'http://a/audio.opus'}}))
            b.write_text(json.dumps({'input':{'url':'http://b/audio.opus'}}))
            with patch.dict(os.environ,{'ROBOT_AUDIO_CONFIG':str(a)},clear=True):
                self.assertEqual(load_config()['input']['url'],'http://a/audio.opus')
                self.assertEqual(load_config(b)['input']['url'],'http://b/audio.opus')

    def test_snapshot_consumes_configured_url_without_a_proxy(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            netrc=folder/'netrc';netrc.write_text('machine configured-cam login fake password fake\n');netrc.chmod(0o600)
            log=folder/'args'
            fake=folder/'curl'
            fake.write_text('#!/usr/bin/env python3\nimport sys\nfrom pathlib import Path\n'
                            f'Path({str(log)!r}).write_text("\\n".join(sys.argv[1:]))\n'
                            'Path(sys.argv[sys.argv.index("--output")+1]).write_bytes(b"\\xff\\xd8"+b"x"*1024+b"\\xff\\xd9")\n')
            fake.chmod(0o700)
            result=subprocess.run([str(ROOT/'bin/camera/snapshot'),str(folder/'frame.jpg')],
                env={**os.environ,'ROBOT_CAMERA_NETRC':str(netrc),'ROBOT_CURL':str(fake),
                     'ROBOT_CAMERA_SNAPSHOT_URL':'http://configured-cam/image.jpg'},capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            args=log.read_text().splitlines()
            self.assertIn('http://configured-cam/image.jpg',args)
            self.assertEqual(args[args.index('--noproxy')+1],'*')

if __name__=='__main__': unittest.main()
