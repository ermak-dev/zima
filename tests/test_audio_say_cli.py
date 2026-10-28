#!/usr/bin/env python3
# FEATURE: robot say dispatches to the same API as the web panel without ROS or SSH.
# SPEC: CLI is a thin client; credentials and synthesis belong to the host server.
# IMPLEMENTATION: shared HTTP client replaces the former direct SSH/Piper command;
# synthesis adapters retain independent coverage in test_audio_say.py.
import subprocess
from pathlib import Path
import unittest
import test_say_api_cli

ROOT = Path(__file__).resolve().parents[1]

class SayCliTests(test_say_api_cli.SayApiTests):
    def run_say(self, *args):
        return subprocess.run([str(ROOT/'bin/robot'),'say',*args],env=self.env,
                              capture_output=True,text=True,timeout=10)

    def test_help_needs_no_ros_or_ssh(self):
        result=self.run_say('--help')
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('text',result.stdout)
        self.assertEqual(self.calls,[])

    def test_text_file_uses_the_same_api(self):
        path=Path(self.tmp.name)/'text.txt'
        path.write_text('Привет из файла',encoding='utf-8')
        result=self.run_say('--text-file',str(path))
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(self.calls[0][1],{'text':'Привет из файла'})

    def test_missing_or_oversized_text_file_is_actionable_and_never_submits(self):
        path=Path(self.tmp.name)/'text.txt'
        result=self.run_say('--text-file',str(path))
        self.assertNotEqual(result.returncode,0)
        self.assertNotIn('Traceback',result.stderr)
        path.write_text('x'*16001)
        result=self.run_say('--text-file',str(path))
        self.assertNotEqual(result.returncode,0)
        self.assertIn('too large',result.stderr)
        self.assertEqual(self.calls,[])

    def test_legacy_credential_and_piper_flags_do_not_bypass_server(self):
        for flag in ('--yandex-env','--piper-module-path'):
            with self.subTest(flag=flag):
                result=self.run_say('Привет',flag,'/not-used')
                self.assertNotEqual(result.returncode,0)
                self.assertNotIn('Traceback',result.stderr)
        self.assertEqual(self.calls,[])

if __name__=='__main__': unittest.main(verbosity=2)
