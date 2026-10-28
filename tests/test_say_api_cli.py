#!/usr/bin/env python3
# FEATURE: robot say is a client of the panel's TTS API, not a second TTS pipeline.
# SPEC: no keys or SSH in the client; one submit, bounded polling, completion required.
# IMPLEMENTATION: bounded HTTP submission and polling replace direct synthesis/playback.
# RESULT: IMPLEMENTED — common API completion, errors, defaults and validation verified.
# STATUS: DONE
import http.server
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1]

class SayApiTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.calls = []
        self.state = 'completed'
        owner = self
        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
                owner.calls.append((self.path, json.loads(body)))
                self.reply(202, {'id':'test-job','state':'queued'})
            def do_GET(self):
                owner.calls.append((self.path, None))
                self.reply(200, {'id':'test-job','state':owner.state,'error':'test_failure' if owner.state != 'completed' else None})
            def reply(self, status, value):
                body = json.dumps(value).encode()
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            def log_message(self, *args): pass
        self.server = http.server.ThreadingHTTPServer(('127.0.0.1',0), Handler)
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
        self.env = {**os.environ, 'ROBOT_API_ENDPOINT':f'http://127.0.0.1:{self.server.server_port}',
                    'ROBOT_SSH':'/nonexistent/ssh', 'HOME':self.tmp.name}

    def run_say(self, *args):
        return subprocess.run(['python3',str(ROOT/'bin/audio/say.py'),*args],env=self.env,
                              capture_output=True,text=True,timeout=10)

    def test_uses_shared_api_and_waits_for_completion_without_ssh_or_keys(self):
        result = self.run_say('Привет', '--volume', '.4')
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(self.calls[0],('/api/audio/say',{'text':'Привет','volume':.4}))
        self.assertEqual(self.calls[1],('/api/audio/tasks/test-job',None))
        self.assertIn('completed',result.stdout)

    def test_failure_or_uncertainty_is_not_success_and_never_reposts(self):
        for state in ('failed','unknown','cancelled'):
            with self.subTest(state=state):
                self.state = state
                self.calls.clear()
                result = self.run_say('Привет')
                self.assertNotEqual(result.returncode,0)
                self.assertNotIn('Traceback',result.stderr)
                self.assertEqual(sum(body is not None for _,body in self.calls),1)
                self.assertIn(state,result.stderr)

    def test_default_volume_is_resolved_by_server_not_client(self):
        result = self.run_say('Привет')
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(self.calls[0][1],{'text':'Привет'})

    def test_invalid_text_or_volume_never_reaches_api(self):
        for args in [(' ',),('x'*2001,),('hello','--volume','nan'),('hello','--volume','1.1')]:
            with self.subTest(args=args):
                result = self.run_say(*args)
                self.assertNotEqual(result.returncode,0)
                self.assertNotIn('Traceback',result.stderr)
        self.assertEqual(self.calls,[])

if __name__ == '__main__': unittest.main()
