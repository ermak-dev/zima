#!/usr/bin/env python3
"""FEATURE: Only explicit companion mode owns camera power and restores after child exit."""
import contextlib,importlib.util,io,json,signal,subprocess
from pathlib import Path
import sys,tempfile,unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))


class CompanionCliTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  self.config=Path(self.tmp.name)/'audio.json';self.config.write_text(json.dumps({'input':{'enabled':True}}))
  spec=importlib.util.spec_from_file_location('companion_cli_test',ROOT/'bin/audio/cli.py')
  self.cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.cli)

 def test_normal_run_is_separate_and_companion_owns_the_session(self):
  events=[];calls=[]
  class Process:
   pid=123
   def poll(self):return 0
   def wait(self,timeout):return 0
  class Session:
   def __init__(self,**kwargs):calls.append(kwargs)
   def __enter__(self):events.append('enter');return self
   def __exit__(self,*args):events.append('restore')
   def check(self):pass
  def popen(args,**kwargs):events.append('child');calls.append(args);return Process()
  with patch('robot_audio.camera_session.CameraSSH'),patch('robot_audio.camera_session.CameraGainLease'),patch('robot_audio.camera_session.CameraSession',Session),patch('subprocess.Popen',popen):
   code=self.cli.main(['companion','--config',str(self.config),'--max-seconds','60'])
  self.assertEqual(code,0);self.assertEqual(events,['enter','child','restore'])
  self.assertTrue(calls[0]['explicit']);self.assertNotIn('--experimental-wake',calls[1])

 def test_session_failure_stops_child_before_restoring_camera(self):
  from robot_audio.camera_session import CameraSessionError
  events=[]
  class Process:
   pid=123;code=None
   def poll(self):return self.code
   def send_signal(self,sig):events.append('stop');self.code=130
   def wait(self,timeout):return self.code
   def terminate(self):self.code=-15
   def kill(self):self.code=-9
  process=Process()
  class Session:
   def __init__(self,**kwargs):pass
   def __enter__(self):events.append('enter');return self
   def __exit__(self,*args):events.append('restore');return False
   def check(self):raise CameraSessionError('expired')
  with patch('robot_audio.camera_session.CameraSSH'),patch('robot_audio.camera_session.CameraGainLease'),patch('robot_audio.camera_session.CameraSession',Session),patch('subprocess.Popen',return_value=process),contextlib.redirect_stderr(io.StringIO()):
   code=self.cli.main(['companion','--config',str(self.config)])
  self.assertNotEqual(code,0);self.assertEqual(events,['enter','stop','restore'])

 # BUG: Repeated termination signals interrupt child shutdown before camera restoration.
 # HYPOTHESIS: The raising session handler remains installed during child waits.
 # SOLUTION: Ignore termination signals through bounded shutdown, restoring handlers in finally.
 # RESULT: FIXED — repeated SIGINT/SIGTERM cannot skip terminate/kill or confirmed exit.
 # STATUS: FIXED
 def test_repeated_signals_do_not_restore_camera_before_child_stops(self):
  from robot_audio.camera_session import CameraSessionError
  for repeated in (signal.SIGINT,signal.SIGTERM):
   with self.subTest(signal=repeated):
    events=[];previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    def interrupt(sig,frame):raise CameraSessionError('session_interrupted')
    class Process:
     pid=123;code=None
     def poll(self):return self.code
     def send_signal(self,sig):events.append(('signal',sig))
     def terminate(self):events.append('terminate')
     def kill(self):events.append('kill');self.code=-9
     def wait(self,timeout):
      events.append(('wait',timeout));signal.raise_signal(repeated)
      if self.code is None:raise subprocess.TimeoutExpired('fake-child',timeout)
      events.append('stopped');return self.code
    process=Process()
    class Session:
     def __init__(self,**kwargs):pass
     def __enter__(self):
      for sig in previous:signal.signal(sig,interrupt)
      return self
     def check(self):signal.raise_signal(signal.SIGINT)
     def __exit__(self,*args):
      events.append(('restore',process.poll()))
      events.append(('handlers_restored',all(signal.getsignal(sig) is interrupt for sig in previous)))
      for sig,handler in previous.items():signal.signal(sig,handler)
      return False
    try:
     with patch('robot_audio.camera_session.CameraSSH'),patch('robot_audio.camera_session.CameraGainLease'),patch('robot_audio.camera_session.CameraSession',Session),patch('subprocess.Popen',return_value=process),contextlib.redirect_stderr(io.StringIO()):
      code=self.cli.main(['companion','--config',str(self.config)])
     self.assertNotEqual(code,0)
     self.assertEqual(events,[('signal',signal.SIGINT),('wait',10),'terminate',('wait',3),'kill',('wait',2),'stopped',('restore',-9),('handlers_restored',True)])
     self.assertEqual({sig:signal.getsignal(sig) for sig in previous},previous)
    finally:
     for sig,handler in previous.items():signal.signal(sig,handler)

 def test_shutdown_restores_handlers_even_when_final_wait_times_out(self):
  previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
  waits=[]
  class Process:
   def poll(self):return None
   def send_signal(self,sig):pass
   def terminate(self):pass
   def kill(self):pass
   def wait(self,timeout):
    waits.append(timeout)
    for sig in previous:
     self_handler=signal.getsignal(sig)
     if self_handler != signal.SIG_IGN:raise AssertionError('shutdown signals must be ignored')
    raise subprocess.TimeoutExpired('fake-child',timeout)
  with self.assertRaises(subprocess.TimeoutExpired):self.cli._stop_child(Process())
  self.assertEqual(waits,[10,3,2])
  self.assertEqual({sig:signal.getsignal(sig) for sig in previous},previous)


if __name__=='__main__':unittest.main()
