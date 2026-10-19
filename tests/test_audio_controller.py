#!/usr/bin/env python3
"""FEATURE: One read-only voice command yields a bounded, cancellable reply.

SPEC: No movement, no shell text, no duplicate replies or invented battery age.
"""
from pathlib import Path
import sys,tempfile,threading,time,unittest
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2'))
from robot_audio.commands import BatterySnapshot


def wait_for(predicate,seconds=2):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  if predicate():return True
  time.sleep(.01)
 return bool(predicate())


class ControllerTests(unittest.TestCase):
 def setUp(self):
  try:
   from robot_audio.controller import VoiceController,RobotBatteryReader
  except ImportError:self.fail('read-only voice controller is not implemented')
  self.Controller,self.Reader=VoiceController,RobotBatteryReader

 def test_readonly_reply_is_deduplicated_and_acknowledged(self):
  spoken=[];finished=[];reads=[]
  def read():reads.append(1);return BatterySnapshot(percent=42,freshness='fresh')
  c=self.Controller(read,lambda text,sid,cancel:spoken.append((text,sid)),finished.append)
  c.start();self.addCleanup(c.close)
  event={'session_id':'session-1','text':'Какой у тебя заряд?','is_final':True,'status':'final'}
  self.assertTrue(c.submit(event));self.assertFalse(c.submit(event))
  self.assertTrue(wait_for(lambda:finished))
  self.assertEqual(spoken,[('У меня сорок два процента заряда.','session-1')])
  self.assertEqual(reads,[1]);self.assertEqual(finished,['session-1'])

 def test_unknown_or_failed_transcripts_never_query_or_speak(self):
  calls=[]
  c=self.Controller(lambda:calls.append('read'),lambda *args:calls.append('speak'),lambda sid:calls.append('finish'))
  c.start();self.addCleanup(c.close)
  for event in ({'session_id':'a','text':'езжай вперёд','is_final':True,'status':'final'},
                {'session_id':'b','text':'какой заряд','is_final':False,'status':'partial'},
                {'session_id':'c','text':'какой заряд','error':'asr_failed','status':'failed'}):
   self.assertFalse(c.submit(event))
  self.assertEqual(calls,['finish'])

 # BUG: Valid long final transcripts leave the acknowledged speech session processing.
 # HYPOTHESIS: The 512-character intent limit is incorrectly used as an event limit.
 # SOLUTION: Validate events to 8192 characters; retain the separate 512-character intent limit.
 # RESULT: FIXED — unsupported long finals finish once without reading or speaking.
 # STATUS: FIXED
 def test_long_final_transcript_finishes_once_without_reading_or_speaking(self):
  for length in (513,8192):
   with self.subTest(length=length):
    calls=[];finished=[]
    c=self.Controller(lambda:calls.append('read'),lambda *args:calls.append('speak'),finished.append)
    c.start();self.addCleanup(c.close)
    event={'session_id':'long-final','text':'какой заряд'+' '* (length-11),'is_final':True,'status':'final'}
    self.assertEqual(len(event['text']),length)
    self.assertFalse(c.submit(event))
    self.assertEqual(finished,['long-final'])
    self.assertEqual(c.last_error,'unsupported_intent')
    self.assertFalse(c.submit(event))
    self.assertFalse(c.submit(dict(event,text='какой заряд')))
    c.close()
    self.assertEqual(finished,['long-final']);self.assertEqual(calls,[])

 def test_oversized_or_malformed_final_transcripts_are_not_acknowledged(self):
  calls=[]
  c=self.Controller(lambda:calls.append('read'),lambda *args:calls.append('speak'),lambda sid:calls.append('finish'))
  c.start();self.addCleanup(c.close)
  for changes in ({'text':'x'*8193},{'text':None},{'session_id':'unsafe/session'},
                  {'is_final':False},{'status':'partial'},{'error':'asr_failed'}):
   with self.subTest(changes=tuple(changes)):
    event={'session_id':'invalid','text':'x'*513,'is_final':True,'status':'final'}
    self.assertFalse(c.submit(dict(event,**changes)))
  c.close();self.assertEqual(calls,[])

 def test_reported_unknown_age_is_qualified_only_when_explicit(self):
  spoken=[];finished=[]
  c=self.Controller(lambda:BatterySnapshot(percent=100),lambda text,*args:spoken.append(text),finished.append,allow_reported=True)
  c.start();self.addCleanup(c.close)
  c.submit({'session_id':'a','text':'какой заряд','is_final':True,'status':'final'})
  self.assertTrue(wait_for(lambda:spoken))
  self.assertIn('сто процентов',spoken[0]);self.assertIn('Давность',spoken[0])

 def test_close_cancels_active_reply(self):
  started=threading.Event();cancelled=threading.Event();finished=[]
  def speak(text,sid,cancel):
   started.set()
   if cancel.wait(2):cancelled.set()
  c=self.Controller(lambda:BatterySnapshot(),speak,finished.append)
  c.start();c.submit({'session_id':'a','text':'какой заряд','is_final':True,'status':'final'})
  self.assertTrue(started.wait(1));c.close()
  self.assertTrue(cancelled.is_set());self.assertEqual(finished,['a'])
  self.assertFalse(c.submit({'session_id':'b','text':'какой заряд','status':'final'}))

 def test_reader_is_fixed_bounded_and_does_not_upgrade_freshness(self):
  calls=[]
  def run(args,**kwargs):
   calls.append((args,kwargs));return SimpleNamespace(returncode=0,stdout='{"valid":25,"percent":88,"voltage":20,"charging":true,"freshness":"unknown"}')
  value=self.Reader(runner=run)()
  self.assertEqual(value.percent,88);self.assertEqual(value.freshness,'unknown')
  self.assertEqual(calls[0][0][-1],'battery')
  self.assertNotIn('shell',calls[0][1]);self.assertLessEqual(calls[0][1]['timeout'],6)


if __name__=='__main__':unittest.main()
