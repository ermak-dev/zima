#!/usr/bin/env python3
"""FEATURE: Final ROS transcript reaches only the optional read-only responder."""
import importlib.util
from pathlib import Path
import sys,threading,time,unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2'))
from test_audio_ros import FakeRuntime,wait_for


@unittest.skipUnless(importlib.util.find_spec('rclpy'),'source ROS and audio interfaces')
class VoiceControllerRosTests(unittest.TestCase):
 def test_final_battery_command_speaks_once_and_motion_is_ignored(self):
  import rclpy
  from rclpy.context import Context
  from rclpy.executors import MultiThreadedExecutor
  from robot_audio.ros_node import AudioNode
  from robot_audio.commands import BatteryProvider,BatterySnapshot
  context=Context();rclpy.init(context=context,domain_id=212)
  runtime=FakeRuntime()
  node=AudioNode(runtime=runtime,config={'controller':{'enabled':True}},context=context,
                 battery_reader=BatteryProvider(BatterySnapshot(percent=42,freshness='fresh')))
  executor=MultiThreadedExecutor(num_threads=4,context=context);executor.add_node(node)
  thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
  try:
   value={'session_id':'session-1','stream_id':'offline','text':'Какой у тебя заряд?',
          'is_final':True,'status':'final'}
   runtime.callbacks['transcript'](value)
   self.assertTrue(wait_for(lambda:runtime.calls))
   self.assertEqual(runtime.calls[0][0],'say')
   self.assertEqual(runtime.calls[0][1]['text'],'У меня сорок два процента заряда.')
   runtime.callbacks['transcript'](value)
   runtime.callbacks['transcript']({**value,'session_id':'motion','text':'Езжай вперёд'})
   time.sleep(.1);self.assertEqual(len(runtime.calls),1)
  finally:
   node.close();executor.shutdown(timeout_sec=4);thread.join(4);node.destroy_node();context.try_shutdown()


if __name__=='__main__':unittest.main()
