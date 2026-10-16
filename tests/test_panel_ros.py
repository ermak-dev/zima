#!/usr/bin/env python3
"""Offline integration: isolated DDS domain, synthetic sensors/TF, real HTTP panel.

Run after sourcing ROS Jazzy. No connection to Player or the robot is made.
"""
import importlib.util
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import unittest
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'source ROS Jazzy to run DDS integration')
class PanelRos(unittest.TestCase):
    def test_latched_inventory_and_scan_time_tf(self):
        import rclpy
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import BatteryState, LaserScan, Range
        from std_msgs.msg import Bool, Float32MultiArray
        from tf2_msgs.msg import TFMessage

        # Both processes explicitly use an isolated domain, never the live bridge.
        domain = 219
        rclpy.init(domain_id=domain)
        node = rclpy.create_node('panel_offline_fixture')
        latched = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publishers = []
        def publish(topic, typ, message, qos=latched):
            pub = node.create_publisher(typ, topic, qos)
            publishers.append(pub)
            pub.publish(message)
            return pub

        battery = BatteryState(percentage=.87, voltage=20.0, power_supply_status=1)
        publish('/battery_state', BatteryState, battery)
        publish('/bumper', Bool, Bool(data=False))
        publish('/wall_ir_0', Range, Range(range=.055))
        publish('/wall_ir/voltages', Float32MultiArray, Float32MultiArray(data=[0.0]))
        for name, count in [('cliff', 4), ('drop', 2)]:
            publish(f'/{name}_ir/voltages', Float32MultiArray,
                    Float32MultiArray(data=[float(i) for i in range(count)]))
            for i in range(count):
                publish(f'/{name}_ir_{i}', Range, Range(range=float(i)/10))
        publish('/sonar', Range, Range(range=65.534))
        mount = TransformStamped()
        mount.header.frame_id = 'base_link'
        mount.child_frame_id = 'laser'
        mount.transform.translation.x = -.1
        mount.transform.rotation.w = 1.0
        publish('/tf_static', TFMessage, TFMessage(transforms=[mount]))
        tf_pub = node.create_publisher(TFMessage, '/tf', 10)
        odom_pub = node.create_publisher(Odometry, '/odom', 10)
        scan_pub = node.create_publisher(LaserScan, '/scan', 10)
        commands = []
        node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
        script = os.environ.get('ROBOT_PANEL_TEST_SCRIPT', str(ROOT / 'ros2/web_panel.py'))
        proc = subprocess.Popen([sys.executable, script, '--ros-args', '-p', f'port:={port}'],
                                env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        url = f'http://127.0.0.1:{port}'
        def get(path):
            with urllib.request.urlopen(url + path, timeout=.5) as resp:
                return json.load(resp)
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                self.assertIsNone(proc.poll(), 'panel exited before serving')
                stamp = node.get_clock().now().to_msg()
                tf = TransformStamped()
                tf.header.frame_id = 'odom'
                tf.header.stamp = stamp
                tf.child_frame_id = 'base_link'
                tf.transform.translation.x = 1.0
                tf.transform.translation.y = 2.0
                tf.transform.rotation.z = math.sqrt(.5)
                tf.transform.rotation.w = math.sqrt(.5)
                tf_pub.publish(TFMessage(transforms=[tf]))
                odom = Odometry()
                odom.header.frame_id = 'odom'
                odom.header.stamp = stamp
                odom.pose.pose.position.x = 1.0
                odom.pose.pose.position.y = 2.0
                odom.pose.pose.orientation = tf.transform.rotation
                odom_pub.publish(odom)
                scan = LaserScan(angle_min=0.0, angle_increment=1.0,
                                 range_max=6.0, ranges=[1.0])
                scan.header.frame_id = 'laser'
                scan.header.stamp = stamp
                scan_pub.publish(scan)
                rclpy.spin_once(node, timeout_sec=.05)
                try:
                    state, cloud = get('/state.json'), get('/scan.json')
                    if (cloud.get('points') and state.get('sensors', {}).get('drop_ranges')
                            and state['sensors'].get('battery')):
                        break
                except (OSError, urllib.error.URLError):
                    pass
            else:
                self.fail('latched sensor inventory and transformed scan never arrived')
            self.assertEqual(len(state['sensors']['cliff_ranges']), 4)
            for actual, expected in zip(state['sensors']['cliff_ranges'], [0.0, .1, .2, .3]):
                self.assertAlmostEqual(actual, expected, places=6)
            self.assertEqual(state['sensors']['cliff_ir'], [0.0, 1.0, 2.0, 3.0])
            self.assertEqual(state['sensors']['bumper'], 'clear')
            self.assertEqual(state['sensors']['battery']['percent'], 87)
            self.assertEqual(cloud['frame'], 'odom')
            self.assertAlmostEqual(cloud['points'][0][0], 1.0, places=5)
            self.assertAlmostEqual(cloud['points'][0][1], 2.9, places=5)
            self.assertEqual(commands, [], 'passive HTTP polling must not publish motion')
            request = urllib.request.Request(url + '/cmd_vel', data=b'{"v":1,"w":2}',
                                             headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(request) as response:
                self.assertEqual(json.load(response), {'v': .1, 'w': .4})
            deadline = time.monotonic() + 2
            while not commands and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            self.assertTrue(commands)
            self.assertAlmostEqual(commands[-1].linear.x, .1)
            self.assertAlmostEqual(commands[-1].angular.z, .4)
        finally:
            proc.terminate()
            output = proc.communicate(timeout=5)[0].decode()
            node.destroy_node()
            rclpy.shutdown()
            if proc.returncode not in (0, -15):
                print(output)


if __name__ == '__main__':
    unittest.main()
