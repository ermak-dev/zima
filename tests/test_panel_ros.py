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
    def test_latched_map_and_map_frame_pose(self):
        self.run_panel_fixture(mapping=True)

    def test_latched_inventory_and_scan_time_tf(self):
        self.run_panel_fixture(mapping=False)

    def run_panel_fixture(self, mapping):
        import rclpy
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
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
        if mapping:
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.width, grid.info.height = 2, 2
            grid.info.resolution = .05
            grid.info.origin.position.x = -1.
            grid.info.origin.orientation.w = 1.
            grid.data = [-1, 0, 100, 50]
            publish('/map', OccupancyGrid, grid)
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
        from websocket import create_connection
        ws = None
        deadline = time.monotonic() + 10
        while ws is None:
            try:
                ws = create_connection(f'ws://127.0.0.1:{port}/ws', timeout=.5)
            except (ConnectionRefusedError, OSError):
                if time.monotonic() > deadline or proc.poll() is not None:
                    raise
                time.sleep(.2)
        ws.settimeout(.5)
        frames = []
        def read_frame():
            frames.append(json.loads(ws.recv()))
            return frames[-1]
        def latest(kind):
            for frame in reversed(frames):
                if frame['type'] == kind:
                    return frame['data']
            return None
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
                correction = TransformStamped()
                correction.header.frame_id = 'map'
                correction.child_frame_id = 'odom'
                correction.header.stamp = stamp
                correction.transform.translation.x = 10.
                correction.transform.rotation.w = 1.
                tf_pub.publish(TFMessage(transforms=[correction, tf] if mapping else [tf]))
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
                    read_frame()
                except Exception:
                    pass
                state, cloud = latest('state'), latest('scan')
                if (cloud and cloud.get('points') and state and state.get('sensors', {}).get('drop_ranges')
                        and state['sensors'].get('battery')
                        and (not mapping or (state.get('map_pose') and cloud.get('frame') == 'map'))):
                    break
            else:
                self.fail('latched sensor inventory and transformed scan never arrived')
            self.assertEqual(len(state['sensors']['cliff_ranges']), 4)
            for actual, expected in zip(state['sensors']['cliff_ranges'], [0.0, .1, .2, .3]):
                self.assertAlmostEqual(actual, expected, places=6)
            self.assertEqual(state['sensors']['cliff_ir'], [0.0, 1.0, 2.0, 3.0])
            self.assertEqual(state['sensors']['bumper'], 'clear')
            self.assertEqual(state['sensors']['battery']['percent'], 87)
            if mapping:
                # Wait for the map push; no-map fixture asserts its absence.
                deadline = time.monotonic() + 5
                while latest('map') is None and time.monotonic() < deadline:
                    try:
                        read_frame()
                    except Exception:
                        pass
                grid = latest('map')
                self.assertEqual(grid['width'], 2)
                self.assertEqual(grid['origin']['x'], -1.)
                self.assertTrue(grid['image'].startswith('data:image/png;base64,'))
                self.assertLess(grid['version'], 2**53, 'version must survive JavaScript JSON numbers exactly')
                frames_before = len(frames)
                deadline = time.monotonic() + 1.5
                while time.monotonic() < deadline:
                    try:
                        read_frame()
                    except Exception:
                        pass
                for frame in frames[frames_before:]:
                    self.assertNotEqual(frame['type'], 'map',
                                        'unchanged map must not be resent')
                self.assertAlmostEqual(state['map_pose']['x'], 11.)
                self.assertAlmostEqual(state['map_pose']['y'], 2.)
                self.assertAlmostEqual(state['map_pose']['yaw'], math.pi / 2)
                self.assertEqual(state['map_pose']['frame'], 'map')
                self.assertIsNotNone(state['map_age_s'])
            else:
                self.assertIsNone(latest('map'), 'map must not arrive without SLAM')
                self.assertIsNone(state.get('map_pose'))
            self.assertEqual(cloud['frame'], 'map' if mapping else 'odom')
            self.assertAlmostEqual(cloud['points'][0][0], 11.0 if mapping else 1.0, places=5)
            self.assertAlmostEqual(cloud['points'][0][1], 2.9, places=5)
            self.assertEqual(commands, [], 'passive viewing must not publish motion')
            ws.send(json.dumps({'type': 'cmd', 'v': 1, 'w': 2}))
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
