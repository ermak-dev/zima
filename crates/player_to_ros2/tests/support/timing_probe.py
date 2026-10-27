# BUG: Player measurement times are lost before scan/odom publication.
# HYPOTHESIS: ROS processing timestamps turn transport delays into pose errors.
# SOLUTION: retain shared measurement time and reject nonuniform scan geometry.
# RESULT: FIXED — delayed odometry no longer moves the synthetic wall.
# STATUS: FIXED
import math
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer

HEADER = struct.Struct('!6Id2I')


def packet(interface, subtype, timestamp, payload, msg_type=1):
    return HEADER.pack(0, 0, interface, 0, msg_type, subtype, timestamp, 0, len(payload)) + payload


def odom(timestamp, yaw):
    return packet(4, 1, timestamp, struct.pack('!6dI', 0, 0, yaw, 0, 0, 0.4, 0))


def scan(timestamp, angles=None, ranges=None):
    if angles is None:
        angles = [-math.pi + i * math.tau / 360 for i in range(360)]
    if ranges is None:
        ranges = [2.0] * len(angles)
    payload = struct.pack('!fII', 6.0, len(ranges), len(ranges))
    payload += struct.pack(f'!{len(ranges)}f', *ranges)
    payload += struct.pack(f'!II{len(angles)}f', len(angles), len(angles), *angles)
    payload += struct.pack('!III', 0, 0, 1)
    return packet(6, 3, timestamp, payload)


def receive_exact(sock, count):
    result = b''
    while len(result) < count:
        part = sock.recv(count - len(result))
        if not part:
            raise EOFError('bridge disconnected')
        result += part
    return result


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class PipelineTimingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.listener = socket.socket()
        cls.addClassCleanup(cls.listener.close)
        cls.listener.bind(('127.0.0.1', 0))
        cls.listener.listen()
        cls.listener.settimeout(15)
        cls.ready = threading.Event()
        cls.server_error = None
        cls.connection = None

        def server():
            try:
                cls.connection, _ = cls.listener.accept()
                cls.addClassCleanup(cls.connection.close)
                cls.connection.settimeout(15)
                cls.connection.sendall(b'Player v.3.1.0'.ljust(32, b'\x00'))
                requests = 0
                while requests < 8:
                    fields = HEADER.unpack(receive_exact(cls.connection, 40))
                    receive_exact(cls.connection, fields[-1])
                    if fields[4] == 3:
                        ack = bytearray(28)
                        ack[19] = 1
                        cls.connection.sendall(packet(fields[2], fields[5], 0, ack, 4))
                        requests += 1
                cls.ready.set()
            except Exception as error:
                cls.server_error = error
                cls.ready.set()

        cls.thread = threading.Thread(target=server, daemon=True)
        cls.thread.start()
        cls.log = tempfile.TemporaryFile(mode='w+')
        cls.addClassCleanup(cls.log.close)
        env = os.environ.copy()
        env.pop('ROBOT_BRIDGE_HEARTBEAT', None)
        cls.bridge = subprocess.Popen([
            sys.argv[1], '--ros-args', '-p', 'player_host:=127.0.0.1',
            '-p', f'player_port:={cls.listener.getsockname()[1]}',
        ], env=env, stdout=cls.log, stderr=cls.log)
        cls.addClassCleanup(cls.stop_bridge)
        rclpy.init()
        cls.addClassCleanup(rclpy.shutdown)
        cls.node = rclpy.create_node('offline_slam_pipeline_probe')
        cls.addClassCleanup(cls.node.destroy_node)
        cls.scans = []
        cls.odometry = []
        cls.transforms = []
        cls.static_transforms = []
        cls.node.create_subscription(TFMessage, '/tf_static', cls.static_transforms.append,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.node.create_subscription(LaserScan, '/scan', cls.scans.append, 100)
        cls.node.create_subscription(Odometry, '/odom', cls.odometry.append, 100)
        cls.node.create_subscription(TFMessage, '/tf', cls.transforms.append, 100)
        if not cls.ready.wait(15) or cls.server_error:
            raise RuntimeError(f'fake Player unavailable: {cls.server_error}')
        deadline = time.monotonic() + 10
        while cls.node.count_publishers('/scan') < 1 or cls.node.count_publishers('/odom') < 1:
            if time.monotonic() > deadline:
                raise RuntimeError('bridge publishers not discovered')
            rclpy.spin_once(cls.node, timeout_sec=0.05)
        cls.spin_for(0.5)

    @classmethod
    def spin_for(cls, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.01)

    @classmethod
    def stop_bridge(cls):
        cls.bridge.terminate()
        try:
            cls.bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            cls.bridge.kill()
            cls.bridge.wait()

    def test_nonuniform_scan_is_dropped_and_next_valid_scan_is_published(self):
        self.scans.clear()
        self.connection.sendall(scan(60005.0, angles=[-0.2, 0.0, 0.3], ranges=[1.0, 2.0, 3.0]))
        self.spin_for(0.2)
        self.assertEqual(self.scans, [], 'nonuniform rays must not be published with invented bearings')
        self.connection.sendall(scan(60005.1))
        self.spin_for(0.2)
        self.assertEqual(len(self.scans), 1, 'dropping a malformed scan must not kill the bridge')

    def test_turn_keeps_a_stationary_wall_in_place(self):
        self.scans.clear()
        self.odometry.clear()
        self.transforms.clear()
        yaw = 0.2
        mount_yaw = math.radians(97.6)
        angles = [-math.pi + i * math.tau / 360 for i in range(360)]
        ranges = []
        for angle in angles:
            cosine = math.cos(yaw + mount_yaw + angle)
            distance = (3.0 + 0.1 * math.cos(yaw)) / cosine if cosine > 0 else 0.0
            ranges.append(distance if 0.15 <= distance <= 6.0 else 0.0)
        self.connection.sendall(odom(60010.0, 0.0) + scan(60010.5, angles, ranges))
        self.spin_for(0.25)
        self.connection.sendall(odom(60011.0, 0.4))
        self.spin_for(0.3)
        self.assertEqual(len(self.scans), 1)
        self.assertEqual(len(self.odometry), 2)
        self.assertTrue(self.static_transforms, 'probe infrastructure: no laser mounting TF')
        buffer = Buffer()
        for message in self.static_transforms:
            for transform in message.transforms:
                buffer.set_transform_static(transform, 'probe')
        for message in self.transforms:
            for transform in message.transforms:
                buffer.set_transform(transform, 'probe')
        measured_scan = self.scans[0]
        transform = buffer.lookup_transform('odom', 'laser',
                                            rclpy.time.Time.from_msg(measured_scan.header.stamp))
        rotation = transform.transform.rotation
        orientation = 2 * math.atan2(rotation.z, rotation.w)
        errors = []
        for index, distance in enumerate(measured_scan.ranges):
            if measured_scan.range_min <= distance <= measured_scan.range_max:
                angle = measured_scan.angle_min + index * measured_scan.angle_increment
                world_x = transform.transform.translation.x + distance * math.cos(orientation + angle)
                errors.append(abs(world_x - 3.0))
        self.assertGreater(len(errors), 50)
        maximum = max(errors)
        print(f'STATIONARY_WALL: maximum x error={maximum:.6f}m', flush=True)
        self.assertLess(maximum, 0.001,
                        'a stationary wall must not move when odometry is delayed during a turn')

    def test_same_measurement_time_survives_different_transport_delays(self):
        self.scans.clear()
        self.odometry.clear()
        self.connection.sendall(scan(60000.0))
        self.spin_for(0.25)
        self.connection.sendall(odom(60000.0, 0.1))
        self.spin_for(0.3)
        self.assertTrue(self.scans, 'probe infrastructure: no scan')
        self.assertTrue(self.odometry, 'probe infrastructure: no odometry')
        difference = abs(stamp(self.scans[-1]) - stamp(self.odometry[-1]))
        print(f'SAME_SOURCE_TIME: ROS stamp difference={difference:.6f}s', flush=True)
        self.assertLess(difference, 0.01, 'same Player measurement time must map to same ROS time')

    def test_batched_odometry_retains_measurement_intervals(self):
        self.odometry.clear()
        self.transforms.clear()
        self.connection.sendall(b''.join(odom(60001 + i * 0.05, i * 0.02) for i in range(10)))
        self.spin_for(0.5)
        self.assertEqual(len(self.odometry), 10, 'probe infrastructure: missing odometry')
        duration = stamp(self.odometry[-1]) - stamp(self.odometry[0])
        print(f'BATCH: Player duration=0.450000s, ROS duration={duration:.6f}s', flush=True)
        for message in self.odometry:
            matching = [t for m in self.transforms for t in m.transforms
                        if t.header.stamp == message.header.stamp]
            self.assertTrue(matching, 'odom/TF stamps must agree')
            self.assertAlmostEqual(matching[0].transform.rotation.z, message.pose.pose.orientation.z)
        self.assertAlmostEqual(duration, 0.45, delta=0.01,
                               msg='buffering must not collapse physical motion time')


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(PipelineTimingTest)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(not result.wasSuccessful())
