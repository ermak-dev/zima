#!/usr/bin/env python3
"""Offline odom->base_link TF broadcaster for bag replay.

The 2026-10-04 bag predates TF support in the bridge: it carries /odom
and /scan only. When replaying it through SLAM Toolbox, this node turns
each /odom message into the matching TF transform (the same mapping the
bridge publishes live), so the full chain odom -> base_link -> laser
exists without the robot.

Timestamp policy: the recorded scans are unstamped (pre-fix bridge) and
odometry carries record-time stamps, so BOTH are re-stamped at arrival
time — and the odom->base_link transform is additionally re-published
with each scan's stamp, so the TF tree always covers the exact instant
SLAM asks about (the robot was stationary in this recording, making the
pose valid at any nearby time; for a moving recording this node must be
replaced by pose interpolation).

The static base_link -> laser mounting matches the bridge defaults
(laser_x=-0.100, yaw 97.6 deg).
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def yaw_to_quat(yaw):
    q = TransformStamped().transform.rotation.__class__()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdomTfReplay(Node):
    def __init__(self):
        super().__init__('odom_tf_replay')
        self.declare_parameter('laser_x', -0.100)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw_deg', 97.6)
        self.pub = self.create_publisher(TFMessage, '/tf', 50)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_stamped', 20)
        self.create_subscription(
            Odometry, '/odom', self.on_odom, QoSProfile(depth=50))
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, QoSProfile(depth=20))
        self.static_sent = False
        # latest odom pose, re-stamped as TF on demand
        self.latest_pose = None

    def _stamp_now(self):
        return self.get_clock().now().to_msg()

    def _tf_message(self, stamp):
        """odom->base_link plus the static base_link->laser, both stamped
        at the given time so the TF cache always covers scan instants."""
        tf = TFMessage()
        if self.latest_pose is None:
            return None
        pos, rot = self.latest_pose
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.rotation = rot
        tf.transforms.append(t)

        yaw = math.radians(self.get_parameter('laser_yaw_deg').value)
        s = TransformStamped()
        s.header.stamp = stamp
        s.header.frame_id = 'base_link'
        s.child_frame_id = 'laser'
        s.transform.translation.x = self.get_parameter('laser_x').value
        s.transform.translation.y = self.get_parameter('laser_y').value
        s.transform.rotation = yaw_to_quat(yaw)
        tf.transforms.append(s)
        return tf

    def on_odom(self, msg):
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.latest_pose = (pos, msg.pose.pose.orientation)

        tf = TFMessage()
        t = TransformStamped()
        t.header.stamp = self._stamp_now()
        t.header.frame_id = msg.header.frame_id or 'odom'
        t.child_frame_id = msg.child_frame_id or 'base_link'
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.rotation = msg.pose.pose.orientation
        tf.transforms.append(t)

        self.pub.publish(tf)

    def on_scan(self, msg):
        stamp = self._stamp_now()
        msg.header.stamp = stamp
        self.scan_pub.publish(msg)

        # Republish odom->base_link with the scan's exact stamp so the
        # message filter always finds a matching transform.
        tf = self._tf_message(stamp)
        if tf is not None:
            self.pub.publish(tf)


def main():
    rclpy.init()
    node = OdomTfReplay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
