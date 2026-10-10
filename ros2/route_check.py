#!/usr/bin/env python3
"""Drive a fixed goal route through NavigateToPose and report each goal.

The route crosses the TB3 sandbox: a short hop forward, the gap between
two pillar columns, the middle of the pillar grid, and back to the start.
Goals are given relative to the robot's start pose; --offset moves them
into the map frame (0,0 under online SLAM, the spawn point under AMCL).

Per goal: status, wall time, final position error, travelled path vs the
straight line. On a failed goal and at the end, the SLAM map and the
global costmap are rendered with the robot and the goal into --out-dir.
Exit status 0 only when every goal succeeded.

usage: route_check.py [--offset X,Y] [--timeout S] [--out-dir DIR]
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

ROUTE = [(0.5, 0.0), (1.45, -0.05), (2.55, 1.05), (0.0, 0.0)]
STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--offset', default='0,0', help='map-frame start X,Y')
    p.add_argument('--timeout', type=float, default=150.0, help='per goal, s')
    p.add_argument('--out-dir', default='/tmp/robot-route')
    return p.parse_args()


class Route(Node):
    def __init__(self, out_dir):
        super().__init__('route_check')
        self.out_dir = out_dir
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.grids = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        for topic in ('/map', '/global_costmap/costmap'):
            self.create_subscription(OccupancyGrid, topic,
                                     lambda m, t=topic: self.grids.__setitem__(t, m), latched)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def drive(self, gx, gy, timeout):
        """Send one goal; return (status, seconds, error m, path m, line m)."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.w = 1.0
        start, t0 = self.pose(), time.time()
        sent = self.client.send_goal_async(goal)
        while not sent.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = sent.result()
        if not handle.accepted:
            return 'REJECTED', 0.0, float('nan'), 0.0, float('nan')
        result = handle.get_result_async()
        travelled, last, sample = 0.0, start, time.time()
        while not result.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() >= sample:
                p = self.pose()
                if p and last:
                    travelled += math.dist(p, last)
                last, sample = p or last, time.time() + 0.5
        if result.done():
            status = STATUS.get(result.result().status, str(result.result().status))
        else:
            handle.cancel_goal_async()
            self.spin_for(2.0)
            status = 'TIMEOUT'
        end = self.pose()
        err = math.dist(end, (gx, gy)) if end else float('nan')
        line = math.dist(start, (gx, gy)) if start else float('nan')
        return status, time.time() - t0, err, travelled, line

    def render(self, tag, goal_xy):
        """Save the map and the global costmap with robot (blue) and goal (red)."""
        try:
            from PIL import Image, ImageDraw
        except ImportError:
            return
        self.spin_for(2.0)
        os.makedirs(self.out_dir, exist_ok=True)
        robot = self.pose()
        for topic, m in list(self.grids.items()):
            info, w, h, s = m.info, m.info.width, m.info.height, 5
            im = Image.new('RGB', (w, h))
            px = im.load()
            for k, v in enumerate(m.data):
                px[k % w, h - 1 - k // w] = ((150, 150, 150) if v < 0 else (255, 255, 255) if v == 0
                                             else (0, 0, 0) if v >= 99 else (255, int(255 - 2.5 * v), 0))
            im = im.resize((w * s, h * s), Image.NEAREST)
            d = ImageDraw.Draw(im)
            for xy, colour in ((robot, 'blue'), (goal_xy, 'red')):
                if xy:
                    u = (xy[0] - info.origin.position.x) / info.resolution * s
                    v = (h - (xy[1] - info.origin.position.y) / info.resolution) * s
                    d.ellipse([u - 7, v - 7, u + 7, v + 7], outline=colour, width=3)
            name = os.path.join(self.out_dir, f'{tag}-{topic.strip("/").replace("/", "_")}.png')
            im.save(name)
            print(f'  saved {name}', flush=True)


def main():
    args = parse_args()
    ox, oy = (float(v) for v in args.offset.split(','))
    rclpy.init()
    node = Route(args.out_dir)
    if not node.client.wait_for_server(timeout_sec=30):
        print('route: navigate_to_pose server not available', flush=True)
        return 1
    node.spin_for(2.0)
    failed = 0
    for i, (dx, dy) in enumerate(ROUTE, 1):
        gx, gy = dx + ox, dy + oy
        status, secs, err, path, line = node.drive(gx, gy, args.timeout)
        print(f'goal {i} ({gx:+.2f},{gy:+.2f}): {status:9s} t={secs:5.1f}s '
              f'err={err:.2f}m path={path:.2f}m line={line:.2f}m', flush=True)
        if status != 'SUCCEEDED':
            failed += 1
            node.render(f'goal{i}-failed', (gx, gy))
        node.spin_for(1.0)
    node.render('end', None)
    print(f'route: {len(ROUTE) - failed}/{len(ROUTE)} goals reached', flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
