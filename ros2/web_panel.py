#!/usr/bin/env python3
"""Single-port ROS panel: occupancy map, MJPEG camera, sensors and teleop.

ROS imports live inside main so the HTTP proxy and pure conversions can be
verified offline. The panel never takes ownership of the bridge session.
"""
import base64
import http.server
import json
import math
import struct
import threading
import time
import urllib.parse
import zlib
from pathlib import Path

MAX_LINEAR = 0.1
MAX_ANGULAR = 0.4


def state_to_json(state):
    """Serialize known readings and an explicit inventory of missing sensors."""
    out = {}
    battery = state.get('battery')
    if battery:
        percent, volts, charging = battery
        out['battery'] = {'percent': round(percent * 100),
                          'volts': round(volts, 1), 'charging': bool(charging)}
    if 'bumper' in state:
        out['bumper'] = 'PRESSED' if state['bumper'] else 'clear'
    if state.get('wall') is not None:
        out['wall_ir'] = state['wall']
    for name in ('cliff', 'drop'):
        if state.get(name):
            out[f'{name}_ir'] = state[name]
    if state.get('sonar') is not None:
        out['sonar'] = state['sonar']
    odom = state.get('odom')
    if odom:
        x, y, vx, vyaw = odom
        out['odom'] = {'x': round(x, 3), 'y': round(y, 3),
                       'vx': round(vx, 2),
                       'yaw': round(state.get('odom_yaw', 0.0), 3),
                       'yaw_rate': round(vyaw, 2)}
    out['sensors'] = {key: out.get(key) for key in (
        'battery', 'bumper', 'wall_ir', 'cliff_ir', 'drop_ir', 'sonar')}
    for key in ('wall_voltages', 'cliff_ranges', 'drop_ranges'):
        out['sensors'][key] = state.get(key)
    out['map_pose'] = state.get('map_pose')
    out['map_age_s'] = state.get('map_age_s')
    return out


def map_to_json(width, height, resolution, origin, data, frame, version):
    """Encode ROS row order losslessly; the browser projects rows into map Y."""
    if (width <= 0 or height <= 0 or len(data) != width * height
            or not math.isfinite(resolution) or resolution <= 0):
        raise ValueError('invalid occupancy grid')
    pixels = bytes(160 if v < 0 else round(255 - min(v, 100) * 2.23) for v in data)
    rows = b''.join(b'\x00' + pixels[i:i + width] for i in range(0, len(pixels), width))

    def chunk(kind, body):
        return (struct.pack('>I', len(body)) + kind + body
                + struct.pack('>I', zlib.crc32(kind + body)))

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))
    x, y, yaw = origin
    return {'width': width, 'height': height, 'resolution': resolution,
            'origin': {'x': x, 'y': y, 'yaw': yaw}, 'frame': frame, 'version': version,
            'image': 'data:image/png;base64,' + base64.b64encode(png).decode('ascii')}


def points_in_odom(points, x, y, yaw):
    """Apply the scan-time odom <- laser TF, including the LDS mounting."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py)
            for px, py in points]


def scan_to_points(angle_min, angle_increment, ranges, range_max):
    """LaserScan ranges -> angles in degrees and points in the laser frame."""
    angles, points = [], []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0 or r > range_max:
            continue
        a = angle_min + i * angle_increment
        angles.append(round(math.degrees(a), 2))
        points.append((round(r * math.cos(a), 4), round(r * math.sin(a), 4)))
    return angles, points


def clamp_cmd(v, w):
    """Clamp a teleop command to the project limits; NaN -> 0."""
    if not math.isfinite(v):
        v = 0.0
    if not math.isfinite(w):
        w = 0.0
    return (max(-MAX_LINEAR, min(MAX_LINEAR, v)),
            max(-MAX_ANGULAR, min(MAX_ANGULAR, w)))


INDEX_HTML = Path(__file__).with_name('panel.html').read_text(encoding='utf-8')


class Handler(http.server.BaseHTTPRequestHandler):
    panel = None

    def log_message(self, *args):
        pass

    def do_GET(self):
        self.panel.dispatch(self, 'GET')

    def do_POST(self):
        self.panel.dispatch(self, 'POST')

    def send(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def proxy_mjpeg(self, url, netrc_path):
        from camera_mjpeg import open_mjpeg
        try:
            upstream = open_mjpeg(url, netrc_path)
        except (OSError, ValueError):
            self.send(503, 'text/plain; charset=utf-8', b'Camera unavailable')
            return
        try:
            with upstream:
                self.send_response(200)
                self.send_header('Content-Type', upstream.headers['Content-Type'])
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                while True:
                    chunk = upstream.read1(65536)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
        except OSError:
            pass


def main():
    import sys
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import BatteryState, LaserScan, Range
    from std_msgs.msg import Bool, Float32MultiArray
    from tf2_ros import Buffer, TransformException, TransformListener

    class Panel(Node):
        def __init__(self):
            super().__init__('web_panel')
            self.state = {}
            self.scan = None
            self.pending_scan = None
            self.map = None
            self.map_received = None
            self.map_version = time.time_ns() // 1_000_000
            self.odom_frame = 'odom'
            self.base_frame = 'base_link'
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.port = self.declare_parameter('port', 8085).value
            self.camera_url = self.declare_parameter('camera_url', 'http://cam/mjpeg').value
            self.netrc_path = self.declare_parameter('netrc', '~/.config/robot/camera.netrc').value
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            latched = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            volatile = QoSProfile(depth=10)
            self.create_subscription(Bool, '/bumper', self.on_bumper, latched)
            self.create_subscription(BatteryState, '/battery_state', self.on_battery, latched)
            self.create_subscription(Range, '/wall_ir_0', self.on_wall, latched)
            for name, count in (('cliff', 4), ('drop', 2)):
                self.create_subscription(Float32MultiArray, f'/{name}_ir/voltages',
                                         lambda msg, key=name: self.on_array(key, msg), latched)
                for i in range(count):
                    self.create_subscription(Range, f'/{name}_ir_{i}',
                                             lambda msg, key=name, index=i: self.on_range(key, index, msg), latched)
            self.create_subscription(Float32MultiArray, '/wall_ir/voltages',
                                     lambda msg: self.on_array('wall_voltages', msg), latched)
            self.create_subscription(Range, '/sonar', self.on_sonar, latched)
            self.create_subscription(Odometry, '/odom', self.on_odom, volatile)
            self.create_subscription(LaserScan, '/scan', self.on_scan, volatile)
            self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                reliability=ReliabilityPolicy.RELIABLE))
            self.create_timer(0.05, self.transform_scan)
            self.create_timer(0.1, self.update_map_pose)

        def on_map(self, msg):
            p, q = msg.info.origin.position, msg.info.origin.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            try:
                grid = map_to_json(msg.info.width, msg.info.height, msg.info.resolution,
                                   (p.x, p.y, yaw), msg.data, msg.header.frame_id,
                                   self.map_version + 1)
            except ValueError:
                self.get_logger().warning('Ignoring invalid occupancy grid')
                return
            self.map_version += 1
            self.map_received = time.monotonic()
            self.map = grid
            self.update_map_pose()

        def update_map_pose(self):
            self.state['map_pose'] = None
            if self.map is None:
                return
            try:
                transform = self.tf_buffer.lookup_transform(self.map['frame'], self.base_frame, Time())
                age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
                if age > 2.0 or age < -0.5:
                    return
                tf = transform.transform
            except TransformException:
                return
            q = tf.rotation
            self.state['map_pose'] = {
                'x': tf.translation.x, 'y': tf.translation.y,
                'yaw': math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                'frame': self.map['frame']}

        def on_bumper(self, msg):
            self.state['bumper'] = bool(msg.data)

        def on_battery(self, msg):
            charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            self.state['battery'] = (msg.percentage, msg.voltage, charging)

        def on_wall(self, msg):
            self.state['wall'] = msg.range

        def on_array(self, key, msg):
            self.state[key] = list(msg.data)

        def on_range(self, key, index, msg):
            values = list(self.state.get(f'{key}_ranges', [None] * (4 if key == 'cliff' else 2)))
            values[index] = msg.range
            self.state[f'{key}_ranges'] = values

        def on_sonar(self, msg):
            self.state['sonar'] = msg.range

        def on_odom(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.odom_frame = msg.header.frame_id
            self.base_frame = msg.child_frame_id or 'base_link'
            self.state['odom'] = (p.x, p.y, msg.twist.twist.linear.x, msg.twist.twist.angular.z)
            self.state['odom_yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                               1 - 2 * (q.y * q.y + q.z * q.z))

        def on_scan(self, msg):
            self.pending_scan = msg
            self.transform_scan()

        def transform_scan(self):
            msg = self.pending_scan
            if msg is None:
                return
            try:
                frame = self.map['frame'] if self.map else self.odom_frame
                tf = self.tf_buffer.lookup_transform(frame, msg.header.frame_id,
                                                     Time.from_msg(msg.header.stamp)).transform
            except TransformException:
                return
            _, points = scan_to_points(msg.angle_min, msg.angle_increment,
                                       msg.ranges, msg.range_max)
            q = tf.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.scan = {'points': points_in_odom(points, tf.translation.x, tf.translation.y, yaw),
                         'range_max': msg.range_max, 'frame': frame}
            self.pending_scan = None

        def dispatch(self, handler, method):
            path = urllib.parse.urlsplit(handler.path).path
            if method == 'GET':
                if path == '/':
                    handler.send(200, 'text/html; charset=utf-8', INDEX_HTML.encode())
                    return
                if path == '/favicon.svg':
                    handler.send(200, 'image/svg+xml', b'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 16 16"><circle cx="8" cy="8" r="7" fill="#2a78d6"/></svg>')
                    return
                if path == '/map.json':
                    grid = self.map
                    query = urllib.parse.parse_qs(urllib.parse.urlsplit(handler.path).query)
                    if grid and query.get('version') == [str(grid['version'])]:
                        grid = {'version': grid['version']}
                    handler.send(200, 'application/json', json.dumps(grid or {}).encode())
                    return
                if path in ('/state.json', '/scan.json', '/odom.json'):
                    snapshot = self.state.copy()
                    snapshot['map_age_s'] = (time.monotonic() - self.map_received
                                             if self.map_received is not None else None)
                    state = state_to_json(snapshot)
                    body = {'/state.json': state, '/scan.json': self.scan or {},
                            '/odom.json': state.get('odom', {})}[path]
                    handler.send(200, 'application/json', json.dumps(body).encode())
                    return
                if path == '/camera.mjpeg':
                    handler.proxy_mjpeg(self.camera_url, self.netrc_path)
                    return
            if method == 'POST' and path == '/cmd_vel':
                length = int(handler.headers.get('Content-Length', 0))
                try:
                    cmd = json.loads(handler.rfile.read(length))
                    v, w = clamp_cmd(float(cmd.get('v', 0)), float(cmd.get('w', 0)))
                except (ValueError, TypeError, AttributeError):
                    handler.send(400, 'application/json', b'{"error":"bad cmd"}')
                    return
                t = Twist()
                t.linear.x, t.angular.z = v, w
                self.cmd_pub.publish(t)
                handler.send(200, 'application/json', json.dumps({'v': v, 'w': w}).encode())
                return
            handler.send_error(404)

    rclpy.init(args=sys.argv)
    panel = Panel()
    Handler.panel = panel
    server = http.server.ThreadingHTTPServer(('0.0.0.0', panel.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    panel.get_logger().info(f'web panel on http://0.0.0.0:{panel.port}/')
    try:
        rclpy.spin(panel)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
