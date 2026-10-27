#!/usr/bin/env python3
"""Single-port ROS panel: occupancy map, MJPEG camera, sensors and teleop.

Transport is one aiohttp server: the page loads over HTTP, everything else
streams through a push WebSocket (state/scan/map on change, cmd_vel back).
ROS imports stay inside main so the camera proxy and pure conversions can
be verified offline. The panel never takes ownership of the bridge session.
"""
import asyncio
import base64
import json
import math
import struct
import time
import urllib.parse
import zlib
from pathlib import Path

import aiohttp
from aiohttp import web

MAX_LINEAR = 0.1
MAX_ANGULAR = 0.4
STATE_INTERVAL = 0.25  # seconds between state pushes even without changes
SCAN_INTERVAL = 0.1
CAMERA_RETRY = 'http://127.0.0.1:1/mjpeg'  # unreachable placeholder


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


def json_frame(kind, data):
    return json.dumps({'type': kind, 'data': data}, separators=(',', ':'))


class PanelBroadcaster:
    """Bounded-rate push fan-out of panel snapshots to all WebSocket clients."""

    def __init__(self, source):
        self.source = source
        self.clients = set()
        self.versions = {}  # per-client: what this client has already seen
        self.sent_state = None
        self.sent_scan = None
        self.sent_map_version = None

    async def start(self):
        self.task = asyncio.get_running_loop().create_task(self.run())

    async def stop(self):
        self.task.cancel()
        try:
            await self.task
        except asyncio.CancelledError:
            pass

    async def run(self):
        try:
            while True:
                await asyncio.sleep(0)
                await self.broadcast()
                await asyncio.sleep(STATE_INTERVAL)
        except asyncio.CancelledError:
            pass

    async def broadcast(self):
        state = state_to_json(self.source.state_snapshot())
        scan = self.source.scan_snapshot()
        grid = self.source.map_snapshot()
        clients = list(self.clients)
        for ws in clients:
            per = self.versions.setdefault(ws, {})
            try:
                if state != per.get('state'):
                    await ws.send_str(json_frame('state', state))
                    per['state'] = state
                if scan is not None and scan != per.get('scan'):
                    await ws.send_str(json_frame('scan', scan))
                    per['scan'] = scan
                if grid is not None and grid.get('version') != per.get('map_version'):
                    await ws.send_str(json_frame('map', grid))
                    per['map_version'] = grid['version']
            except (ConnectionResetError, RuntimeError):
                self.clients.discard(ws)

    def attach(self, ws):
        self.clients.add(ws)

    def detach(self, ws):
        self.clients.discard(ws)
        self.versions.pop(ws, None)

    def handle_cmd(self, raw):
        try:
            cmd = json.loads(raw, parse_constant=lambda _: None)
            v = float(cmd['v'])
            w = float(cmd['w'])
            if not (math.isfinite(v) and math.isfinite(w)):
                return
        except (KeyError, ValueError, TypeError, AttributeError, json.JSONDecodeError):
            return
        self.source.publish_cmd(*clamp_cmd(v, w))


async def ws_handler(request):
    broadcaster = request.app['broadcaster']
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    broadcaster.attach(ws)
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                broadcaster.handle_cmd(msg.data)
    finally:
        broadcaster.detach(ws)
    return ws


async def camera_proxy(request):
    """Stream the OpenIPC MJPEG endpoint through unchanged."""
    import netrc
    import os

    import urllib.request
    camera_url = request.app['camera_url']
    headers = {}
    try:
        entry = netrc.netrc(os.path.expanduser(request.app['netrc'])).authenticators(
            urllib.parse.urlsplit(camera_url).hostname)
        if entry:
            token = base64.b64encode(f'{entry[0]}:{entry[2]}'.encode()).decode()
            headers['Authorization'] = f'Basic {token}'
    except (FileNotFoundError, netrc.NetrcParseError):
        pass  # camera may be open on the LAN
    try:
        upstream = await asyncio.get_running_loop().run_in_executor(
            None, lambda: urllib.request.urlopen(
                urllib.request.Request(camera_url, headers=headers), timeout=10))
    except OSError:
        return web.Response(status=503, text='Camera unavailable')
    response = web.StreamResponse(
        headers={'Content-Type': upstream.headers.get('Content-Type', 'application/octet-stream'),
                 'Cache-Control': 'no-store'})
    await response.prepare(request)
    loop = asyncio.get_running_loop()
    try:
        while True:
            chunk = await loop.run_in_executor(None, upstream.read1, 65536)
            if not chunk:
                break
            await response.write(chunk)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    finally:
        upstream.close()
        await response.write_eof()
    return response


def make_app(source, index_html=INDEX_HTML, camera_url=CAMERA_RETRY,
             netrc='~/.config/robot/camera.netrc'):
    broadcaster = PanelBroadcaster(source)

    async def index(request):
        return web.Response(body=index_html, content_type='text/html', charset='utf-8')

    async def favicon(request):
        return web.Response(
            body=b'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 16 16">'
                 b'<circle cx="8" cy="8" r="7" fill="#2a78d6"/></svg>',
            content_type='image/svg+xml')

    app = web.Application()
    app['broadcaster'] = broadcaster
    app['camera_url'] = camera_url
    app['netrc'] = netrc
    app.router.add_get('/', index)
    app.router.add_get('/favicon.svg', favicon)
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/camera.mjpeg', camera_proxy)
    async def on_start(_app):
        await broadcaster.start()

    async def on_stop(_app):
        await broadcaster.stop()
    app.on_startup.append(on_start)
    app.on_cleanup.append(on_stop)
    return app


def main():
    import sys
    import threading

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
            self.lock = threading.Lock()
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

        # Snapshots consumed by the broadcaster; deep copies so the async
        # server never races a ROS callback mutating these fields mid-send.
        def state_snapshot(self):
            with self.lock:
                return dict(self.state)

        def scan_snapshot(self):
            with self.lock:
                return dict(self.scan) if self.scan else None

        def map_snapshot(self):
            with self.lock:
                return dict(self.map) if self.map else None

        def publish_cmd(self, v, w):
            t = Twist()
            t.linear.x, t.angular.z = v, w
            self.cmd_pub.publish(t)

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
            with self.lock:
                self.map_received = time.monotonic()
                self.map = grid
            self.update_map_pose()

        def update_map_pose(self):
            pose = None
            if self.map is not None:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.map['frame'], self.base_frame, Time())
                    age = (self.get_clock().now()
                           - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
                    if age > 2.0 or age < -0.5:
                        transform = None
                    tf = transform.transform if transform else None
                except TransformException:
                    tf = None
                if tf is not None:
                    q = tf.rotation
                    pose = {'x': tf.translation.x, 'y': tf.translation.y,
                            'yaw': math.atan2(2 * (q.w * q.z + q.x * q.y),
                                              1 - 2 * (q.y * q.y + q.z * q.z)),
                            'frame': self.map['frame']}
            with self.lock:
                self.state['map_pose'] = pose
                self.state['map_age_s'] = (time.monotonic() - self.map_received
                                           if self.map_received is not None else None)

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
            scan = {'points': points_in_odom(points, tf.translation.x, tf.translation.y, yaw),
                    'range_max': msg.range_max, 'frame': frame,
                    'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}
            with self.lock:
                self.scan = scan
                self.pending_scan = None

    rclpy.init(args=sys.argv)
    panel = Panel()
    app = make_app(panel, INDEX_HTML, camera_url=panel.camera_url,
                   netrc=panel.netrc_path)
    broadcaster = app['broadcaster']
    # A short shutdown timeout: Ctrl-C/SIGTERM must not wait on a browser
    # tab holding the socket open.
    runner = web.AppRunner(app, shutdown_timeout=2)

    async def serve():
        await runner.setup()
        await web.TCPSite(runner, '0.0.0.0', panel.port).start()

    loop = asyncio.new_event_loop()
    threading.Thread(target=loop.run_forever, daemon=True).start()
    asyncio.run_coroutine_threadsafe(serve(), loop).result(10)
    panel.get_logger().info(f'web panel on http://0.0.0.0:{panel.port}/')
    try:
        rclpy.spin(panel)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        asyncio.run_coroutine_threadsafe(runner.cleanup(), loop).result(10)
        loop.call_soon_threadsafe(loop.stop)
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
