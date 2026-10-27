#!/usr/bin/env python3
"""FEATURE: panel transport is a single push WebSocket instead of polling.

SPEC: /ws streams state/scan/map on change (bounded rates, map only on
version change), accepts clamped cmd_vel messages and survives garbage.
Camera proxy and index page keep their old behaviour over aiohttp.
"""
import asyncio
import http.server
import json
import sys
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from websocket import create_connection

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
import web_panel

INDEX = b'<html><title>robot panel</title></html>'


class FakeSource:
    """Same interface the ROS Panel node implements; no ROS needed here."""

    def __init__(self):
        self.state = {'odom': (1.0, 2.0, 0.1, 0.0), 'odom_yaw': 0.0,
                      'battery': (0.9, 20.0, False)}
        self.scan = {'points': [(1.0, 0.0)], 'range_max': 8.0, 'frame': 'odom'}
        self.map = {'version': 7, 'frame': 'map'}
        self.published = []

    def state_snapshot(self):
        return dict(self.state)

    def scan_snapshot(self):
        return dict(self.scan) if self.scan else None

    def map_snapshot(self):
        return dict(self.map) if self.map else None

    def publish_cmd(self, v, w):
        self.published.append((v, w))


class Server:
    """aiohttp app on an ephemeral port, served from a background loop."""

    def __init__(self, source, **kwargs):
        self.source = source
        self.kwargs = kwargs
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)

    def __enter__(self):
        self.thread.start()
        async def start():
            app = web_panel.make_app(self.source, INDEX, **self.kwargs)
            self.runner = web_panel.web.AppRunner(app, shutdown_timeout=0.1)
            await self.runner.setup()
            site = web_panel.web.TCPSite(self.runner, '127.0.0.1', 0)
            await site.start()
            return self.runner.addresses[0][1]
        self.port = asyncio.run_coroutine_threadsafe(start(), self.loop).result(5)
        return f'ws://127.0.0.1:{self.port}/ws', f'http://127.0.0.1:{self.port}'

    def __exit__(self, *exc):
        async def stop():
            await self.runner.cleanup()
        asyncio.run_coroutine_threadsafe(stop(), self.loop).result(5)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(5)
        self.loop.close()


def recv_until(ws, kind, limit=20):
    """Read frames until one of `kind` arrives; return it and keep the rest."""
    for _ in range(limit):
        msg = json.loads(ws.recv())
        if msg['type'] == kind:
            return msg
    raise AssertionError(f'no {kind} message among first {limit}')


class WebSocketTests(unittest.TestCase):
    def connect(self, url):
        ws = create_connection(url, timeout=3)
        self.addCleanup(ws.close)
        return ws

    def test_connect_gets_full_state_scan_map_snapshot(self):
        with Server(FakeSource()) as (ws_url, _):
            ws = self.connect(ws_url)
            state = recv_until(ws, 'state')
            self.assertEqual(state['data']['odom']['x'], 1.0)
            self.assertEqual(state['data']['battery']['percent'], 90)
            scan = recv_until(ws, 'scan')
            self.assertEqual(scan['data']['points'], [[1.0, 0.0]])
            self.assertEqual(scan['data']['frame'], 'odom')
            map_msg = recv_until(ws, 'map')
            self.assertEqual(map_msg['data']['version'], 7)

    def test_missing_scan_and_map_are_omitted_not_zero(self):
        source = FakeSource()
        source.scan = None
        source.map = None
        with Server(source) as (ws_url, _):
            ws = self.connect(ws_url)
            state = recv_until(ws, 'state')
            self.assertIn('odom', state['data'])
            # No scan/map frames may arrive after the state snapshot.
            with self.assertRaises(Exception):
                ws.settimeout(0.4)
                while True:
                    msg = json.loads(ws.recv())
                    self.assertNotIn(msg['type'], ('scan', 'map'))

    def test_updates_push_without_polling(self):
        source = FakeSource()
        with Server(source) as (ws_url, _):
            ws = self.connect(ws_url)
            recv_until(ws, 'state'); recv_until(ws, 'scan'); recv_until(ws, 'map')
            source.state['odom'] = (3.0, 4.0, 0.0, 0.5)
            source.map['version'] = 8
            state = recv_until(ws, 'state')
            self.assertEqual(state['data']['odom']['x'], 3.0)
            map_msg = recv_until(ws, 'map')
            self.assertEqual(map_msg['data']['version'], 8)

    def test_map_with_same_version_is_not_resent(self):
        source = FakeSource()
        with Server(source) as (ws_url, _):
            ws = self.connect(ws_url)
            recv_until(ws, 'map')
            source.map['extra'] = 'mutated-but-same-version'
            source.state['odom'] = (9.0, 9.0, 0.0, 0.0)
            ws.settimeout(0.8)
            deadline = 8
            for _ in range(deadline):
                msg = json.loads(ws.recv())
                self.assertNotEqual(msg['type'], 'map', 'unchanged map must not be resent')
                if msg['type'] == 'state' and msg['data']['odom']['x'] == 9.0:
                    break

    def test_cmd_is_clamped_and_published(self):
        source = FakeSource()
        with Server(source) as (ws_url, _):
            ws = self.connect(ws_url)
            recv_until(ws, 'state')
            ws.send(json.dumps({'type': 'cmd', 'v': 5, 'w': -9}))
            ws.send(json.dumps({'type': 'cmd', 'v': 0.05, 'w': 0.2}))
            ws.send(json.dumps({'type': 'cmd', 'v': 'oops'}))
            ws.send(json.dumps({'type': 'cmd', 'v': float('nan'), 'w': float('nan')}))
            for _ in range(40):
                if len(source.published) >= 2:
                    break
                ws.settimeout(0.1)
                try:
                    ws.recv()
                except Exception:
                    pass
            self.assertEqual(source.published[0], (0.1, -0.4))
            self.assertEqual(source.published[1], (0.05, 0.2))

    def test_garbage_does_not_kill_the_connection(self):
        source = FakeSource()
        with Server(source) as (ws_url, _):
            ws = self.connect(ws_url)
            recv_until(ws, 'state')
            ws.send('not json at all')
            ws.send(json.dumps({'type': 'unknown'}))
            ws.send(json.dumps({'type': 'cmd', 'v': 0.02, 'w': 0.0}))
            for _ in range(40):
                if source.published:
                    break
                ws.settimeout(0.1)
                try:
                    ws.recv()
                except Exception:
                    pass
            self.assertEqual(source.published, [(0.02, 0.0)])


class HttpTests(unittest.TestCase):
    def test_index_page_is_served(self):
        with Server(FakeSource()) as (_, http_url):
            with urllib.request.urlopen(http_url + '/', timeout=3) as r:
                self.assertEqual(r.read(), INDEX)
                self.assertIn('text/html', r.headers['Content-Type'])

    def test_camera_failure_is_503(self):
        with Server(FakeSource()) as (_, http_url):
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(http_url + '/camera.mjpeg', timeout=3)
            self.assertEqual(caught.exception.code, 503)

    def test_camera_boundary_and_bytes_pass_through(self):
        body = (b'--camera-boundary\r\nContent-Type: image/jpeg\r\n\r\n'
                b'\xff\xd8test\xff\xd9\r\n--camera-boundary--\r\n')

        class Camera(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=camera-boundary')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        srv = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Camera)
        thread = threading.Thread(target=srv.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(thread.join)
        self.addCleanup(srv.server_close)
        self.addCleanup(srv.shutdown)
        with Server(FakeSource(), camera_url=f'http://127.0.0.1:{srv.server_port}/mjpeg') as (_, http_url):
            with urllib.request.urlopen(http_url + '/camera.mjpeg', timeout=3) as r:
                self.assertEqual(r.headers['Content-Type'],
                                 'multipart/x-mixed-replace; boundary=camera-boundary')
                self.assertEqual(r.read(), body)


if __name__ == '__main__':
    unittest.main()
