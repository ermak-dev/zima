#!/usr/bin/env python3
"""FEATURE: Shared MJPEG transport, without ROS or a physical camera.

SPEC: Expand ~ in netrc paths, authenticate by hostname, and retain the
upstream Content-Type (including its boundary) and unmodified stream bytes.
"""
import base64
import importlib.util
import os
from pathlib import Path
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "camera_mjpeg", ROOT / "ros2" / "camera_mjpeg.py")
cam = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cam)

CONTENT_TYPE = 'multipart/x-mixed-replace; boundary="local-camera-boundary"'
BODY = (b'--local-camera-boundary\r\nContent-Type: image/jpeg\r\n\r\n'
        b'\xff\xd8offline-fake\xff\xd9\r\n--local-camera-boundary--\r\n')


class CameraTransportTests(unittest.TestCase):
    def setUp(self):
        self.home = tempfile.TemporaryDirectory()
        self.addCleanup(self.home.cleanup)
        self.addCleanup(patch.stopall)
        patch.dict(os.environ, {"HOME": self.home.name}).start()
        self.netrc = Path(self.home.name) / "camera.netrc"
        self.authorized = False
        self.require_auth = False
        test = self

        class CameraHandler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass  # Never log requests or authentication data.

            def do_GET(self):
                expected = "Basic " + base64.b64encode(
                    b"offline-user:offline-password").decode()
                test.authorized = self.headers.get("Authorization") == expected
                if test.require_auth and not test.authorized:
                    self.send_response(401)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", CONTENT_TYPE)
                self.send_header("Content-Length", str(len(BODY)))
                self.end_headers()
                self.wfile.write(BODY)

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), CameraHandler)
        self.addCleanup(self.server.server_close)
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(thread.join)
        self.addCleanup(self.server.shutdown)
        self.url = f"http://127.0.0.1:{self.server.server_port}/mjpeg"

    def write_netrc(self, text):
        self.netrc.write_text(text)
        self.netrc.chmod(0o600)

    def assert_stream(self, netrc_path):
        with cam.open_mjpeg(self.url, netrc_path) as response:
            self.assertEqual(response.headers["Content-Type"], CONTENT_TYPE)
            self.assertEqual(response.read(), BODY)
        self.assertTrue(response.closed)

    def test_tilde_netrc_auth_and_upstream_boundary(self):
        self.require_auth = True
        self.write_netrc("machine 127.0.0.1 login offline-user "
                         "password offline-password\n")
        self.assert_stream("~/camera.netrc")
        self.assertTrue(self.authorized)

    def test_missing_netrc_allows_open_camera(self):
        self.assert_stream(str(self.netrc))
        self.assertFalse(self.authorized)

    def test_malformed_netrc_allows_open_camera(self):
        self.write_netrc("invalid-netrc-directive\n")
        self.assert_stream(str(self.netrc))
        self.assertFalse(self.authorized)

    def test_unmatched_netrc_host_allows_open_camera(self):
        self.write_netrc("machine other.invalid login offline-user "
                         "password offline-password\n")
        self.assert_stream(str(self.netrc))
        self.assertFalse(self.authorized)


if __name__ == "__main__":
    unittest.main()
