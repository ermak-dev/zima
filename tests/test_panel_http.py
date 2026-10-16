#!/usr/bin/env python3
"""BUG: panel MJPEG used invented boundaries and returned 200 on camera failure.
HYPOTHESIS: proxy must open upstream first and forward its actual content type.
"""
import http.server
import sys
import threading
import unittest
import urllib.error
import urllib.request
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
import web_panel


@contextmanager
def server(handler):
    srv = http.server.ThreadingHTTPServer(('127.0.0.1', 0), handler)
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    try:
        yield f'http://127.0.0.1:{srv.server_port}'
    finally:
        srv.shutdown()
        srv.server_close()
        thread.join()


class ProxyTests(unittest.TestCase):
    def test_boundary_and_first_frame_survive_proxy(self):
        Handler = web_panel.Handler
        body = b'--camera-boundary\r\nContent-Type: image/jpeg\r\n\r\n\xff\xd8test\xff\xd9\r\n--camera-boundary--\r\n'
        class Camera(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=camera-boundary')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            def log_message(self, *args):
                pass
        with server(Camera) as upstream:
            class Proxy(Handler):
                panel = SimpleNamespace(dispatch=lambda h, method: h.proxy_mjpeg(upstream, '/nonexistent/netrc'))
            with server(Proxy) as url, urllib.request.urlopen(url, timeout=3) as response:
                self.assertEqual(response.headers['Content-Type'], 'multipart/x-mixed-replace; boundary=camera-boundary')
                self.assertEqual(response.read(), body)

    def test_upstream_failure_is_http_503(self):
        Handler = web_panel.Handler
        class Proxy(Handler):
            panel = SimpleNamespace(dispatch=lambda h, method: h.proxy_mjpeg('http://127.0.0.1:1', '/nonexistent/netrc'))
        with server(Proxy) as url:
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(url, timeout=3)
            self.assertEqual(caught.exception.code, 503)


if __name__ == '__main__':
    unittest.main()
