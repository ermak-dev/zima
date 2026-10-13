#!/usr/bin/env python3
"""MJPEG camera stream -> ROS 2 CompressedImage publisher.

The OpenIPC camera serves an MJPEG multipart stream over plain HTTP
(http://<cam>/mjpeg). MJPEG is the lowest-latency option of the camera's
endpoints (no GOP/keyframe waits, frames are self-contained JPEGs), and
each frame maps onto sensor_msgs/CompressedImage with zero transcoding:
the JPEG bytes go on the wire as-is, decoding happens in consumers
(image_proc, RViz, web panels).

QoS: Sensor Data (Best Effort, depth 5) — a late frame is worthless,
the newest one matters; Reliable QoS would queue up over Wi-Fi.

The multipart framing parser (split_mjpeg_frames) is a pure function
tested by tests/test_web_panel.py; this module stays importable without
rclpy, like the other ros2/ scripts.

Auth: like bin/camera/snapshot, credentials come from a mode-0600 netrc
(machine <host>); they never enter the stream URL or logs.
"""
import base64
import threading
import time
import urllib.request

# Pull SOI..EOI-delimited JPEG frames out of an arbitrary byte stream
# (multipart headers, boundaries and any garbage are skipped).
_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"


def split_mjpeg_frames(buf):
    """Return (complete_jpeg_frames, remainder).

    The remainder starts at the next SOI with no matching EOI yet, so the
    caller can prepend it to the next read chunk; nothing is lost across
    buffer boundaries.
    """
    frames = []
    start = buf.find(_SOI)
    while start != -1:
        end = buf.find(_EOI, start + 2)
        if end == -1:
            break  # frame still arriving
        frames.append(buf[start:end + 2])
        buf = buf[end + 2:]
        start = buf.find(_SOI)
    return frames, buf


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage

    class CameraMjpeg(Node):
        def __init__(self):
            super().__init__("camera_mjpeg")
            self.declare_parameter("url", "http://cam/mjpeg")
            self.declare_parameter("netrc",
                                   "~/.config/robot/camera.netrc")
            self.declare_parameter("fps", 10.0)
            qos = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.pub = self.create_publisher(CompressedImage,
                                             "/camera/image_compressed", qos)
            self.seq = 0
            threading.Thread(target=self.stream_loop, daemon=True).start()

        def stream_loop(self):
            import netrc
            url = self.get_parameter("url").value
            netrc_path = self.get_parameter("netrc").value
            fps = self.get_parameter("fps").value
            min_interval = 1.0 / fps if fps > 0 else 0.0
            while True:
                try:
                    auth = None
                    try:
                        entry = netrc.netrc(netrc_path).authenticators(
                            urllib.parse.urlsplit(url).hostname)
                        if entry:
                            token = base64.b64encode(
                                f"{entry[0]}:{entry[2]}".encode()).decode()
                            auth = {"Authorization": f"Basic {token}"}
                    except (FileNotFoundError, netrc.NetrcParseError):
                        pass  # camera may be open on the LAN
                    self.stream(url, auth, min_interval)
                except Exception as e:  # reconnect with backoff
                    self.get_logger().warn(f"camera stream: {e}; retry in 2 s")
                    time.sleep(2.0)

        def stream(self, url, auth, min_interval):
            req = urllib.request.Request(url, headers=auth or {})
            with urllib.request.urlopen(req, timeout=10) as resp:
                self.get_logger().info(f"camera streaming from {url}")
                buf = b""
                last_pub = 0.0
                while True:
                    chunk = resp.read(4096)
                    if not chunk:
                        raise IOError("camera closed the stream")
                    buf += chunk
                    frames, buf = split_mjpeg_frames(buf)
                    for jpeg in frames:
                        now = time.monotonic()
                        if now - last_pub < min_interval:
                            continue  # throttle to the fps budget
                        last_pub = now
                        self.publish(jpeg)

        def publish(self, jpeg):
            from rclpy.qos import QoSProfile  # noqa: F401 (doc anchor)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.format = "jpeg"
            msg.data = bytearray(jpeg)
            self.seq += 1
            self.pub.publish(msg)

    rclpy.init()
    node = CameraMjpeg()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
