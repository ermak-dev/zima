#!/usr/bin/env python3
"""Self-contained web control panel for the robot (one HTTP port).

Serves a single HTML page with: live camera view (the camera's MJPEG
proxied through this server so the browser needs no credentials), a
lidar canvas drawn from /scan, all chassis sensor readouts (the same
set as the RViz overlay), an odom pose readout, and keyboard teleop
publishing geometry_msgs/Twist to /cmd_vel (clamped to the project
limits 0.1 m/s / 0.4 rad/s server-side; the same limits as the bridge).

No rosbridge/foxglove dependency: the page polls /state.json and draws
locally; /camera.mjpeg streams multipart; /cmd_vel accepts POST.

Pure logic (state_to_json, scan_to_points, clamp_cmd, INDEX_HTML) is
module-level and covered by tests/test_web_panel.py; ROS imports live
inside main() so the module imports without rclpy.
"""
import json
import math
import threading

MAX_LINEAR = 0.1
MAX_ANGULAR = 0.4


def state_to_json(state):
    """Serialize the overlay-style sensor state dict for /state.json."""
    out = {}
    battery = state.get("battery")
    if battery:
        percent, volts, charging = battery
        out["battery"] = {
            "percent": round(percent * 100),
            "volts": round(volts, 1),
            "charging": bool(charging),
        }
    if "bumper" in state:
        out["bumper"] = "PRESSED" if state["bumper"] else "clear"
    if state.get("wall") is not None:
        out["wall_ir"] = state["wall"]
    for name in ("cliff", "drop"):
        if state.get(name):
            out[f"{name}_ir"] = state[name]
    if state.get("sonar") is not None:
        out["sonar"] = state["sonar"]
    odom = state.get("odom")
    if odom:
        x, y, vx, vyaw = odom
        yaw = state.get("odom_yaw", 0.0)
        out["odom"] = {"x": round(x, 3), "y": round(y, 3),
                       "vx": round(vx, 2), "yaw": round(yaw, 3),
                       "yaw_rate": round(vyaw, 2)}
    return out


def scan_to_points(angle_min, angle_increment, ranges, range_max):
    """LaserScan ranges -> (angle_deg_list, [x, y] list), robot-forward-up.

    Out-of-range beams (inf/nan, > range_max) are dropped; angles come
    back in degrees for canvas drawing.
    """
    angles, points = [], []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0 or r > range_max:
            continue
        a = angle_min + i * angle_increment
        angles.append(round(math.degrees(a), 2))
        # robot frame: x forward, y left (canvas rotates in the browser)
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


INDEX_HTML = """<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot panel</title>
<link rel="icon" href="/favicon.svg" type="image/svg+xml">
<style>
:root{--bg:#16181d;--card:#1f232b;--fg:#e8eaed;--dim:#9aa0a6;
--accent:#4da3ff;--warn:#ff4032;--ok:#3ddc84}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
font:14px/1.45 system-ui,sans-serif;display:grid;
grid-template-columns:1fr 1fr;gap:12px;padding:12px}
h1{font-size:16px;margin:0 0 8px;color:var(--dim);font-weight:600}
.card{background:var(--card);border-radius:10px;padding:12px;
min-width:0}
#camera img{width:100%;display:block;border-radius:6px;
background:#000;min-height:120px}
canvas{width:100%;aspect-ratio:1;display:block;border-radius:6px;
background:#0d0f12}
.row{display:flex;justify-content:space-between;padding:2px 0}
.row b{font-weight:600}
.pressed{color:var(--warn);font-weight:700}
.charging{color:var(--ok)}
#help{color:var(--dim);font-size:12px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px}
.kv span:nth-child(odd){color:var(--dim)}
@media(max-width:760px){body{grid-template-columns:1fr}}
</style></head><body>
<div class="card"><h1>Camera</h1><div id="camera">
<img id="cam" alt="camera stream"></div></div>
<div class="card"><h1>Lidar /scan</h1><canvas id="scan" width="600"
height="600"></canvas></div>
<div class="card"><h1>Sensors</h1><div id="sensors">waiting…</div></div>
<div class="card"><h1>Robot model &amp; pose</h1>
<canvas id="model" width="600" height="300"></canvas>
<div class="kv" id="odom"></div>
<p id="help">Teleop: hold arrows (also WASD), Space = stop.
Limits: 0.1 m/s, 0.4 rad/s.</p></div>
<script>
const $=id=>document.getElementById(id);
// camera feed: swap to a placeholder while the stream is down
const cam=$('cam');
cam.onerror=()=>{cam.style.display='none';
  cam.insertAdjacentHTML('afterend',
  '<div id="cam-off" style="color:#9aa0a6;padding:24px;text-align:center">'+
  'camera offline</div>')};
cam.onload=()=>{const off=$('cam-off');if(off)off.remove();
  cam.style.display='block'};
cam.src='/camera.mjpeg';
const scan=$('scan').getContext('2d');
function drawScan(a){
  fetch('/scan.json').then(r=>r.json()).then(s=>{
    if(!s||!s.points)return;
    const c=scan,W=600,maxr=Math.max(1,s.range_max),k=(W/2-10)/maxr;
    c.fillStyle='#0d0f12';c.fillRect(0,0,W,W);
    c.strokeStyle='#333';c.beginPath();c.moveTo(W/2,0);c.lineTo(W/2,W);
    c.moveTo(0,W/2);c.lineTo(W,W/2);c.stroke();
    c.fillStyle='#4da3ff';
    // points are robot-frame (x fwd, y left): canvas x=y_left, y=-x_fwd
    for(const[x,y]of s.points){
      c.fillRect(W/2+y*k-1.5,W/2-x*k-1.5,3,3);}
    c.save();c.translate(W/2,W/2);c.fillStyle='#e8eaed';
    c.beginPath();c.moveTo(0,-10);c.lineTo(7,8);c.lineTo(-7,8);
    c.closePath();c.fill();c.restore();
  });
}
setInterval(drawScan,500);
function drawModel(pose){
  const c=$('model').getContext('2d'),W=600,H=300;
  c.fillStyle='#0d0f12';c.fillRect(0,0,W,H);
  // trail scale: 4 m across the canvas
  const k=(W-40)/4,ox=W/2,oy=H/2;
  c.strokeStyle='#333';c.beginPath();c.moveTo(0,oy);c.lineTo(W,oy);
  c.moveTo(ox,0);c.lineTo(ox,H);c.stroke();
  fetch('/odom.json').then(r=>r.json()).then(o=>{
    if(!o)return;
    $('odom').innerHTML=`<span>x</span><b>${o.x} m</b>
     <span>y</span><b>${o.y} m</b>
     <span>v</span><b>${o.vx} m/s</b>
     <span>ω</span><b>${o.yaw_rate} rad/s</b>`;
    const x=ox+o.x*k,y=oy-o.y*k;
    c.save();c.translate(x,y);c.rotate(o.yaw||0);
    c.fillStyle='#3ddc84';c.beginPath();c.arc(0,0,14,0,7);c.fill();
    c.fillStyle='#16181d';c.beginPath();c.moveTo(0,-6);c.lineTo(5,5);
    c.lineTo(-5,5);c.closePath();c.fill();c.restore();
  });
}
setInterval(drawModel,300);
function poll(){
  fetch('/state.json').then(r=>r.json()).then(s=>{
    const rows=[];
    if(s.battery)rows.push(`<div class="row"><span>battery</span>
      <b class="${s.battery.charging?'charging':''}">
      ${s.battery.percent}% ${s.battery.volts}V
      ${s.battery.charging?'charging':''}</b></div>`);
    if(s.bumper)rows.push(`<div class="row"><span>bumper</span>
      <b class="${s.bumper==='PRESSED'?'pressed':''}">${s.bumper}</b></div>`);
    if(s.wall_ir!==undefined)rows.push(`<div class="row">
      <span>wall IR</span><b>${s.wall_ir.toFixed(3)} m</b></div>`);
    if(s.cliff_ir)rows.push(`<div class="row"><span>cliff IR</span>
      <b>${s.cliff_ir.map(v=>v.toFixed(2)).join(' ')}</b></div>`);
    if(s.drop_ir)rows.push(`<div class="row"><span>drop IR</span>
      <b>${s.drop_ir.map(v=>v.toFixed(2)).join(' ')}</b></div>`);
    if(s.sonar!==undefined)rows.push(`<div class="row"><span>sonar</span>
      <b>${s.sonar.toFixed(3)} m</b></div>`);
    $('sensors').innerHTML=rows.join('')||'no sensor data yet';
  }).catch(()=>{});
}
setInterval(poll,700);poll();
// ---- teleop ----
let cur={v:0,w:0};
function send(){fetch('/cmd_vel',{method:'POST',
  headers:{'Content-Type':'application/json'},
  body:JSON.stringify(cur)}).catch(()=>{})}
function stop(){cur={v:0,w:0};send()}
const keys={ArrowUp:['v',1],KeyW:['v',1],ArrowDown:['v',-1],KeyS:['v',-1],
ArrowLeft:['w',1],KeyA:['w',1],ArrowRight:['w',-1],KeyD:['w',-1]};
addEventListener('keydown',e=>{
  if(e.code==='Space'){stop();e.preventDefault();return}
  const k=keys[e.code];if(!k)return;e.preventDefault();
  cur[k[0]]=k[1];send()});
addEventListener('keyup',e=>{const k=keys[e.code];if(!k)return;
  cur[k[0]]=0;send()});
addEventListener('blur',stop);
</script></body></html>
"""


def main():
    import http.server
    import urllib.parse

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import BatteryState, LaserScan, Range
    from std_msgs.msg import Bool, Float32MultiArray

    class Panel(Node):
        def __init__(self):
            super().__init__("web_panel")
            self.state = {}
            self.scan = None
            self.port = self.declare_parameter("port", 8085).value
            self.camera_url = self.declare_parameter(
                "camera_url", "http://cam/mjpeg").value
            self.netrc_path = self.declare_parameter(
                "netrc", "~/.config/robot/camera.netrc").value
            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            latched = QoSProfile(
                depth=5, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            volatile = QoSProfile(depth=10)
            self.create_subscription(Bool, "/bumper", self.on_bumper, latched)
            self.create_subscription(BatteryState, "/battery_state",
                                     self.on_battery, latched)
            self.create_subscription(Range, "/wall_ir_0", self.on_wall, latched)
            self.create_subscription(Float32MultiArray, "/cliff_ir/voltages",
                                      self.on_cliff, latched)
            self.create_subscription(Float32MultiArray, "/drop_ir/voltages",
                                      self.on_drop, latched)
            self.create_subscription(Range, "/sonar", self.on_sonar, latched)
            self.create_subscription(Odometry, "/odom", self.on_odom, volatile)
            self.create_subscription(LaserScan, "/scan", self.on_scan, volatile)

        def on_bumper(self, msg):
            self.state["bumper"] = bool(msg.data)

        def on_battery(self, msg):
            from sensor_msgs.msg import BatteryState as B
            charging = (msg.power_supply_status
                        == B.POWER_SUPPLY_STATUS_CHARGING)
            self.state["battery"] = (msg.percentage, msg.voltage, charging)

        def on_wall(self, msg):
            self.state["wall"] = msg.range

        def on_cliff(self, msg):
            self.state["cliff"] = list(msg.data)

        def on_drop(self, msg):
            self.state["drop"] = list(msg.data)

        def on_sonar(self, msg):
            self.state["sonar"] = msg.range

        def on_odom(self, msg):
            p = msg.pose.pose.position
            self.state["odom"] = (p.x, p.y, msg.twist.twist.linear.x,
                                  msg.twist.twist.angular.z)
            import math
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self.state.setdefault("odom_json", {})
            self.state["odom"] = (p.x, p.y, msg.twist.twist.linear.x,
                                  msg.twist.twist.angular.z)
            self.state["odom_yaw"] = yaw

        def on_scan(self, msg):
            angles, points = scan_to_points(
                msg.angle_min, msg.angle_increment, list(msg.ranges),
                msg.range_max)
            self.scan = {"points": points, "range_max": msg.range_max}

        # HTTP handlers ------------------------------------------------
        def dispatch(self, handler, method):
            path = urllib.parse.urlsplit(handler.path).path
            if method == "GET":
                if path == "/":
                    body = INDEX_HTML.encode()
                    handler.send(200, "text/html; charset=utf-8", body)
                    return
                if path == "/favicon.svg":
                    handler.send(200, "image/svg+xml",
                                 b'<svg xmlns="http://www.w3.org/2000/svg" '
                                 b'viewBox="0 0 16 16"><circle cx="8" cy="8" '
                                 b'r="7" fill="#4da3ff"/></svg>')
                    return
                if path == "/state.json":
                    handler.send(200, "application/json",
                                 json.dumps(state_to_json(self.state)).encode())
                    return
                if path == "/scan.json":
                    handler.send(200, "application/json",
                                 json.dumps(self.scan or {}).encode())
                    return
                if path == "/odom.json":
                    odom = self.state.get("odom")
                    body = json.dumps({} if not odom else {
                        "x": round(odom[0], 2), "y": round(odom[1], 2),
                        "vx": round(odom[2], 2),
                        "yaw": round(self.state.get("odom_yaw", 0.0), 2),
                        "yaw_rate": round(odom[3], 2)}).encode()
                    handler.send(200, "application/json", body)
                    return
                if path == "/camera.mjpeg":
                    handler.proxy_mjpeg(self.camera_url, self.netrc_path)
                    return
            if method == "POST" and path == "/cmd_vel":
                length = int(handler.headers.get("Content-Length", 0))
                try:
                    cmd = json.loads(handler.rfile.read(length))
                    v, w = clamp_cmd(float(cmd.get("v", 0)),
                                     float(cmd.get("w", 0)))
                except (ValueError, json.JSONDecodeError):
                    handler.send(400, "application/json", b'{"error":"bad cmd"}')
                    return
                t = Twist()
                t.linear.x = v
                t.angular.z = w
                self.cmd_pub.publish(t)
                handler.send(200, "application/json",
                             json.dumps({"v": v, "w": w}).encode())
                return
            handler.send_error(404)

    class Handler(http.server.BaseHTTPRequestHandler):
        panel = None  # set below
        netrc_auth = None
        mjpeg_thread = None

        def log_message(self, *a):  # keep the console quiet
            pass

        def do_GET(self):
            self.panel.dispatch(self, "GET")

        def do_POST(self):
            self.panel.dispatch(self, "POST")

        def send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def proxy_mjpeg(self, url, netrc_path):
            import base64
            import netrc
            import urllib.request
            headers = {}
            try:
                entry = netrc.netrc(netrc_path).authenticators(
                    urllib.parse.urlsplit(url).hostname)
                if entry:
                    token = base64.b64encode(
                        f"{entry[0]}:{entry[2]}".encode()).decode()
                    headers["Authorization"] = f"Basic {token}"
            except (FileNotFoundError, netrc.NetrcParseError):
                pass
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; "
                             "boundary=frame")
            self.end_headers()
            try:
                req = urllib.request.Request(url, headers=headers)
                with urllib.request.urlopen(req, timeout=10) as resp:
                    while True:
                        chunk = resp.read(4096)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        self.wfile.flush()
            except (IOError, OSError):
                pass  # browser closed or camera went away

    import sys
    rclpy.init(args=sys.argv)
    panel = Panel()
    Handler.panel = panel

    server = http.server.ThreadingHTTPServer(("0.0.0.0", panel.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    panel.get_logger().info(f"web panel on http://0.0.0.0:{panel.port}/")
    try:
        rclpy.spin(panel)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
