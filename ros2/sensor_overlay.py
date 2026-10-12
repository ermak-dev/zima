#!/usr/bin/env python3
"""Live sensor value overlay for RViz (text labels over the robot).

The bridge publishes several chassis sensors as plain numbers (battery,
bumper, IR voltages, sonar) that RViz has no built-in display for. This
node subscribes to them all and renders the latest values as a
TEXT_VIEW_FACING MarkerArray above base_link, so a single Marker Array
display in RViz shows every sensor at a glance. A pressed bumper paints
its line red.

QoS: the event-driven sensors (bumper, battery, cliff/drop) are
transient_local on the bridge, so this node subscribes with the same
durability — a late-started overlay still receives the latched state.

The formatting and layout live in module-level pure functions (no ROS
imports) so tests/test_sensor_overlay.py runs with a bare python3,
exactly the way tests/test_robot_bug.py tests bin/bug.
"""

# Vertical layout of the label stack above base_link, metres.
LINE_HEIGHT = 0.035
BASE_HEIGHT = 0.20

NEUTRAL = (0.9, 0.9, 0.9, 1.0)
ALERT = (1.0, 0.25, 0.15, 1.0)

TEXT_NS = "sensor_overlay"


def format_state(state):
    """Render the latest sensor snapshot into label lines.

    `state` maps a sensor name to its latest value; a line is either a
    plain string (neutral colour) or a (text, alert) tuple (alert paints
    the label red). Sensors with no data yet are skipped, so the stack
    only grows as values arrive.
    """
    lines = []
    battery = state.get("battery")
    if battery:
        percent, volts, charging = battery
        pct = round(percent * 100)
        lines.append(f"battery {pct}% {volts:.1f}V "
                     f"{'charging' if charging else 'discharging'}")
    if state.get("bumper"):
        lines.append(("bumper PRESSED", True))
    elif "bumper" in state:
        lines.append("bumper clear")
    wall = state.get("wall")
    if wall is not None:
        lines.append(f"wall IR {wall:.3f} m")
    for name in ("cliff", "drop"):
        arr = state.get(name)
        if arr:
            values = " ".join(f"{v:.2f}" for v in arr)
            lines.append(f"{name} IR {values}")
    sonar = state.get("sonar")
    if sonar is not None:
        lines.append(f"sonar {sonar:.3f} m")
    odom = state.get("odom")
    if odom:
        x, y, vx, vyaw = odom
        lines.append(
            f"odom x{x:.2f} y{y:.2f}  v {vx:.2f} m/s {vyaw:.2f} rad/s")
    return lines


def label_layout(lines):
    """Assign each formatted line a slot: (index, alert, z-height)."""
    return [
        (i, line[1] if isinstance(line, tuple) else False,
         BASE_HEIGHT + i * LINE_HEIGHT)
        for i, line in enumerate(lines)
    ]


def main():
    import rclpy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import BatteryState, Range
    from std_msgs.msg import Bool, Float32MultiArray
    from visualization_msgs.msg import Marker, MarkerArray

    class SensorOverlay(Node):
        def __init__(self):
            super().__init__("sensor_overlay")
            self.state = {}
            self.latched = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.volatile = QoSProfile(depth=10)
            self.pub = self.create_publisher(MarkerArray, "/sensor_overlay",
                                             self.latched)
            self.create_subscription(Bool, "/bumper", self.on_bumper,
                                     self.latched)
            self.create_subscription(BatteryState, "/battery_state",
                                     self.on_battery, self.latched)
            self.create_subscription(Range, "/wall_ir_0", self.on_wall_range,
                                     self.latched)
            self.create_subscription(Float32MultiArray, "/cliff_ir/voltages",
                                     self.on_cliff, self.latched)
            self.create_subscription(Float32MultiArray, "/drop_ir/voltages",
                                     self.on_drop, self.latched)
            self.create_subscription(Range, "/sonar", self.on_sonar,
                                     self.latched)
            self.create_subscription(Odometry, "/odom", self.on_odom,
                                     self.volatile)
            self._last_used = 0
            self.publish()
            self.create_timer(0.5, self.publish)

        # -- subscriptions: remember the latest value, repaint ---------
        def on_bumper(self, msg):
            self.state["bumper"] = bool(msg.data)

        def on_battery(self, msg):
            charging = (msg.power_supply_status
                        == BatteryState.POWER_SUPPLY_STATUS_CHARGING)
            self.state["battery"] = (msg.percentage, msg.voltage, charging)

        def on_wall_range(self, msg):
            self.state["wall"] = msg.range

        def on_cliff(self, msg):
            self.state["cliff"] = list(msg.data)

        def on_drop(self, msg):
            self.state["drop"] = list(msg.data)

        def on_sonar(self, msg):
            self.state["sonar"] = msg.range

        def on_odom(self, msg):
            p = msg.pose.pose.position
            self.state["odom"] = (p.x, p.y,
                                  msg.twist.twist.linear.x,
                                  msg.twist.twist.angular.z)

        # -- rendering --------------------------------------------------
        def publish(self):
            lines = format_state(self.state)
            markers = MarkerArray()
            for i, alert, z in label_layout(lines):
                text = lines[i][0] if isinstance(lines[i], tuple) else lines[i]
                m = Marker()
                m.header.frame_id = "base_link"
                m.ns = TEXT_NS
                m.id = i
                m.type = Marker.TEXT_VIEW_FACING
                m.action = Marker.ADD
                m.text = text
                m.pose.position = Point(x=0.0, y=0.0, z=z)
                m.scale.z = 0.025
                m.color.r, m.color.g, m.color.b, m.color.a = (
                    ALERT if alert else NEUTRAL)
                markers.markers.append(m)
            # Clear stale slots when the stack shrinks (no odom yet).
            used = len(markers.markers)
            for i in range(used, self._last_used):
                m = Marker()
                m.header.frame_id = "base_link"
                m.ns = TEXT_NS
                m.id = i
                m.action = Marker.DELETE
                markers.markers.append(m)
            self._last_used = used
            self.pub.publish(markers)

    rclpy.init()
    node = SensorOverlay()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
