#!/bin/bash
# Offline Nav2 validation: full bringup (AMCL + planners + controller)
# against the replayed mapping bag. /cmd_vel lands in a sink file instead
# of any robot. Success: NavigateToPose accepted and cmd_vel non-empty.
# (no set -u: ROS setup scripts reference unbound vars)
source /opt/ros/jazzy/setup.sh
LOG=${LOG:-/tmp/nav2-replay.log}
: > "$LOG"

BAG=${BAG:-$HOME/robot-bags/mapping-drive-20261004-stamped}
PARAMS=${PARAMS:-$HOME/robot/config/nav2_replay.yaml}
SINK=${SINK:-/tmp/cmd_vel_sink.txt}
: > "$SINK"

# Command sink: every Twist is recorded, never forwarded.
ros2 topic echo /cmd_vel --field linear.x >> "$SINK" 2>/dev/null &
SINKPROC=$!

ros2 launch nav2_bringup bringup_launch.py \
    map:=$HOME/robot/maps/mapping_drive_20261004.yaml \
    use_sim_time:=false autostart:=true \
    params_file:=$PARAMS >> "$LOG" 2>&1 &
NAV=$!
sleep 8

echo "[stage] bag play" >> "$LOG"
ros2 bag play "$BAG" >> "$LOG" 2>&1 &
BAGPROC=$!

# Let localization settle on the first scans, then send a goal within
# the small test map (map spans roughly x in [-1, 0.6], y in [-0.9, 0.6]).
sleep 15
echo "[stage] goal" >> "$LOG"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: -0.3, y: 0.0}, orientation: {w: 1.0}}}}" \
    >> "$LOG" 2>&1

# Give the controller time to react to the goal while the bag replays.
sleep 60
echo "[stage] wrap up]" >> "$LOG"
kill $BAGPROC $NAV $SINKPROC 2>/dev/null
wait $BAGPROC $NAV $SINKPROC 2>/dev/null
echo "[done]" >> "$LOG"
echo "cmd_vel samples: $(wc -l < "$SINK")"
