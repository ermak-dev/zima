#!/bin/bash
# Offline SLAM pipeline with self-checks at each stage.
# (no set -u: ROS setup scripts reference unbound vars)
source /opt/ros/jazzy/setup.sh
# shellcheck source=/dev/null
source "$HOME/robot/ros-ws/install/setup.sh"
LOG=/tmp/slam-run.log
: > "$LOG"

# refuse to start on top of a leftover instance — duplicate /tf with
# mismatched stamps is exactly what breaks the message filter
# match the python process itself, not this script's or a shell's
# command line that merely mentions the name
if pgrep -f '[o]dom_tf_replay.py$' >/dev/null; then
    echo "[abort] leftover odom_tf_replay running" >> "$LOG"
    exit 9
fi
echo "[stage] replay node" >> "$LOG"
# The replay node always runs: it re-stamps scans onto the bag odom
# timeline; REPLAY_TF only controls whether it also republishes TF.
python3 ~/robot/ros2/odom_tf_replay.py >> "$LOG" 2>&1 &
REPLAY=$!
sleep 1

echo "[stage] bag play (rate 0.5 for stability)" >> "$LOG"
ros2 bag play "${BAG:-$HOME/robot-bags/live-session-20261004}" --rate 0.5 >> "$LOG" 2>&1 &
BAG=$!
sleep 4

echo "[stage] tf check" >> "$LOG"
timeout 6 ros2 run tf2_ros tf2_echo odom laser >> "$LOG" 2>&1

echo "[stage] slam toolbox" >> "$LOG"
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$HOME/robot/config/slam_offline.yaml use_sim_time:=false >> "$LOG" 2>&1 &
SLAM=$!

# wait for the bag to finish, then save the map
wait $BAG
BAG_RC=$?
echo "[stage] bag done rc=$BAG_RC" >> "$LOG"
sleep 5
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: ${MAP_NAME:-./maps/live_session_20261004}}}" >> "$LOG" 2>&1
[[ -n $REPLAY ]] && kill $REPLAY 2>/dev/null; kill $SLAM 2>/dev/null
[[ -n $REPLAY ]] && wait $REPLAY 2>/dev/null; wait $SLAM 2>/dev/null
echo "[stage] done" >> "$LOG"
exit $BAG_RC
