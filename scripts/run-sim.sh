#!/bin/bash
# Offline Nav2 playground: Gazebo TB3 simulation with the full Nav2 stack
# and RViz (click-to-goal). Started by `robot run --sim`. Never touches
# the real robot — no ssh, no bridge, no supervisor.
# (no set -u: ROS setup scripts reference unbound vars)
set -eo pipefail
ROS_SETUP=${ROBOT_ROS_SETUP:-/opt/ros/jazzy/setup.sh}
# shellcheck source=/dev/null
source "$ROS_SETUP"
# The simulator stack is distro-provided; the local workspace is not
# needed, but keep the session consistent with `robot run` when present.
WS_SETUP=${ROBOT_WS_SETUP:-$HOME/robot/ros-ws/install/setup.sh}
# shellcheck source=/dev/null
source "$WS_SETUP" 2>/dev/null || true

repo=$(cd "$(dirname "$(realpath "$0")")/.." && pwd)
# shellcheck source=scripts/lib.sh
source "$(dirname "$(realpath "$0")")/lib.sh"

NAV_PARAMS=${ROBOT_SIM_PARAMS:-$repo/config/nav2_replay.yaml}
SIM_OVERLAY=$repo/config/nav2_sim.yaml
RVIZ=${ROBOT_SIM_RVIZ:-True}
# Headless (no 3D window) for scripted/offline checks; RViz still opens
# unless ROBOT_SIM_RVIZ:=False.
HEADLESS=${ROBOT_SIM_HEADLESS:-False}
# The playground spawns the vacuum model, not the stock TurtleBot3: frames
# and laser mounting match the live player_to_ros2 bridge contract.
ROBOT_SDF=${ROBOT_SIM_ROBOT_SDF:-$repo/models/roborock_v1/model.sdf}
ROBOT_URDF=$repo/models/roborock_v1/roborock_v1.urdf
ROBOT_NAME=${ROBOT_SIM_ROBOT_NAME:-roborock_v1}

# Merge the sim overlay into the base params (the launch takes one file):
# PyYAML is in the ROS desktop install; if absent, plain-text concat of the
# two files also works because the overlay redefines whole top-level
# sections only.
merged=$(mktemp /tmp/robot-nav2-sim.XXXXXX.yaml)
if python3 - "$NAV_PARAMS" "$SIM_OVERLAY" "$merged" <<'PY'
import sys
try:
    import yaml
except ImportError:
    sys.exit(10)
base, overlay, out = sys.argv[1:4]
b = yaml.safe_load(open(base)) or {}
o = yaml.safe_load(open(overlay)) or {}
for k, v in o.items():
    if k in b and isinstance(b.get(k), dict) and isinstance(v, dict):
        for k2, v2 in v.items():
            if isinstance(v2, dict) and isinstance(b[k].get(k2), dict):
                b[k][k2].update(v2)
            else:
                b[k][k2] = v2
    else:
        b[k] = v
yaml.safe_dump(b, open(out, 'w'), sort_keys=False)
PY
then :; else
    cat "$NAV_PARAMS" "$SIM_OVERLAY" >"$merged"
fi
NAV_PARAMS=$merged

banner() { printf '\n[robot sim] %s\n' "$*"; }
die() { echo "robot sim: $*" >&2; exit 2; }

command -v ros2 >/dev/null || die 'ros2 is not in PATH; source ROS 2 Jazzy'
[[ -f $NAV_PARAMS ]] || die "nav2 params not found: $NAV_PARAMS"

pids=()
sim_rc=0

cleanup() {
    local status=$?
    # Swallow further Ctrl-C while tearing down; every step is bounded.
    trap '' INT TERM EXIT
    (( sim_rc )) || sim_rc=$status
    rm -f "$merged" "${rsp_params:-}"
    banner 'teardown'
    # The gz simulator detaches from its launcher group on its own path;
    # sweep by command line so nothing lingers after the session.
    pkill -TERM -f 'gz sim' 2>/dev/null || true
    stop_components "${pids[@]}"
    pkill -KILL -f 'gz sim' 2>/dev/null || true
    banner 'done'
    exit "$sim_rc"
}
trap 'cleanup interrupt' INT TERM
trap 'cleanup exit' EXIT

banner "starting Gazebo + Nav2 with the roborock_v1 model (headless:=$HEADLESS, rviz:=$RVIZ)"
: >/tmp/robot-sim.log
setsid ros2 launch nav2_bringup tb3_simulation_launch.py \
    headless:="$HEADLESS" use_rviz:="$RVIZ" \
    params_file:="$NAV_PARAMS" \
    robot_sdf:="$ROBOT_SDF" robot_name:="$ROBOT_NAME" \
    use_robot_state_pub:=False \
    >>/tmp/robot-sim.log 2>&1 &
LAUNCH_PID=$!
pids+=("$LAUNCH_PID")

# The distro launch hardcodes the TB3 URDF in its robot_state_publisher;
# ours must describe the vacuum, so run our own publisher alongside (it
# only serves RViz's RobotModel display; Nav2 uses TF from the bridge).
rsp_params=$(mktemp /tmp/robot-rsp.XXXXXX.yaml)
rsp_params_write "$ROBOT_URDF" "$rsp_params" true
setsid ros2 run robot_state_publisher robot_state_publisher \
    --ros-args --params-file "$rsp_params" >>/tmp/robot-sim.log 2>&1 &
URDF_PID=$!
pids+=("$URDF_PID")

banner 'waiting for the simulator (/cmd_vel publishers, /scan stream)'
up=0
for _ in $(seq 1 "${ROBOT_SIM_TRIES:-45}"); do
    if ros2 topic list 2>/dev/null | grep -q '^/cmd_vel$' \
        && ros2 topic list 2>/dev/null | grep -q '^/scan$'; then
        up=1; break
    fi
    kill -0 "$LAUNCH_PID" 2>/dev/null || break
    sleep 2
done
if (( ! up )); then
    echo 'robot sim: simulator did not come up; see /tmp/robot-sim.log' >&2
    exit 3
fi
banner 'playground up: RViz is open — use 2D Goal Pose to send the robot; Ctrl-C here to end'

# End on the launcher dying (RViz window close kills the launch) or Ctrl-C.
wait "$LAUNCH_PID" || true
banner 'session over'
exit 0
