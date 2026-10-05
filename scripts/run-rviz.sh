#!/bin/bash
# Live RViz + teleop session on top of `robot bridge`.
# Started by `robot run`; needs ROS 2 Jazzy and the ros-ws workspace built.
# Closing the RViz window (or Ctrl-C here, or the bridge dying) stops the
# robot and hands the vacuum back to its stock controller via the on-robot
# supervisor. Closing the teleop window alone does not end the session.
# (no set -u: ROS setup scripts reference unbound vars)
set -eo pipefail
ROS_SETUP=${ROBOT_ROS_SETUP:-/opt/ros/jazzy/setup.sh}
WS_SETUP=${ROBOT_WS_SETUP:-$HOME/robot/ros-ws/install/setup.sh}
# shellcheck source=/dev/null
source "$ROS_SETUP"
# shellcheck source=/dev/null
source "$WS_SETUP" 2>/dev/null || {
    echo 'robot run: ros-ws workspace is not built; see README' >&2
    exit 2
}

repo=$(cd "$(dirname "$(realpath "$0")")/.." && pwd)
robot_bin=${ROBOT_BIN:-$repo/bin/robot}

# Same safety limits as `robot move`: they also cap the teleop keys.
TELEOP_SPEED=${ROBOT_TELEOP_SPEED:-0.1}
TELEOP_TURN=${ROBOT_TELEOP_TURN:-0.4}
RVIZ_CONFIG=${ROBOT_RVIZ_CONFIG:-$repo/config/rviz_teleop.rviz}
RUN_LOG=${ROBOT_RUN_LOG:-/tmp/robot-run.log}

banner() { printf '\n[robot run] %s\n' "$*"; }
die() { echo "robot run: $*" >&2; exit 2; }

[[ -x $robot_bin ]] || die "robot CLI not found: $robot_bin"
[[ -f $RVIZ_CONFIG ]] || die "rviz config not found: $RVIZ_CONFIG"
command -v rviz2 >/dev/null || die 'rviz2 is not in PATH'
command -v ros2 >/dev/null || die 'ros2 is not in PATH'
# A second bridge would fight the running supervisor for the heartbeat
# port and disable an already-disabled controller; refuse instead.
if pgrep -f '[p]layer_to_ros2' >/dev/null; then
    die 'a bridge session is already running (player_to_ros2 found); end it first'
fi

pids=()
BRIDGE_PID=
session_rc=0

# Teardown order: zero /cmd_vel while the bridge still forwards it (the
# last Player velocity command persists until replaced), stop every
# component, then wait for the on-robot supervisor to restore the stock
# controller and verify it did.
cleanup() {
    local status=$?
    # Swallow further Ctrl-C while tearing down: every step here is
    # time-bounded, and bailing out mid-cleanup would leave the bridge
    # alive and the robot disabled (observed live: a second ^C exited
    # before the bridge got its TERM).
    trap '' INT TERM EXIT
    local component=${1:-exit}
    (( session_rc )) || session_rc=$status
    banner "teardown ($component)"
    if [[ -n $BRIDGE_PID ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        timeout 5 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
            '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
    fi
    # The teleop node runs under the desktop terminal server, outside any
    # process group we can address: stop it by command line.
    pkill -TERM -f 'teleop_twist_keyboard --ros-args' 2>/dev/null || true
    local pid
    for pid in "${pids[@]}"; do
        # SIGTERM, not SIGINT: children backgrounded from a script inherit
        # SIG_IGN for SIGINT, so INT would not stop them.
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    for pid in "${pids[@]}"; do
        # Bounded reap: a wedged child (e.g. ssh over dead Wi-Fi) must not
        # stall the teardown; `wait` cannot be timed out, so poll instead.
        local waited=0
        while kill -0 "$pid" 2>/dev/null && (( waited < 20 )); do
            sleep 1; waited=$((waited + 1))
        done
        kill -KILL "$pid" 2>/dev/null || true
    done
    # The robot is stopping/rolled back by now (the bridge TERM reached the
    # supervisor); what follows is only verification. A further Ctrl-C may
    # skip it, but never skips the kill above.
    trap 'echo "robot run: interrupted; verify with: '"$robot_bin"' status" >&2; exit 130' INT
    trap '' TERM EXIT
    local restored=0 _
    for _ in $(seq 1 "${ROBOT_RESTORE_TRIES:-10}"); do
        if timeout 20 "$robot_bin" status 2>/dev/null | grep -q '^enabled$'; then restored=1; break; fi
        sleep 1
    done
    if (( ! restored )); then
        # The supervisor rolls back only after ACTIVATE; a bridge killed
        # before activation leaves the controller down. Restore it here.
        banner 'controller still down; restoring via robot enable'
        if timeout 120 "$robot_bin" enable >/dev/null 2>&1 \
            && timeout 20 "$robot_bin" status 2>/dev/null | grep -q '^enabled$'; then
            banner 'stock controller restored via robot enable'
        else
            echo "robot run: WARNING: stock controller did not return; run '$robot_bin enable' and check the robot" >&2
            session_rc=2
        fi
    fi
    banner 'done'
    exit "$session_rc"
}
trap 'cleanup interrupt' INT TERM
trap 'cleanup exit' EXIT

banner "starting supervised bridge ($robot_bin bridge); log: $RUN_LOG"
: >"$RUN_LOG"
# Mirror the bridge output to the console: its ssh errors (disable
# failures, upload problems) must be visible live, not only in the log.
# The whole pipeline runs inside the setsid group so teardown can stop
# both the bridge and tee with one signal.
setsid bash -c 'bash "$1" bridge 2>&1 | tee -a "$2"' _ "$robot_bin" "$RUN_LOG" </dev/null &
BRIDGE_PID=$!
pids+=("$BRIDGE_PID")

banner 'waiting for the lidar stream (/scan)'
# The budget covers slow-Wi-Fi ssh phases (binary upload, disable with
# watchdog respawn retries, the Player TCP wait): measured live at ~20 s,
# worst observed higher. Only a dead bridge ends the wait early.
scan_seen=0
for _ in $(seq 1 "${ROBOT_SCAN_TRIES:-60}"); do
    if timeout 2 ros2 topic echo /scan --once >/dev/null 2>&1; then scan_seen=1; break; fi
    kill -0 "$BRIDGE_PID" 2>/dev/null || break
done
if (( ! scan_seen )); then
    echo "robot run: no /scan from the bridge; see $RUN_LOG" >&2
    exit 3
fi
banner 'lidar stream ok'

banner 'starting RViz (close its window to end the session)'
setsid rviz2 -d "$RVIZ_CONFIG" >>"$RUN_LOG" 2>&1 &
RVIZ_PID=$!
pids+=("$RVIZ_PID")

# gnome-terminal runs its command through the desktop server, so ROS must
# be sourced inside the command itself. The teleop window is not part of
# the exit condition: Ctrl-C there only drops manual control.
teleop_cmd() {
    # Single line: gnome-terminal passes it through `bash -c` as one word.
    printf '%s ' \
        "source $ROS_SETUP; source $WS_SETUP;" \
        'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args' \
        "-p speed:=$TELEOP_SPEED -p turn:=$TELEOP_TURN"
    printf '\n'
}

if [[ -n ${DISPLAY:-} ]] && command -v gnome-terminal >/dev/null; then
    setsid gnome-terminal --wait --title='robot teleop (WASD; close RViz to end)' \
        -- bash -c "$(teleop_cmd)" >>"$RUN_LOG" 2>&1 &
    TELEOP_PID=$!
    pids+=("$TELEOP_PID")
    sleep 2
    if kill -0 "$TELEOP_PID" 2>/dev/null; then
        banner "teleop window open (speed cap $TELEOP_SPEED m/s, $TELEOP_TURN rad/s)"
    else
        banner 'teleop window failed to open (see log); RViz and bridge continue'
    fi
    # End on RViz closing, the bridge dying, or Ctrl-C here.
    wait -n "$BRIDGE_PID" "$RVIZ_PID" || true
else
    banner "teleop in this console (WASD, Ctrl-C to end; speed cap $TELEOP_SPEED m/s)"
    bash -c "$(teleop_cmd)" || true
fi

banner 'session over; shutting down'
exit 0
