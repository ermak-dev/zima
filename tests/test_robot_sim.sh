#!/usr/bin/env bash
# FEATURE: `robot run --sim` wrapper (scripts/run-sim.sh).
# SPEC: launch the distro TB3 Nav2 simulation with the project nav params,
# wait for /scan + /cmd_vel, end on launcher exit or Ctrl-C, sweep the
# simulator clean. Fakes for ros2; no Gazebo, no robot.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/fake-bin"
export ROBOT_TEST_LOG=$tmp/log ROBOT_TEST_DIR=$tmp
: > "$ROBOT_TEST_LOG"

fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }
assert_log() { grep -Fq -- "$1" "$ROBOT_TEST_LOG" || fail "expected log: $1"; }

: > "$tmp/ros-setup.sh"

cat > "$tmp/fake-bin/ros2" <<'EOF'
#!/usr/bin/env bash
printf 'ros2:%s\n' "$*" >> "$ROBOT_TEST_LOG"
case $1 in
    launch)
        trap 'printf "launch:term\n" >> "$ROBOT_TEST_LOG"; exit 0' TERM
        # Snapshot the params file the wrapper merged: the wrapper cleans
        # it up on teardown, so assertions must run on this copy.
        for a in "$@"; do
            case $a in params_file:=*) cp "${a#params_file:=}" "$ROBOT_TEST_DIR/params-snapshot.yaml";; esac
        done
        if [[ ${ROBOT_SIM_NEVER_UP:-0} == 1 ]]; then
            sleep 600 &
            wait $!
        fi
        printf 'nav2_bringup tb3_simulation_launch.py\n' >> "$ROBOT_TEST_LOG"
        printf '%s\n' "$*" >> "$ROBOT_TEST_DIR/launch-args"
        sleep "${ROBOT_SIM_LAUNCH_SECS:-600}" &
        wait $!
        ;;
    topic)
        if [[ $2 == list ]]; then
            if [[ -e $ROBOT_TEST_DIR/sim-up ]]; then
                printf '/cmd_vel\n/scan\n/map\n'
            fi
            exit 0
        fi
        ;;
esac
exit 0
EOF

chmod +x "$tmp/fake-bin/ros2"

run_sim() {
    env -u DISPLAY \
        PATH="$tmp/fake-bin:$PATH" \
        ROBOT_TEST_LOG="$ROBOT_TEST_LOG" ROBOT_TEST_DIR="$tmp" \
        ROBOT_ROS_SETUP="$tmp/ros-setup.sh" ROBOT_WS_SETUP="$tmp/ws-missing.sh" \
        ROBOT_SIM_TRIES=2 \
        "$@"
}

# The playground must spawn the Roborock model (not the stock TB3): the
# SDF exists, the launch gets it, and the SDF carries the vacuum's laser
# mounting from the live bridge geometry.
[[ -f $root/models/roborock_v1/model.sdf ]] || fail 'roborock SDF model missing'
grep -q 'gz-sim-diff-drive-system' "$root/models/roborock_v1/model.sdf" || fail 'SDF lacks diff drive'
grep -q 'gz-sim-sensors-lidar-system\|gpu_lidar' "$root/models/roborock_v1/model.sdf" || fail 'SDF lacks a lidar'
: > "$tmp/launch-args"

# --- normal session: launcher exits by itself (RViz closed) ---
touch "$tmp/sim-up"
: > "$ROBOT_TEST_LOG"
run_sim ROBOT_SIM_LAUNCH_SECS=3 \
    bash "$root/scripts/run-sim.sh" >"$tmp/out" 2>&1 \
    || fail "sim session exited nonzero: $(cat "$tmp/out")"
grep -q 'tb3_simulation_launch.py' "$tmp/launch-args" || fail 'tb3 launch not used'
# The launch takes ONE params file: a merged temp one that must carry the
# base config's limits and the sim overlay's spawn-point initial pose
# (snapshotted by the fake at launch time; the wrapper cleans the temp).
[[ -f $tmp/params-snapshot.yaml ]] || fail 'params snapshot missing'
python3 - "$tmp/params-snapshot.yaml" <<'PY' || fail 'merged params lack base limits or sim overlay'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
assert d['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] == 0.06
assert d['amcl']['ros__parameters']['initial_pose']['x'] == -2.0
assert d['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] == 'cmd_vel'
PY
grep -q 'headless:=False' "$tmp/launch-args" || fail '3D world disabled by default'
# The launch must be told to spawn the Roborock model and name it.
grep -q -- "robot_sdf:=$root/models/roborock_v1/model.sdf" "$tmp/launch-args" \
    || fail "roborock SDF not passed: $(cat "$tmp/launch-args")"
grep -q 'robot_name:=roborock_v1' "$tmp/launch-args" || fail 'robot_name not set'
grep -q 'playground up' "$tmp/out" || fail 'playground-ready banner missing'
grep -q '\[robot sim\] done' "$tmp/out" || fail 'teardown banner missing'

# --- Ctrl-C path: a terminal SIGINT/TERM goes to the whole foreground
# group; emulate that faithfully (setsid wrapper, signal to the group).
# A bare `kill -TERM $!` on a function-backgrounded wrapper hits the
# intermediate subshell and never reaches the script's traps.
: > "$ROBOT_TEST_LOG"; rm -f "$tmp/launch-args" "$tmp/out"
run_sim setsid bash "$root/scripts/run-sim.sh" >"$tmp/out" 2>&1 &
subshell=$!
# The setsid child becomes the group leader; find it under the subshell.
for _ in 1 2 3 4 5; do
    wpid=$(ps -eo pid,ppid,cmd | awk -v p="$subshell" '$2==p && /run-sim/ {print $1; exit}')
    [[ -n ${wpid:-} ]] && break
    sleep 1
done
[[ -n ${wpid:-} ]] || { kill -KILL "$subshell" 2>/dev/null; fail 'wrapper did not start'; }
until [[ -f $tmp/out ]] && grep -q 'playground up' "$tmp/out"; do sleep 1; done
kill -TERM -- "-$wpid" 2>/dev/null || kill -TERM "$wpid" 2>/dev/null || true
for _ in 1 2 3 4 5; do kill -0 "$wpid" 2>/dev/null || break; sleep 1; done
kill -0 "$wpid" 2>/dev/null && fail 'wrapper survived group TERM'
wait "$subshell" 2>/dev/null || true
assert_log 'launch:term'

# --- params override via env (replaces the base, merge still applies) ---
: > "$ROBOT_TEST_LOG"; rm -f "$tmp/launch-args" "$tmp/params-snapshot.yaml"
cat > "$tmp/custom-nav.yaml" <<'Y'
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.03
amcl:
  ros__parameters:
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
Y
run_sim ROBOT_SIM_LAUNCH_SECS=2 ROBOT_SIM_PARAMS="$tmp/custom-nav.yaml" \
    bash "$root/scripts/run-sim.sh" >/dev/null 2>&1 || fail 'custom-params session failed'
[[ -f $tmp/params-snapshot.yaml ]] || fail 'params snapshot missing (custom run)'
python3 - "$tmp/params-snapshot.yaml" <<'PY' || fail 'custom base not honored through the merge'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
assert d['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] == 0.03
assert d['amcl']['ros__parameters']['initial_pose']['x'] == -2.0
PY

# --- simulator never comes up: exit 3, no playground banner ---
: > "$ROBOT_TEST_LOG"; rm -f "$tmp/launch-args" "$tmp/sim-up"
status=0
run_sim ROBOT_SIM_NEVER_UP=1 \
    bash "$root/scripts/run-sim.sh" >"$tmp/out" 2>&1 || status=$?
[[ $status == 3 ]] || fail "never-up session exited $status, want 3"
grep -q 'did not come up' "$tmp/out" || fail 'failure banner missing'
assert_log 'launch:term'
printf 'robot sim session tests passed\n'
