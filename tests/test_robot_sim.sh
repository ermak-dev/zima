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
# Every simulated link must sit where the URDF (the TF that ROS publishes)
# says. SDF places a link by the link's own <pose>; a joint <pose> only
# moves the joint frame relative to the child. A mount copied from a URDF
# joint origin into the SDF joint left the lidar at the body center on the
# floor: scans grazed the ground, missed the pillars and broke SLAM.
python3 - "$root/models/roborock_v1/model.sdf" "$root/models/roborock_v1/roborock_v1.urdf" <<'PY' \
    || fail 'SDF link placement differs from the URDF joint origins'
import sys, xml.etree.ElementTree as ET
sdf = ET.parse(sys.argv[1]).getroot().find('model')
urdf = ET.parse(sys.argv[2]).getroot()
def xyz(text):
    return [round(float(v), 4) for v in (text or '0 0 0').split()[:3]]
links = {l.get('name'): l for l in sdf.findall('link')}
for joint in urdf.findall('joint'):
    child = joint.find('child').get('link')
    origin = joint.find('origin')
    want = xyz(origin.get('xyz') if origin is not None else None)
    pose = links[child].find('pose')
    got = xyz(pose.text if pose is not None else None)
    assert got == want, f'{child}: SDF link pose {got}, URDF origin {want}'
for joint in sdf.findall('joint'):
    pose = joint.find('pose')
    assert pose is None or xyz(pose.text) == [0, 0, 0], \
        f'{joint.get("name")}: joint <pose> does not place the child link'
PY
# The robot must stand on its wheels and caster: an SDF cylinder's axis is
# its local z, so a wheel needs a roll to turn about its joint axis (the
# URDF visuals carry rpy 1.5708 0 0). Without it the "wheels" were flat
# pucks flipping like coins, the body slid on the floor, and the wheel
# odometry overstated every turn — SLAM smeared the map. The body must
# clear the floor so the wheels carry the weight.
python3 - "$root/models/roborock_v1/model.sdf" <<'PY' || fail 'SDF robot does not stand on its wheels'
import math, sys, xml.etree.ElementTree as ET
sdf = ET.parse(sys.argv[1]).getroot().find('model')
links = {l.get('name'): l for l in sdf.findall('link')}
def pose6(el):
    p = el.find('pose')
    return [float(v) for v in (p.text if p is not None else '0 0 0 0 0 0').split()]
def collision(link):
    c = link.find('collision')
    return c, pose6(c)
supports = []
for joint in sdf.findall('joint'):
    child = links[joint.find('child').text]
    lz = pose6(child)[2]
    col, (cx, cy, cz, roll, pitch, yaw) = collision(child)
    if joint.get('type') == 'revolute':
        axis = [float(v) for v in joint.find('axis/xyz').text.split()]
        # Local z of the collision after roll/pitch/yaw (the cylinder axis).
        z = [math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll),
             math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll),
             math.cos(pitch) * math.cos(roll)]
        dot = abs(sum(a * b for a, b in zip(axis, z)))
        assert dot > 0.99, f'{child.get("name")}: wheel cylinder axis {z} is not the joint axis {axis}'
        supports.append(lz + cz - float(col.find('geometry/cylinder/radius').text))
    sphere = col.find('geometry/sphere/radius')
    if sphere is not None:
        supports.append(lz + cz - float(sphere.text))
assert supports, 'no wheels or caster found'
ground = min(supports)
assert max(supports) - ground < 0.002, f'supports do not share the floor: {supports}'
body, (_, _, bz, _, _, _) = collision(links['base_link'])
bottom = pose6(links['base_link'])[2] + bz - float(body.find('geometry/cylinder/length').text) / 2
assert bottom - ground >= 0.005, f'body bottom {bottom:.3f} m does not clear the floor at {ground:.3f} m'
PY

# Nav2 drive limits (base config, shared by the sim and the replay).
# Every Nav2 velocity source — the controller and the recovery behaviors
# (the stock tree backs up at 0.15 m/s) — goes through the velocity
# smoother, so its bounds are the stack's ceiling and must sit within
# the project limits (0.1 m/s, 0.4 rad/s). The controller is Regulated
# Pure Pursuit inside those bounds: DWB with the gentle acceleration
# limits never left its dynamic window and stalled 0.24 m short of every
# goal in the sandbox. Turning in place must count as progress, or a
# U-turn toward a goal aborts as "failed to make progress".
python3 - "$root/config/nav2_replay.yaml" <<'PY' || fail 'Nav2 drive limits or controller setup wrong'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
vs = d['velocity_smoother']['ros__parameters']
assert vs['max_velocity'][0] <= 0.1 and vs['max_velocity'][2] <= 0.4, vs['max_velocity']
assert vs['min_velocity'][0] >= -0.1 and vs['min_velocity'][2] >= -0.4, vs['min_velocity']
c = d['controller_server']['ros__parameters']
fp = c['FollowPath']
assert fp['plugin'] == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController', fp['plugin']
assert fp['desired_linear_vel'] <= vs['max_velocity'][0]
assert fp['rotate_to_heading_angular_vel'] <= vs['max_velocity'][2]
assert not fp.get('allow_reversing', False)
assert c['progress_checker']['plugin'] == 'nav2_controller::PoseProgressChecker'
assert c['progress_checker']['required_movement_angle'] > 0
PY
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
assert d['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] == 0.06
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
grep -q 'slam:=False' "$tmp/launch-args" || fail 'mapping must be off by default (prebuilt map + AMCL)'

# --- --slam: online SLAM Toolbox instead of the prebuilt map + AMCL;
# click goals in RViz and the map grows as the robot drives ---
: > "$ROBOT_TEST_LOG"; rm -f "$tmp/launch-args" "$tmp/params-snapshot.yaml" "$tmp/out"
run_sim ROBOT_SIM_LAUNCH_SECS=2 \
    bash "$root/scripts/run-sim.sh" --slam >"$tmp/out" 2>&1 \
    || fail "slam session exited nonzero: $(cat "$tmp/out")"
grep -q 'slam:=True' "$tmp/launch-args" || fail "slam not enabled: $(cat "$tmp/launch-args")"
[[ -f $tmp/params-snapshot.yaml ]] || fail 'params snapshot missing (slam run)'
python3 - "$tmp/params-snapshot.yaml" <<'PY' || fail 'merged params lack the online SLAM section'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
s = d['slam_toolbox']['ros__parameters']
# Frames of the bridge contract: the slam_toolbox default base_footprint
# silently broke the first offline map.
assert (s['base_frame'], s['odom_frame'], s['map_frame']) == ('base_link', 'odom', 'map')
assert s['mode'] == 'mapping'
# Online, SLAM itself publishes map->odom; a zero period disables it.
assert s['transform_publish_period'] > 0
# Base limits and the sim cmd_vel chain survive the extra overlay.
assert d['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] == 0.06
assert d['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] == 'cmd_vel'
# Goals in not-yet-mapped space must stay plannable while mapping.
assert d['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is True
PY
grep -q 'map_saver_cli' "$tmp/out" || fail 'slam banner does not tell how to save the map'

# --- refuses to start next to another ROS session on this host: a second
# simulator fights over /clock and node names (observed: both crashed),
# and next to the live bridge the sim's Nav2 /cmd_vel would reach the
# real wheels. Fakes match by command line, as pgrep -f sees them. ---
for stray_cmd in 'ros2 launch nav2_bringup tb3_simulation_launch.py' 'player_to_ros2'; do
    : > "$ROBOT_TEST_LOG"
    setsid bash -c "exec -a '$stray_cmd' sleep 600" >/dev/null 2>&1 &
    stray=$!
    status=0
    # A short fake launch: a missing guard fails the asserts, not a hang.
    run_sim ROBOT_SIM_LAUNCH_SECS=2 \
        bash "$root/scripts/run-sim.sh" --slam >"$tmp/out" 2>&1 || status=$?
    kill -TERM "$stray" 2>/dev/null || true
    wait "$stray" 2>/dev/null || true
    [[ $status == 2 ]] || fail "sim next to '$stray_cmd' exited $status, want 2"
    if grep -q '^ros2:launch' "$ROBOT_TEST_LOG"; then fail "sim launched next to '$stray_cmd'"; fi
    grep -q 'already running' "$tmp/out" || fail "refusal does not say why: $(cat "$tmp/out")"
done

# --- unknown option: refuse before launching anything ---
: > "$ROBOT_TEST_LOG"
status=0
run_sim bash "$root/scripts/run-sim.sh" --bogus >"$tmp/out" 2>&1 || status=$?
[[ $status == 2 ]] || fail "unknown option exited $status, want 2"
if grep -q '^ros2:launch' "$ROBOT_TEST_LOG"; then fail 'unknown option still launched the simulator'; fi

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
      desired_linear_vel: 0.03
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
assert d['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] == 0.03
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
