#!/usr/bin/env bash
# FEATURE: Flat robot CLI with controller handoff, mask ownership and bounded movement.
# SPEC: Dispatch commands, reject unsafe arguments, stop wheels after motion.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/bin"
export ROBOT_SSH="$tmp/bin/ssh" ROBOT_TEST_LOG="$tmp/log" ROBOT_MASK_OWNER_BIN="$tmp/bin/mask-owner" ROBOT_ROBOTCTL_BIN="$tmp/bin/robotctl"
cat > "$tmp/bin/ssh" <<'EOF'
#!/usr/bin/env bash
if [[ $1 == -G ]]; then
    printf 'hostname 192.0.2.82\n'
    # Emulate ssh -G failing after output (SIGPIPE / transport error):
    # the bridge must report it, not die silently under pipefail.
    exit "${ROBOT_TEST_SSHG_RC:-0}"
fi
if [[ $* == 'robot bash -s' ]]; then
    script=$(cat)
    if [[ $script == *'ROBOT_STALE_SUPERVISOR'* ]]; then
        # Internal supervisor bookkeeping, not a controller action: keep it
        # out of the ssh: log so "touched the controller" checks hold.
        if [[ ${ROBOT_TEST_SUPERVISOR_ACTIVE:-0} == 1 ]]; then
            echo 'robot: supervisor reports an activated session; refusing to start another' >&2
            exit 1
        fi
        printf 'pkill:stale-supervisor\n' >> "$ROBOT_TEST_LOG"
        exit 0
    fi
    printf 'ssh:%s\n' "$*" >> "$ROBOT_TEST_LOG"
    if [[ $script == *'ROBOT_STATUS_COMMANDS'* ]]; then
        printf '%s\n' "${ROBOT_TEST_CONTROLLER_STATE:-disabled}"
    elif [[ $script == *'ROBOT_DISABLE_CONTROLLER'* ]]; then
        printf 'controller disabled\n'
    elif [[ $script == *'ROBOT_ENABLE_CONTROLLER'* ]]; then
        printf 'controller enabled\n'
    else
        printf '/tmp/robot-drive-guard.TEST\n'
    fi
elif [[ $1 == robot && $2 == /tmp/mask-owner/mask-owner ]]; then
    # Emulate remote execution of the deployed binary.
    printf 'ssh:%s\n' "$*" >> "$ROBOT_TEST_LOG"
    exec "$(dirname "$0")/mask-owner" "${@:3}"
elif [[ $1 == robot && $2 == *robot-supervisor*listen* ]]; then
    printf 'supervisor:%s\n' "$2" >> "$ROBOT_TEST_LOG"
    if [[ ${ROBOT_TEST_PORT_BUSY:-0} == 1 ]]; then
        # A stale never-activated supervisor refuses to give up the port.
        echo 'robot-supervisor: cannot listen on 7665: Address already in use' >&2
        exit 1
    fi
    exit 0
else
    printf 'ssh:%s\n' "$*" >> "$ROBOT_TEST_LOG"
    printf 'disabled\n'
fi
EOF
cat > "$tmp/bin/robotctl" <<'EOF'
#!/usr/bin/env bash
printf 'robotctl:%s\n' "$*" >> "$ROBOT_TEST_LOG"
[[ ${ROBOT_ROBOTCTL_FAIL:-0} != 1 ]]
EOF
cat > "$tmp/bin/camera" <<'EOF'
#!/usr/bin/env bash
printf 'camera:%s\n' "$*" >> "$ROBOT_TEST_LOG"
EOF
cat > "$tmp/bin/mask-owner" <<'EOF'
#!/usr/bin/env bash
printf 'mask-owner:%s\n' "$*" >> "$ROBOT_TEST_LOG"
[[ ${ROBOT_MASK_OWNER_FAIL:-0} != 1 ]]
EOF
chmod +x "$tmp/bin/ssh" "$tmp/bin/robotctl" "$tmp/bin/camera" "$tmp/bin/mask-owner"
export ROBOT_CAMERA_BIN="$tmp/bin/camera"

fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }
assert_log() { grep -Fqx -- "$1" "$tmp/log" || fail "expected $1"; }
expect_failure() { if "$@" >"$tmp/out" 2>&1; then fail "unexpected success: $*"; fi; }

"$root/bin/robot" --help >"$tmp/out"
grep -q 'robot enable|disable|status' "$tmp/out" || fail 'help is missing flat controller commands'
: > "$tmp/log"
"$root/bin/robot" disable
"$root/bin/robot" enable
# The fake ssh logs only its argument string; the marker scripts were
# consumed on stdin. Two bash -s dispatches mean both scripts ran.
[[ $(grep -c 'ssh:robot bash -s' "$tmp/log") == 2 ]] || fail 'enable/disable not dispatched'
: > "$tmp/log"
"$root/bin/robot" position
assert_log 'robotctl:192.0.2.82 position'
"$root/bin/robot" snapshot "$tmp/frame.jpg"
assert_log "camera:$tmp/frame.jpg"
: > "$tmp/log"
"$root/bin/robot" move forward 0.5
assert_log 'robotctl:192.0.2.82 move forward 0.5'
assert_log 'ssh:robot rm -f -- /tmp/robot-drive-guard.TEST'
[[ $(grep -n 'ssh:robot bash -s' "$tmp/log" | tail -1 | cut -d: -f1) -lt $(grep -n 'robotctl:.* move ' "$tmp/log" | cut -d: -f1) ]] || fail 'guard was not armed before movement'
: > "$tmp/log"
ROBOT_TEST_CONTROLLER_STATE=enabled expect_failure "$root/bin/robot" move forward 0.5
if grep -q 'robotctl:.* move ' "$tmp/log"; then fail 'default move bypassed controller gate'; fi
: > "$tmp/log"
ROBOT_TEST_CONTROLLER_STATE=enabled "$root/bin/robot" move --with-controller forward 0.5
assert_log 'robotctl:192.0.2.82 move forward 0.5'
assert_log 'ssh:robot rm -f -- /tmp/robot-drive-guard.TEST'
: > "$tmp/log"
ROBOT_ROBOTCTL_FAIL=1 expect_failure "$root/bin/robot" move forward 0.1
if grep -q 'ssh:robot rm -f -- /tmp/robot-drive-guard.TEST' "$tmp/log"; then fail 'guard was disarmed after failed movement'; fi
: > "$tmp/log"
for args in 'move sideways 1' 'move forward 0' 'move forward 3' 'move forward nan' 'move forward -1' 'move forward 1 extra' 'move --with-controller forward 3' 'move --with-controller sideways 0.5' 'position extra' 'enable extra' 'disable extra' 'status extra'; do
    read -r -a argv <<< "$args"
    expect_failure "$root/bin/robot" "${argv[@]}"
done
: > "$tmp/log"
"$root/bin/robot" mask hold
assert_log 'ssh:robot /tmp/mask-owner/mask-owner hold --mask 10f8ff --timeout 5'
: > "$tmp/log"
"$root/bin/robot" mask hold --mask 10f0ff
assert_log 'ssh:robot /tmp/mask-owner/mask-owner hold --mask 10f0ff --timeout 5'
: > "$tmp/log"
ROBOT_TEST_CONTROLLER_STATE=enabled expect_failure "$root/bin/robot" mask hold
if grep -q 'mask-owner' "$tmp/log"; then fail 'mask hold bypassed controller gate'; fi
: > "$tmp/log"
ROBOT_MASK_OWNER_FAIL=1 expect_failure "$root/bin/robot" mask hold
: > "$tmp/log"
"$root/bin/robot" mask restore
assert_log 'ssh:robot /tmp/mask-owner/mask-owner restore'
: > "$tmp/log"
for args in 'mask' 'mask unknown' 'mask hold --mask zzz' 'mask hold --mask 1f8ff' 'mask hold --mask 10f8ff --timeout 0' 'mask restore extra' 'mask hold --mask 10f8ff --timeout' 'mask hold --mask'; do
    read -r -a argv <<< "$args"
    expect_failure "$root/bin/robot" "${argv[@]}"
done
[[ ! -s $tmp/log ]] || fail 'invalid mask commands contacted robot'

# --- robot bridge: supervisor first, then disable, then the ROS node ---
mkdir -p "$tmp/fake-ros"
cat > "$tmp/fake-ros/ros2" <<'ROSEOF'
#!/usr/bin/env bash
printf 'ros2:%s\n' "$*" >> "$ROBOT_TEST_LOG"
[[ ${ROBOT_BRIDGE_FAIL:-0} != 1 ]]
ROSEOF
chmod +x "$tmp/fake-ros/ros2"
: > "$tmp/log"
PATH="$tmp/fake-ros:$PATH" ROBOT_TEST_CONTROLLER_STATE=disabled "$root/bin/robot" bridge
grep -q "supervisor:.*listen" "$tmp/log" || fail 'supervisor not started for bridge'
# disable travels as a stdin heredoc: count the bash -s dispatch between
# the supervisor start and the ros2 launch
[[ $(awk '/supervisor:/{s=1} /ros2:run/{e=1} s && !e && /bash -s/{n++} END{print n+0}' "$tmp/log") -ge 1 ]] || fail 'controller not disabled for bridge'
grep -q 'ros2:run player_to_ros2' "$tmp/log" || fail 'bridge node not launched'
: > "$tmp/log"
PATH="$tmp/fake-ros:$PATH" ROBOT_BRIDGE_FAIL=1 ROBOT_TEST_CONTROLLER_STATE=disabled expect_failure "$root/bin/robot" bridge
: > "$tmp/log"
# A busy heartbeat port (stale never-activated supervisor) must stop the
# bridge before it touches the controller.
ROBOT_TEST_PORT_BUSY=1 ROBOT_TEST_CONTROLLER_STATE=disabled expect_failure "$root/bin/robot" bridge
grep -q 'heartbeat port 7665 is busy' "$tmp/out" || fail 'busy heartbeat port not reported'
if grep -q 'bash -s' "$tmp/log"; then fail 'bridge touched the controller despite a busy port'; fi
: > "$tmp/log"
# A stale never-activated supervisor from an aborted session is reaped
# before a new one starts; an activated one refuses the new session.
PATH="$tmp/fake-ros:$PATH" ROBOT_TEST_CONTROLLER_STATE=disabled \
    "$root/bin/robot" bridge >/dev/null 2>&1 &
bpid=$!
sleep 2
kill -TERM "$bpid" 2>/dev/null || true
wait "$bpid" 2>/dev/null || true
grep -q 'pkill:stale-supervisor' "$tmp/log" || fail 'stale supervisor not reaped before a new session'
first_stale=$(grep -n 'pkill:stale-supervisor' "$tmp/log" | head -1 | cut -d: -f1)
first_sup=$(grep -n 'supervisor:' "$tmp/log" | head -1 | cut -d: -f1)
[[ -n $first_stale && -n $first_sup && $first_stale -lt $first_sup ]] \
    || fail 'stale supervisor reaped after starting the new one'
: > "$tmp/log"
ROBOT_TEST_SUPERVISOR_ACTIVE=1 ROBOT_TEST_CONTROLLER_STATE=disabled expect_failure "$root/bin/robot" bridge
grep -q 'refusing to start another' "$tmp/out" || fail 'activated supervisor session not refused'
if grep -q 'pkill:stale-supervisor' "$tmp/log"; then fail 'an activated supervisor was killed as stale'; fi
if grep -q 'controller disabled' "$tmp/out"; then fail 'controller was disabled despite an active session'; fi
: > "$tmp/log"
# ssh -G failing after printing the hostname: the bridge must surface the
# error instead of dying silently under set -e pipefail.
ROBOT_TEST_SSHG_RC=1 ROBOT_TEST_CONTROLLER_STATE=disabled expect_failure "$root/bin/robot" bridge
grep -q 'cannot resolve SSH host' "$tmp/out" || fail 'ssh -G failure not reported'
: > "$tmp/log"
# A hanging robot must not hang the CLI: every ssh call is bounded by a
# timeout, so `status` fails fast instead of blocking forever.
cat > "$tmp/bin/hang-ssh" <<'EOF'
#!/usr/bin/env bash
if [[ $1 == -G ]]; then printf 'hostname 192.0.2.82\n'; exit 0; fi
sleep 300
EOF
chmod +x "$tmp/bin/hang-ssh"
start=$SECONDS
ROBOT_SSH="$tmp/bin/hang-ssh" ROBOT_SSH_TIMEOUT_S=3 expect_failure "$root/bin/robot" status
elapsed=$((SECONDS - start))
(( elapsed < 30 )) || fail "status hung for ${elapsed}s on an unreachable robot"
: > "$tmp/log"
for args in 'bridge extra' 'bridge --heartbeat-port' 'bridge --timeout-ms x'; do
    read -r -a argv <<< "$args"
    expect_failure "$root/bin/robot" "${argv[@]}"
done
[[ ! -s $tmp/log ]] || fail 'invalid bridge commands contacted robot'
: > "$tmp/log"

# --- robot run: dispatches the live RViz/teleop session wrapper ---
cat > "$tmp/fake-run.sh" <<'RUNEOF'
#!/usr/bin/env bash
printf 'run-script:%s\n' "$*" >> "$ROBOT_TEST_LOG"
[[ ${ROBOT_RUN_FAIL:-0} != 1 ]]
RUNEOF
cat > "$tmp/fake-sim.sh" <<'SIMEOF'
#!/usr/bin/env bash
printf 'sim-script:%s\n' "$*" >> "$ROBOT_TEST_LOG"
[[ ${ROBOT_SIM_FAIL:-0} != 1 ]]
SIMEOF
chmod +x "$tmp/fake-run.sh" "$tmp/fake-sim.sh"
"$root/bin/robot" --help >"$tmp/out"
grep -q 'robot run' "$tmp/out" || fail 'help is missing robot run'
grep -q -- '--sim' "$tmp/out" || fail 'help is missing --sim'
grep -q -- '--slam' "$tmp/out" || fail 'help is missing --slam'
: > "$tmp/log"
ROBOT_RUN_SCRIPT="$tmp/fake-run.sh" "$root/bin/robot" run
assert_log 'run-script:'
ROBOT_SIM_SCRIPT="$tmp/fake-sim.sh" "$root/bin/robot" run --sim
assert_log 'sim-script:'
# --slam rides on the sim wrapper (map grows while driving), either order.
: > "$tmp/log"
ROBOT_SIM_SCRIPT="$tmp/fake-sim.sh" "$root/bin/robot" run --sim --slam
assert_log 'sim-script:--slam'
: > "$tmp/log"
ROBOT_SIM_SCRIPT="$tmp/fake-sim.sh" "$root/bin/robot" run --slam --sim
assert_log 'sim-script:--slam'
[[ -f $root/scripts/run-sim.sh ]] || fail 'default sim wrapper scripts/run-sim.sh is missing'
: > "$tmp/log"
ROBOT_RUN_SCRIPT="$tmp/fake-run.sh" ROBOT_RUN_FAIL=1 expect_failure "$root/bin/robot" run
ROBOT_SIM_SCRIPT="$tmp/fake-sim.sh" ROBOT_SIM_FAIL=1 expect_failure "$root/bin/robot" run --sim
: > "$tmp/log"
expect_failure "$root/bin/robot" run extra
expect_failure "$root/bin/robot" run --sim extra
# Live SLAM needs the drive admission work first: refuse, never start.
ROBOT_RUN_SCRIPT="$tmp/fake-run.sh" ROBOT_SIM_SCRIPT="$tmp/fake-sim.sh" \
    expect_failure "$root/bin/robot" run --slam
grep -q 'sim' "$tmp/out" || fail 'live --slam refusal does not point to --sim'
ROBOT_RUN_SCRIPT="$tmp/nonexistent.sh" expect_failure "$root/bin/robot" run
ROBOT_SIM_SCRIPT="$tmp/nonexistent.sh" expect_failure "$root/bin/robot" run --sim
[[ ! -s $tmp/log ]] || fail 'invalid run command started a session'
: > "$tmp/log"

"$root/bin/robot" stop
assert_log 'robotctl:192.0.2.82 stop'
"$root/bin/robot" fan
assert_log 'robotctl:192.0.2.82 fan'
[[ -d $root/bin/controller ]] && fail 'bin/controller still exists'
[[ ! -e $root/bin/controller/player.c ]] || fail 'C player client still exists'
printf 'robot CLI tests passed\n'

# --- controller handoff internals (executed remotely by bin/robot) ---
# `robot enable` deploys controller-restore.sh (mkdir/sha/upload) and then
# runs it — the same script the supervisor runs for rollback. The fake ssh
# maps the remote path onto a per-test copy; enable/disable internals run
# locally against it.
fake_remote_dir=$tmp/remote
mkdir -p "$fake_remote_dir/mask-owner"
restore_sh=$fake_remote_dir/mask-owner/controller-restore.sh
cat > "$tmp/bin/ssh" <<EOF
#!/usr/bin/env bash
case " \$* " in
    *' mkdir -p -- /tmp/mask-owner '*) exit 0 ;;
    *" sha256sum /tmp/mask-owner/controller-restore.sh "*)
        sha256sum "$restore_sh" 2>/dev/null | cut -d' ' -f1; exit 0 ;;
    *'controller-restore.sh.tmp'*)
        cat > "$fake_remote_dir/mask-owner/controller-restore.sh.tmp"
        chmod 700 "$fake_remote_dir/mask-owner/controller-restore.sh.tmp"
        mv "$fake_remote_dir/mask-owner/controller-restore.sh.tmp" "$restore_sh"
        exit 0 ;;
esac
[[ \$1 == robot && \$2 == bash && \$3 == -s ]] || exit 9
script=\$(cat)
if [[ \$script == *ROBOT_ENABLE_CONTROLLER* ]]; then
    exec bash "$restore_sh"
fi
exec bash -s <<< "\$script"
EOF
cat > "$tmp/bin/initctl" <<'EOF'
#!/usr/bin/env bash
printf 'initctl:%s\n' "$*" >> "$ROBOT_TEST_LOG"
[[ ! -e ${ROBOT_FAIL_RESTART:-/nonexistent} ]] || exit 1
if grep -q '^RoboController,' "$ROBOT_PROCESS_LIST"; then echo alive > "$ROBOT_TEST_STATE"; fi
EOF
cat > "$tmp/bin/pgrep" <<'EOF'
#!/usr/bin/env bash
if [[ $* == '-x WatchDoge' ]]; then
    # Emulate a watchdog mid-respawn: absent until the delay file drains.
    if [[ -n ${ROBOT_TEST_WD_DELAY:-} && -e $ROBOT_TEST_WD_DELAY ]]; then
        n=$(cat "$ROBOT_TEST_WD_DELAY" 2>/dev/null || echo 0)
        if (( n > 0 )); then echo $((n-1)) > "$ROBOT_TEST_WD_DELAY"; exit 1; fi
    fi
    exit 0
fi
[[ $* == '-x RoboController' ]] && [[ -e $ROBOT_TEST_STATE ]]
EOF
cat > "$tmp/bin/pkill" <<'EOF'
#!/usr/bin/env bash
[[ $* == '-TERM -x RoboController' ]] || exit 1
rm -f "$ROBOT_TEST_STATE"
EOF
chmod +x "$tmp/bin/ssh" "$tmp/bin/initctl" "$tmp/bin/pgrep" "$tmp/bin/pkill"
export PATH="$tmp/bin:$PATH" ROBOT_PROCESS_LIST="$tmp/ProcessList.conf" ROBOT_BACKUP="$tmp/ProcessList.backup" ROBOT_TEST_STATE="$tmp/controller-alive"
printf 'player,player\nRoboController,setsid RoboController&,1,3,0\nAppProxy,app\n' > "$ROBOT_PROCESS_LIST"
echo alive > "$ROBOT_TEST_STATE"
"$root/bin/robot" disable
grep -q '^# ROBOT_DISABLED: RoboController,setsid RoboController&,1,3,0$' "$ROBOT_PROCESS_LIST" || fail 'watchdog entry not disabled'
[[ ! -e $ROBOT_TEST_STATE ]] || fail 'controller still running'
"$root/bin/robot" disable
"$root/bin/robot" enable
grep -q '^RoboController,setsid RoboController&,1,3,0$' "$ROBOT_PROCESS_LIST" || fail 'watchdog entry not restored'
[[ -e $ROBOT_TEST_STATE ]] || fail 'controller did not return'
"$root/bin/robot" enable
[[ ! -e $ROBOT_BACKUP ]] || fail 'recovery backup left after successful restore'

# Watchdog readiness race: after initctl restart the old WatchDoge takes
# ~10 s to die (and upstart may crash-loop it longer); disable/enable must
# keep retrying past that window instead of failing and rolling back.
echo 7 > "$tmp/wd-delay"
ROBOT_TEST_WD_DELAY="$tmp/wd-delay" "$root/bin/robot" disable \
    || fail 'disable failed while WatchDoge was respawning'
grep -q '^# ROBOT_DISABLED:' "$ROBOT_PROCESS_LIST" || fail 'entry not disabled during watchdog delay'
echo 7 > "$tmp/wd-delay"
ROBOT_TEST_WD_DELAY="$tmp/wd-delay" "$root/bin/robot" enable \
    || fail 'enable failed while WatchDoge was respawning'

# A stale recovery copy left by an aborted rollback must not block every
# future disable: when the controller is running and the live config has
# the entry enabled (a previous session restored it), disable clears the
# stale copy and proceeds, leaving a fresh recovery copy behind.
cp -p "$ROBOT_PROCESS_LIST" "$tmp/enabled-copy"
cp -p "$ROBOT_PROCESS_LIST" "$ROBOT_BACKUP"
"$root/bin/robot" disable \
    || fail 'disable blocked by a stale recovery copy'
grep -q '^# ROBOT_DISABLED:' "$ROBOT_PROCESS_LIST" || fail 'entry not disabled despite stale copy cleanup'
[[ ! -e $ROBOT_TEST_STATE ]] || fail 'controller still running with stale copy present'
[[ -f $ROBOT_BACKUP ]] || fail 'recovery copy missing after stale-copy cleanup'
cmp -s "$tmp/enabled-copy" "$ROBOT_BACKUP" || fail 'recovery copy is not the fresh enabled config'
"$root/bin/robot" enable >/dev/null 2>&1 || fail 'enable after stale-copy cleanup failed'

sed -i 's/RoboController,setsid RoboController\&,1,3,0/RoboController,unexpected/' "$ROBOT_PROCESS_LIST"
expect_failure "$root/bin/robot" disable
grep -q '^RoboController,unexpected$' "$ROBOT_PROCESS_LIST" || fail 'unexpected entry was overwritten'

printf 'player,player\nRoboController,setsid RoboController&,1,3,0\nAppProxy,app\n' > "$ROBOT_PROCESS_LIST"
touch "$tmp/fail-restart"
ROBOT_FAIL_RESTART="$tmp/fail-restart" expect_failure "$root/bin/robot" disable
grep -q '^RoboController,setsid RoboController&,1,3,0$' "$ROBOT_PROCESS_LIST" || fail 'entry not restored after failed restart'
[[ -e $ROBOT_BACKUP ]] || fail 'lost recovery copy while restart failed'
rm "$tmp/fail-restart"
"$root/bin/robot" enable
[[ ! -e $ROBOT_BACKUP ]] || fail 'recovery copy remained after retry'
[[ -e $ROBOT_TEST_STATE ]] || fail 'controller did not recover after retry'

printf 'robot controller tests passed\n'
