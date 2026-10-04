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
if [[ $1 == -G ]]; then printf 'hostname 192.0.2.82\n'; exit; fi
printf 'ssh:%s\n' "$*" >> "$ROBOT_TEST_LOG"
if [[ $* == 'robot bash -s' ]]; then
    script=$(cat)
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
    exec "$(dirname "$0")/mask-owner" "${@:3}"
else
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
"$root/bin/robot" stop
assert_log 'robotctl:192.0.2.82 stop'
"$root/bin/robot" fan
assert_log 'robotctl:192.0.2.82 fan'
[[ -d $root/bin/controller ]] && fail 'bin/controller still exists'
[[ ! -e $root/bin/controller/player.c ]] || fail 'C player client still exists'
printf 'robot CLI tests passed\n'

# --- controller handoff internals (executed remotely by bin/robot) ---
cat > "$tmp/bin/ssh" <<'EOF'
#!/usr/bin/env bash
[[ $1 == robot && $2 == bash && $3 == -s ]] || exit 9
exec bash -s
EOF
cat > "$tmp/bin/initctl" <<'EOF'
#!/usr/bin/env bash
printf 'initctl:%s\n' "$*" >> "$ROBOT_TEST_LOG"
[[ ! -e ${ROBOT_FAIL_RESTART:-/nonexistent} ]] || exit 1
if grep -q '^RoboController,' "$ROBOT_PROCESS_LIST"; then echo alive > "$ROBOT_TEST_STATE"; fi
EOF
cat > "$tmp/bin/pgrep" <<'EOF'
#!/usr/bin/env bash
if [[ $* == '-x WatchDoge' ]]; then exit 0; fi
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
