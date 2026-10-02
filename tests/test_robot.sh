#!/usr/bin/env bash
# FEATURE: Reversible robot controller handoff and bounded Player movement.
# SPEC: Dispatch CLI commands, reject unsafe arguments, and stop wheels after motion.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/bin" "$tmp/controller"
export ROBOT_BIN_DIR="$tmp/controller" ROBOT_PLAYER_CLIENT="$tmp/bin/player" ROBOT_SSH="$tmp/bin/ssh" ROBOT_TEST_LOG="$tmp/log"
cat > "$tmp/bin/ssh" <<'EOF'
#!/usr/bin/env bash
if [[ $1 == -G ]]; then printf 'hostname 192.0.2.82\n'; exit; fi
printf 'ssh:%s\n' "$*" >> "$ROBOT_TEST_LOG"
if [[ $* == 'robot bash -s' ]]; then
    script=$(cat)
    if [[ $script == *'ROBOT_DISABLED'* ]]; then printf '%s\n' "${ROBOT_TEST_CONTROLLER_STATE:-disabled}"; else printf '/tmp/robot-drive-guard.TEST\n'; fi
else
    printf 'disabled\n'
fi
EOF
cat > "$tmp/bin/player" <<'EOF'
#!/usr/bin/env bash
printf 'player:%s\n' "$*" >> "$ROBOT_TEST_LOG"
[[ ${ROBOT_PLAYER_FAIL:-0} != 1 ]]
EOF
for action in disable enable; do
    cat > "$tmp/controller/$action" <<'EOF'
#!/usr/bin/env bash
printf 'controller:%s\n' "$(basename "$0")" >> "$ROBOT_TEST_LOG"
EOF
    chmod +x "$tmp/controller/$action"
done
cat > "$tmp/bin/camera" <<'EOF'
#!/usr/bin/env bash
printf 'camera:%s\n' "$*" >> "$ROBOT_TEST_LOG"
EOF
chmod +x "$tmp/bin/ssh" "$tmp/bin/player" "$tmp/bin/camera"
export ROBOT_CAMERA_BIN="$tmp/bin/camera"

fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }
assert_log() { grep -Fqx -- "$1" "$tmp/log" || fail "expected $1"; }
expect_failure() { if "$@" >"$tmp/out" 2>&1; then fail "unexpected success: $*"; fi; }

"$root/bin/robot" --help >"$tmp/out"
grep -q 'robot controller' "$tmp/out" || fail 'help is missing controller command'
: > "$tmp/log"
"$root/bin/robot" controller stop
"$root/bin/robot" controller start
assert_log 'controller:disable'
assert_log 'controller:enable'
: > "$tmp/log"
"$root/bin/robot" position
assert_log 'player:192.0.2.82 position'
"$root/bin/robot" fan
assert_log 'player:192.0.2.82 fan'
"$root/bin/robot" snapshot "$tmp/frame.jpg"
assert_log "camera:$tmp/frame.jpg"
: > "$tmp/log"
"$root/bin/robot" move forward 0.5
assert_log 'player:192.0.2.82 move forward 0.5'
assert_log 'ssh:robot rm -f -- /tmp/robot-drive-guard.TEST'
[[ $(grep -n 'ssh:robot bash -s' "$tmp/log" | tail -1 | cut -d: -f1) -lt $(grep -n 'player:.* move ' "$tmp/log" | cut -d: -f1) ]] || fail 'guard was not armed before movement'
: > "$tmp/log"
ROBOT_TEST_CONTROLLER_STATE=enabled expect_failure "$root/bin/robot" move forward 0.5
if grep -q 'player:.* move ' "$tmp/log"; then fail 'default move bypassed controller gate'; fi
: > "$tmp/log"
ROBOT_TEST_CONTROLLER_STATE=enabled "$root/bin/robot" move --with-controller forward 0.5
assert_log 'player:192.0.2.82 move forward 0.5'
assert_log 'ssh:robot rm -f -- /tmp/robot-drive-guard.TEST'
: > "$tmp/log"
ROBOT_PLAYER_FAIL=1 expect_failure "$root/bin/robot" move forward 0.1
if grep -q 'ssh:robot rm -f -- /tmp/robot-drive-guard.TEST' "$tmp/log"; then fail 'guard was disarmed after failed movement'; fi
: > "$tmp/log"
for args in 'move sideways 1' 'move forward 0' 'move forward 3' 'move forward nan' 'move forward -1' 'move forward 1 extra' 'move --with-controller forward 3' 'move --with-controller sideways 0.5' 'position extra' 'fan extra' 'controller unknown'; do
    read -r -a argv <<< "$args"
    expect_failure "$root/bin/robot" "${argv[@]}"
done
[[ ! -s $tmp/log ]] || fail 'invalid commands contacted robot'
"$root/bin/robot" stop
assert_log 'player:192.0.2.82 stop'
printf 'robot CLI tests passed\n'

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
"$root/bin/controller/disable"
grep -q '^# ROBOT_DISABLED: RoboController,setsid RoboController&,1,3,0$' "$ROBOT_PROCESS_LIST" || fail 'watchdog entry not disabled'
[[ ! -e $ROBOT_TEST_STATE ]] || fail 'controller still running'
"$root/bin/controller/disable"
"$root/bin/controller/enable"
grep -q '^RoboController,setsid RoboController&,1,3,0$' "$ROBOT_PROCESS_LIST" || fail 'watchdog entry not restored'
[[ -e $ROBOT_TEST_STATE ]] || fail 'controller did not return'
"$root/bin/controller/enable"
[[ ! -e $ROBOT_BACKUP ]] || fail 'recovery backup left after successful restore'

sed -i 's/RoboController,setsid RoboController\&,1,3,0/RoboController,unexpected/' "$ROBOT_PROCESS_LIST"
expect_failure "$root/bin/controller/disable"
grep -q '^RoboController,unexpected$' "$ROBOT_PROCESS_LIST" || fail 'unexpected entry was overwritten'

printf 'player,player\nRoboController,setsid RoboController&,1,3,0\nAppProxy,app\n' > "$ROBOT_PROCESS_LIST"
touch "$tmp/fail-restart"
ROBOT_FAIL_RESTART="$tmp/fail-restart" expect_failure "$root/bin/controller/disable"
grep -q '^RoboController,setsid RoboController&,1,3,0$' "$ROBOT_PROCESS_LIST" || fail 'entry not restored after failed restart'
[[ -e $ROBOT_BACKUP ]] || fail 'lost recovery copy while restart failed'
rm "$tmp/fail-restart"
"$root/bin/controller/enable"
[[ ! -e $ROBOT_BACKUP ]] || fail 'recovery copy remained after retry'
[[ -e $ROBOT_TEST_STATE ]] || fail 'controller did not recover after retry'

printf 'robot controller tests passed\n'
