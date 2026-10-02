#!/usr/bin/env bash
# FEATURE: Fetch one authenticated OpenIPC JPEG without partial or overwritten files.
# SPEC: Private credentials and output, bounded HTTP request, validated JPEG, no motion.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/bin" "$tmp/config" "$tmp/output"
export ROBOT_CAMERA_NETRC="$tmp/config/camera.netrc" ROBOT_CURL="$tmp/bin/curl" ROBOT_TEST_LOG="$tmp/log"
printf 'machine cam login example password fake\n' > "$ROBOT_CAMERA_NETRC"
chmod 600 "$ROBOT_CAMERA_NETRC"
cat > "$tmp/bin/curl" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$ROBOT_TEST_LOG"
out=
while (($#)); do
    if [[ $1 == --output ]]; then out=$2; shift 2; else shift; fi
done
[[ -n $out ]] || exit 2
case ${ROBOT_FAKE_RESPONSE:-good} in
    auth|timeout) exit 22 ;;
    bad) printf 'not a jpeg' > "$out" ;;
    large) python3 - "$out" <<'PY'
import sys
with open(sys.argv[1], 'wb') as f: f.write(b'\xff\xd8' + b'0' * 6000000 + b'\xff\xd9')
PY
        ;;
    *) python3 - "$out" <<'PY'
import sys
with open(sys.argv[1], 'wb') as f: f.write(b'\xff\xd8' + b'0' * 1024 + b'\xff\xd9')
PY
        ;;
esac
EOF
chmod +x "$tmp/bin/curl"
fail() { echo "FAIL: $*" >&2; exit 1; }
expect_failure() { if "$@" >"$tmp/stdout" 2>"$tmp/stderr"; then fail "unexpected success: $*"; fi; }
command="$root/bin/camera/snapshot"

output="$tmp/output/frame.jpg"
"$command" "$output" > "$tmp/stdout"
[[ -s $output ]] || fail 'JPEG was not saved'
[[ $(stat -c %a "$output") == 600 ]] || fail 'JPEG permissions are not private'
grep -Fq "$output" "$tmp/stdout" || fail 'output path was not reported'
grep -q -- '--netrc-file' "$ROBOT_TEST_LOG" || fail 'netrc was not passed'
grep -q -- '--noproxy cam' "$ROBOT_TEST_LOG" || fail 'camera traffic could use a proxy'
expect_failure "$command" "$output"

rm "$output"; chmod 644 "$ROBOT_CAMERA_NETRC"
: > "$ROBOT_TEST_LOG"
expect_failure "$command" "$output"
[[ ! -s $ROBOT_TEST_LOG ]] || fail 'insecure credentials triggered HTTP'
chmod 600 "$ROBOT_CAMERA_NETRC"; rm "$ROBOT_CAMERA_NETRC"
expect_failure "$command" "$output"
[[ ! -s $ROBOT_TEST_LOG ]] || fail 'missing credentials triggered HTTP'
printf 'machine cam login example password fake\n' > "$ROBOT_CAMERA_NETRC"; chmod 600 "$ROBOT_CAMERA_NETRC"

for response in auth timeout bad large; do
    ROBOT_FAKE_RESPONSE="$response" expect_failure "$command" "$output"
    [[ ! -e $output ]] || fail "$response left a partial JPEG"
    [[ -z $(find "$tmp/output" -type f -print -quit) ]] || fail "$response left a temp file"
done

printf 'camera snapshot tests passed\n'
