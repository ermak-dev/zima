#!/usr/bin/env bash
# Local build only. Deployment is an explicit, separate operation.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
output=
while (( $# )); do
    case $1 in
        --output) (( $# >= 2 )) || exit 2; output=$2; shift 2 ;;
        *) printf 'usage: build-supervisor.sh [--output PATH]\n' >&2; exit 2 ;;
    esac
done
export CARGO_TARGET_DIR=${CARGO_TARGET_DIR:-$root/crates/target}
# Resolve these before changing directory, including relative --output paths.
CARGO_TARGET_DIR=$(realpath -m "$CARGO_TARGET_DIR")
if [[ -n $output ]]; then output=$(realpath -m "$output"); fi
export CC_armv7_unknown_linux_gnueabihf=$root/tools/arm-gnu-cc.sh
export AR_armv7_unknown_linux_gnueabihf=${AR:-ar}
export CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER=$root/tools/arm-gnu-cc.sh
# Target-specific variables leave the existing static musl configuration intact.
(cd "$root/crates" && cargo build -p mask-owner --release \
    --target armv7-unknown-linux-gnueabihf --bin robot-supervisor)
binary=$CARGO_TARGET_DIR/armv7-unknown-linux-gnueabihf/release/robot-supervisor
if [[ -z $output ]]; then output=$binary; fi
if [[ $output != "$binary" ]]; then
    mkdir -p "$(dirname "$output")"
    cp -- "$binary" "$output"
fi
printf '%s\n' "$output"
