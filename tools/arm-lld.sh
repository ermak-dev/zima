#!/bin/bash
# rustc -> lld shim for armv7-unknown-linux-musleabihf: strips cc-driver
# flags a bare ld-flavor lld rejects (-Wl, prefixes, -nostartfiles,
# -nodefaultlibs). Used via -C linker in crates/.cargo/config.toml.
# lld is resolved at runtime from the active rustup toolchain if
# tools/.rust-lld-path is absent or empty.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
lld=""
if [[ -s "$here/.rust-lld-path" ]]; then
    lld=$(cat "$here/.rust-lld-path")
fi
if [[ -z $lld || ! -x $lld ]]; then
    if command -v rustup >/dev/null 2>&1; then
        sysroot=$(rustc --print sysroot)
        lld=$sysroot/lib/rustlib/x86_64-unknown-linux-gnu/bin/rust-lld
    fi
fi
if [[ -z $lld || ! -x $lld ]]; then
    printf 'arm-lld.sh: rust-lld not found; create tools/.rust-lld-path with its absolute path\n' >&2
    exit 1
fi
out=()
for a in "$@"; do
    case $a in
        -Wl,*) out+=("${a#-Wl,}") ;;
        -nostartfiles|-nodefaultlibs) ;;
        *) out+=("$a") ;;
    esac
done
exec "$lld" -flavor ld "${out[@]}"
