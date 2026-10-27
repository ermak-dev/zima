#!/bin/bash
# rustc -> lld shim for armv7-unknown-linux-musleabihf: strips cc-driver
# flags a bare ld-flavor lld rejects (-Wl, prefixes, -nostartfiles,
# -nodefaultlibs). Used via -C linker in crates/.cargo/config.toml.
lld="$(cat "$(dirname "$0")/.rust-lld-path")"
out=()
for a in "$@"; do
    case $a in
        -Wl,*) out+=("${a#-Wl,}") ;;
        -nostartfiles|-nodefaultlibs) ;;
        *) out+=("$a") ;;
    esac
done
exec "$lld" -flavor ld "${out[@]}"
