#!/usr/bin/env bash
# Rebuild current source with the locally installed MRPT, never download it.
set -euo pipefail
base=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
output=
source="$base/adapter.cpp"
while (($#)); do
    case "$1" in
        --output) [[ $# -ge 2 && -n $2 ]] || { printf '%s\n' '--output needs a path' >&2; exit 2; }; output=$2; shift 2 ;;
        --test) source="$base/test_adapter.cpp"; shift ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
    esac
done
if [[ -z $output ]]; then
    if [[ $source == "$base/test_adapter.cpp" ]]; then output="$base/adapter-tests"
    else output="$base/adapter"; fi
fi
prefix="$base/third_party/install"
[[ -d "$prefix/include" && -d "$prefix/lib" ]] || { printf '%s\n' 'Install MRPT first; see README.md' >&2; exit 1; }
includes=(-isystem "$prefix/include")
for d in "$prefix"/include/mrpt/*/include; do includes+=(-isystem "$d"); done
libs=()
for f in "$prefix"/lib/libmrpt-*.so; do
    name=${f##*/lib}
    libs+=("-l${name%.so}")
done
# Absolute RPATH also makes --output /tmp/... work with transitive libraries.
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra "${includes[@]}" "$source" -o "$output" \
    -L "$prefix/lib" "${libs[@]}" -Wl,--disable-new-dtags -Wl,-rpath,"$prefix/lib"
