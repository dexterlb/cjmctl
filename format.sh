#!/bin/bash

set -euo pipefail

cdir="$(dirname "$(readlink -f "${0}")")"

check=false
if [[ ${1:-} == "--check" ]]; then
    check=true
    shift
fi

if [[ $# -eq 1 ]]; then
    dirs=( "$(readlink -f "${1}")" )
elif [[ $# -eq 0 ]]; then
    dirs=( "${cdir}"/{src,tools/tester} )
else
    echo 'what?' >&2
    exit 1
fi

if $check; then
    clang_args=( --dry-run --Werror )
else
    clang_args=( -i )
fi

cd "${cdir}"

for dir in "${dirs[@]}"; do
    if ! $check; then
        echo "formatting in ${dir}" >&2
    fi
    find "${dir}" -type f -iregex '.*\.[ch]' -not -path '*/sys/*' -exec clang-format "${clang_args[@]}" -style=file '{}' \+
done
