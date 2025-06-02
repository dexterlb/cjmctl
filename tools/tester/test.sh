#!/bin/bash

set -euo pipefail

cdir="$(dirname "$(readlink -f "${0}")")"
build_dir="${cdir}/build"

if [[ ! -d "${build_dir}" ]]; then
    mkdir "${build_dir}"
    cd "${build_dir}"
    cmake "${cdir}" -D CMAKE_BUILD_TYPE=RelWithDebInfo
fi

echo "building"
cd "${build_dir}"
make -j8

"${cdir}"/plotter.py "${cdir}"/sample_pos_vel_data.txt.gz "${build_dir}/tester"
