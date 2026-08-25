#!/bin/sh
# Fails if any flight algorithm performs an Eigen heap allocation.
#
# Hosted build on purpose: it runs anywhere, and -DEIGEN_ALLOCA makes it
# target-representative. See noHeapCheck.cpp for why two counters are needed.
set -e

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
eigen=${EIGEN3_DIR:-/home/user/eigen}
out=${TMPDIR:-/tmp}/noHeapCheck

[ -d "$eigen/Eigen" ] || { echo "Eigen not found at $eigen; set EIGEN3_DIR" >&2; exit 2; }

${CXX:-g++} -std=gnu++23 -O2 -DNDEBUG \
    -DEIGEN_FREESTANDING=1 \
    -DEIGEN_ALLOCA=probeAlloca \
    -include "$here/probeAlloca.h" \
    -I"$eigen" -I"$root" -I"$root/algorithms" \
    -I"$root/algorithms/inertialFilter" \
    -I"$root/algorithms/sunlineFilter" \
    -I"$root/algorithms/flybyFilter" \
    -I"$root/algorithms/forceTorqueThrForceMapping" \
    "$here/noHeapCheck.cpp" \
    "$root/algorithms/inertialFilter/inertialFilterAlgorithm.cpp" \
    "$root/algorithms/sunlineFilter/sunlineFilterAlgorithm.cpp" \
    "$root/algorithms/flybyFilter/flybyFilterAlgorithm.cpp" \
    "$root/algorithms/forceTorqueThrForceMapping/forceTorqueThrForceMappingAlgorithm.cpp" \
    -o "$out" -Wl,--wrap=malloc,--wrap=free

exec "$out"
