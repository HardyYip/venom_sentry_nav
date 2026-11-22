#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
bash /tmp/rmul-jammy/sim_shell.bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    dir=$1
    cd "$dir"
    MAKEFLAGS="-j4" colcon --log-base "$dir/build-logs" build --parallel-workers 2 \
      --base-paths "$dir/source/rmul_c072_planner" "$dir/source/sensor_scan_generation" "$dir/source/nav2_velocity_smoother" \
      --build-base "$dir/build" --install-base "$dir/planner-install" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
' build-local "/evidence/$(basename -- "$here")"
