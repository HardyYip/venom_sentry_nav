#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
bash /tmp/rmul-jammy/sim_shell.bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    dir=$1
    # Keep FetchContent on the already-staged dependency inside the offline fixture.
    gicp_dependency="$PWD/c072-transfer-source/small_gicp_dependency"
    test -s "$gicp_dependency/CMakeLists.txt"
    test ! -L "$dir/gicp-install" || {
      echo "gicp-install is linked to a prior release. Build the candidate in a fresh copy." >&2
      exit 2
    }
    MAKEFLAGS="-j2" colcon --log-base "$dir/gicp-build-logs" build --executor sequential \
      --base-paths "$dir/source/gicp-recovery" \
      --build-base "$dir/gicp-build" --install-base "$dir/gicp-install" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -DFETCHCONTENT_FULLY_DISCONNECTED=ON "-DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=$gicp_dependency"
    cd "$dir"
    MAKEFLAGS="-j4" colcon --log-base "$dir/build-logs" build --parallel-workers 2 \
      --base-paths "$dir/source/rmul_c072_planner" "$dir/source/sensor_scan_generation" "$dir/source/nav2_velocity_smoother" \
      --build-base "$dir/build" --install-base "$dir/planner-install" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    python3 "$dir/record-build-inputs.py" "$dir"
' build-local "/evidence/$(basename -- "$here")"
