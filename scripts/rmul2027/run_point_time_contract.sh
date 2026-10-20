#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="$(cd -- "$task_dir/../../../.." && pwd)"
task_build="${1:-$task_ws/artifacts/rmul_2027_3v3/tools/point-time-contract}"
source /opt/ros/humble/setup.bash
source "$task_ws/install/setup.bash"
cmake -S "$task_dir/tests/point_time_contract" -B "$task_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$task_build" -j2
ROS_DOMAIN_ID="${RMUL2027_TEST_DOMAIN:-177}" "$task_build/point_time_contract"
