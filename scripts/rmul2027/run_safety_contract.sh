#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="$(cd -- "$task_dir/../../../.." && pwd)"
task_build="${1:-$task_ws/artifacts/rmul_2027_3v3/tools/safety-contract}"
source /opt/ros/humble/setup.bash
source "$task_ws/install/setup.bash"
task_tf2="$task_ws/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib"
test -f "$task_tf2/libtf2.so"
export LD_LIBRARY_PATH="$task_tf2:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${RMUL2027_TEST_DOMAIN:-181}"
cmake -S "$task_dir/tests/safety_contract" -B "$task_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$task_build" -j2
"$task_build/kalman_contract"
"$task_build/pose_fresh_contract" "$task_ws/install/pb_nav2_plugins/lib/libpb_pose_fresh_condition_bt_node.so"
