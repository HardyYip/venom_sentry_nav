#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="$(cd -- "$task_dir/../../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$task_ws/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2
exec ros2 run rviz2 rviz2 -d "$task_dir/config/interactive-light.rviz" \
  --ros-args -r __ns:=/red_standard_robot1 -r /tf:=tf -r /tf_static:=tf_static -p use_sim_time:=true
