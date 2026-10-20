#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="$(cd -- "$task_dir/../../../.." && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
task_linux_path=
IFS=: read -r -a task_path_entries <<< "$PATH"
for task_entry in "${task_path_entries[@]}"; do
  case "$task_entry" in /mnt/[a-zA-Z]/*) continue ;; esac
  task_linux_path="${task_linux_path:+$task_linux_path:}$task_entry"
done
export PATH="$task_linux_path"
cd "$task_ws"
rosdep check --from-paths src --ignore-src --rosdistro humble
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
colcon build --base-paths src --symlink-install --parallel-workers 2 \
  --packages-up-to pb2025_nav_bringup rmu_gazebo_simulator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DFETCHCONTENT_SOURCE_DIR_SMALL_GICP="$task_ws/deps/small_gicp" "$@"
