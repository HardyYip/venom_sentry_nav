#!/usr/bin/env bash
# Build only the official navigation and its companion simulator, not old Venom.
set -eo pipefail
task_ws=/home/hardy/pb2025_nav_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
cd "$task_ws"
task_patches="$task_ws/src/pb2025_sentry_nav/scripts/venom/patches"
apply_compat() {
  if git -C "$1" apply --reverse --check "$2" 2>/dev/null; then
    return
  fi
  git -C "$1" apply --check "$2"
  git -C "$1" apply "$2"
}
apply_compat "$task_ws/src/rmoss_gazebo" "$task_patches/rmoss_gazebo.patch"
apply_compat "$task_ws/src/rmu_gazebo_simulator" "$task_patches/rmu_gazebo_simulator.patch"
rosdep check --from-paths src --ignore-src --rosdistro humble
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
colcon build --base-paths src --symlink-install --parallel-workers 2 \
  --packages-up-to pb2025_nav_bringup rmu_gazebo_simulator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DFETCHCONTENT_SOURCE_DIR_SMALL_GICP="$task_ws/deps/small_gicp"
