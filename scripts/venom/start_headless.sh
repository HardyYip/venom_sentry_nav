#!/usr/bin/env bash
# New navigation only; all paths are in the D-drive WSL virtual disk.
set -eo pipefail
task_ws=/home/hardy/pb2025_nav_ws
task_root="$task_ws/src/pb2025_sentry_nav"
task_mode="${1:-nav}"
case "$task_mode" in
  nav) task_slam=False ;;
  mapping) task_slam=True ;;
  *) printf 'Usage: bash %s [nav|mapping]\n' "$0" >&2; exit 2 ;;
esac
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$task_ws/install/setup.bash"
task_pcd="$task_root/pb2025_nav_bringup/pcd/simulation/rmuc_2025.pcd"
if [ "$task_mode" = nav ] && [ ! -s "$task_pcd" ]; then
  printf 'Missing official prior PCD: %s\n' "$task_pcd" >&2
  exit 1
fi
mkdir -p "$task_ws/logs"
exec 9>"$task_ws/.headless.lock"
if ! flock -n 9; then
  printf 'A headless test is already running in %s. Stop it first.\n' "$task_ws" >&2
  exit 1
fi
task_run="$task_ws/logs/$(date +%Y%m%d-%H%M%S)-$task_mode"
mkdir "$task_run"
export ROS_LOG_DIR="$task_run/ros"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
task_sim_pid=
task_nav_pid=
cleanup() {
  trap - EXIT INT TERM
  python3 "$task_root/scripts/venom/stop_launches.py" ${task_nav_pid:+"$task_nav_pid"} ${task_sim_pid:+"$task_sim_pid"}
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
setsid ros2 launch "$task_root/scripts/venom/headless_sim.launch.py" \
  > "$task_run/simulator.log" 2>&1 &
task_sim_pid=$!
setsid ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 namespace:=red_standard_robot1 \
  slam:="$task_slam" use_rviz:=False use_sim_time:=True \
  > "$task_run/navigation.log" 2>&1 &
task_nav_pid=$!
printf 'Headless %s started. Logs: %s\n' "$task_mode" "$task_run"
printf 'No RViz or Gazebo GUI. Ctrl+C stops both launch groups.\n'
wait -n "$task_sim_pid" "$task_nav_pid"
