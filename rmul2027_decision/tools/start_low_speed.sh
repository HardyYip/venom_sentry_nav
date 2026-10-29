#!/usr/bin/env bash
set -eo pipefail
task_ws="${RMUL_WORKSPACE:-/home/hardy/pb2025_nav_ws}"
task_repo="$task_ws/src/pb2025_sentry_nav"
task_build="${RMUL_NAV_BASE_BUILD_ROOT:-${RMUL_DECISION_BUILD_ROOT:-$task_ws/artifacts/rmul_2027_3v3/decision-C062}}"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
  if [ -z "${RMUL_TEST_ARTIFACT_ROOT:-}" ]; then printf 'Set RMUL_TEST_ARTIFACT_ROOT to the bounded decision session root\n' >&2; exit 2; fi
  exec python3 "$task_repo/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --artifact-root "$RMUL_TEST_ARTIFACT_ROOT" --observe-root "$task_ws/logs" --profile light -- bash "$0" "$@"
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$task_ws/install/setup.bash"
source "$task_build/safety-install/local_setup.bash"
source "$task_build/nav-safety-install/local_setup.bash"
if [ "${RMUL_IMU_RECOVERY:-0}" = 1 ]; then
  task_recovery="${RMUL_RECOVERY_BUILD_ROOT:-$task_ws/artifacts/rmul_2027_3v3/imu-recovery-C064}"
  source "$task_recovery/safety-install/local_setup.bash"
  source "$task_recovery/localization-install/local_setup.bash"
fi
source "$task_repo/scripts/rmul2027/configure_dds.sh"
task_params="$RMUL_ARTIFACT_DIR/nav2-c052-decision.yaml"
python3 - "$task_repo/scripts/rmul2027/config/nav2_test.yaml" "$task_params" <<'PY'
from pathlib import Path
import sys,yaml,os
params=yaml.safe_load(Path(sys.argv[1]).read_text())
# C063: distance-based steps omit braking motion at zero/low target speed.
# Match the integration/output step to the actual controller period.
controller=params['controller_server']['ros__parameters']
controller['FollowPath']['discretize_by_time']=True
controller['FollowPath']['time_granularity']=1.0/controller['controller_frequency']
# Simulation-specific filter: source-stamped ~3cm floor returns are not obstacles.
# Original C052 footprints, padding, critics and speed limits remain authoritative.
for name in ('local_costmap','global_costmap'):
 params[name][name]['ros__parameters']['rplidar_obstacle_layer']['rplidar_cloud']['min_obstacle_height']=0.06
params['fake_vel_transform']['ros__parameters']['decision_stop_timeout']=0.3
names=params['bt_navigator']['ros__parameters']['plugin_lib_names']
assert names.count('pb_pose_fresh_condition_bt_node')==1
params['bt_navigator']['ros__parameters']['plugin_lib_names']=[
 'rmul_pose_fresh_condition_bt_node' if n=='pb_pose_fresh_condition_bt_node' else n for n in names]
if os.environ.get('RMUL_IMU_RECOVERY')=='1':
 params['fake_vel_transform']['ros__parameters']['recovery_stop_timeout']=0.3
 params['point_lio']['ros__parameters']['max_imu_gap']=0.1
 if os.environ.get('RMUL_RECOVERY_IMU_TOPIC'):
  params['point_lio']['ros__parameters']['common']['imu_topic']=os.environ['RMUL_RECOVERY_IMU_TOPIC']
Path(sys.argv[2]).write_text(yaml.safe_dump(params,sort_keys=False))
PY
task_recovery_args=()
if [ "${RMUL_IMU_RECOVERY:-0}" = 1 ]; then task_recovery_args+=(--controlled-imu-recovery); fi
exec python3 "$task_repo/scripts/rmul2027/run_simulation.py" nav "${task_recovery_args[@]}" --render-mode auto --field-contact cad-flat-floor --gz-world-path "$task_repo/scripts/rmul2027/config/gz_world_test.yaml" --params-file "$task_params" --tf2-library-dir "$task_ws/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib"
