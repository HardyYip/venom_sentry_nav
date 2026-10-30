#!/usr/bin/env bash
set -eo pipefail
W="${RMUL_WORKSPACE:-/home/hardy/pb2025_nav_ws}"
P="$W/src/pb2025_sentry_nav"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
  exec bash "$P/rmul2027_decision/tools/guarded_run.sh" bash "$0" "$@"
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$W/install/setup.bash"
B="$W/artifacts/rmul_2027_3v3/decision-C062"
source "$B/safety-install/local_setup.bash"
source "$B/nav-safety-install/local_setup.bash"
extra=()
if [ "${RMUL_IMU_RECOVERY:-0}" = 1 ]; then
  RB="${RMUL_RECOVERY_BUILD_ROOT:-$W/artifacts/rmul_2027_3v3/imu-recovery-C064}"
  source "$RB/safety-install/local_setup.bash"
  source "$RB/localization-install/local_setup.bash"
  extra+=(--controlled-imu-recovery)
fi
source "$P/scripts/rmul2027/configure_dds.sh"
input="$P/scripts/rmul2027/config/nav2_test.yaml"
args=("$@")
filtered=()
for ((i=0;i<${#args[@]};i++)); do
  if [ "${args[i]}" = --params-file ]; then
    ((i+=1)); input="${args[i]}"
  else
    filtered+=("${args[i]}")
  fi
done
profile="$(mktemp -d "$RMUL_ARTIFACT_DIR/dual-mid-XXXXXX")"
rmdir "$profile"
python3 "$P/scripts/rmul2027/dual_mid360_profile.py" --output-root "$profile" --params-file "$input" > "$profile.json"
export AMENT_PREFIX_PATH="$profile/prefix:$AMENT_PREFIX_PATH"
export IGN_GAZEBO_RESOURCE_PATH="$profile/prefix/share/pb2025_robot_description/resource/models:${IGN_GAZEBO_RESOURCE_PATH:-}"
export RMUL_SENSOR_PROFILE_MANIFEST="$profile/profile.json"
if [ "${#filtered[@]}" = 0 ]; then
  filtered=(nav --render-mode auto --field-contact cad-flat-floor --gz-world-path "$P/scripts/rmul2027/config/gz_world_test.yaml" --tf2-library-dir "$W/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib")
fi
exec python3 "$P/scripts/rmul2027/run_simulation.py" "${filtered[@]}" "${extra[@]}" --params-file "$profile/nav2-dual-mid.yaml"
