#!/usr/bin/env bash
set -eo pipefail
W="${RMUL_WORKSPACE:-/home/hardy/pb2025_nav_ws}"
P="$W/src/pb2025_sentry_nav"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
export RMUL_SENSOR_PROFILE=cad-mid-odin
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
 R="${RMUL_TEST_ARTIFACT_ROOT:?Set fixed C075 artifact root}"
 exec python3 "$P/scripts/rmul2027/c075_storage_guard.py" --host-root /mnt/d --artifact-root "$R" --state-dir "$R/storage-state" --observe-root "$R" --observe-root "$W/logs" --profile light -- bash "$0" "$@"
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$W/install/setup.bash"
B="$W/artifacts/rmul_2027_3v3/decision-C062"
source "$B/safety-install/local_setup.bash"
source "$B/nav-safety-install/local_setup.bash"
extra=()
if [ "${RMUL_IMU_RECOVERY:-0}" = 1 ]; then
 if [ "${RMUL_MID_ODIN_C072_FULL:-0}" != 1 ]; then
  RB="${RMUL_RECOVERY_BUILD_ROOT:-$W/artifacts/rmul_2027_3v3/imu-recovery-C064}"
  source "$RB/safety-install/local_setup.bash"
  source "$RB/localization-install/local_setup.bash"
 fi
 extra+=(--controlled-imu-recovery)
fi
source "$W/artifacts/rmul_2027_3v3/cad-dual-mid-C072/install/local_setup.bash"
source "$W/artifacts/rmul_2027_3v3/cad-dual-mid-C072/controller-trajectory4-install/local_setup.bash"
if [ "${RMUL_MID_ODIN_C072_FULL:-0}" = 1 ]; then
 source "${RMUL_C072_TRANSFER_ROOT:?C072 build prefix required}/planner-install/local_setup.bash"
 if [ "${RMUL_IMU_RECOVERY:-0}" = 1 ]; then
  source "$RMUL_C072_TRANSFER_ROOT/gicp-install/local_setup.bash"
 fi
fi
if [ "${RMUL_MID_ODIN_VIEW:-0}" = 1 ]; then
 source "$W/artifacts/rmul_2027_3v3/cad-mid-odin-C075/view1-install/local_setup.bash"
 source "$W/artifacts/rmul_2027_3v3/cad-mid-odin-C075/goal-guard1-install/local_setup.bash"
fi
# C075 opt-in private executables. The normal sensing/control algorithms stay unchanged.
if [ "${RMUL_MID_ODIN_CLEAN_SHUTDOWN:-0}" = 1 ]; then
 if [ "${RMUL_MID_ODIN_VIEW:-0}" != 1 ]; then
  printf 'C075 clean shutdown requires the bounded-view profile.\n' >&2
  exit 2
 fi
 source "$W/artifacts/rmul_2027_3v3/cad-mid-odin-C075/shutdown1-install/local_setup.bash"
 source "$W/artifacts/rmul_2027_3v3/cad-mid-odin-C075/base-shutdown2-install/local_setup.bash"
 export AMENT_PREFIX_PATH="$W/artifacts/rmul_2027_3v3/cad-mid-odin-C075/container2-prefix:$AMENT_PREFIX_PATH"
 export AMENT_PREFIX_PATH="$W/artifacts/rmul_2027_3v3/cad-mid-odin-C075/gz-launch1-prefix:$AMENT_PREFIX_PATH"
fi
source "$W/c072-speed-initial-tf-build/nav-safety-install/local_setup.bash"
source "$P/scripts/rmul2027/configure_dds.sh"
input="$P/scripts/rmul2027/config/nav2_test.yaml"
args=("$@"); filtered=()
for ((i=0;i<${#args[@]};i++)); do
 if [ "${args[i]}" = --params-file ]; then
  ((i+=1)); input="${args[i]}"
 else filtered+=("${args[i]}"); fi
done
profile="$(mktemp -d "$RMUL_ARTIFACT_DIR/cad-mid-odin-XXXXXX")"
rmdir "$profile"
if [ "${RMUL_MID_ODIN_C072_FULL:-0}" = 1 ] && [ "${RMUL_IMU_RECOVERY:-0}" = 1 ]; then
 prepared="$profile-recovery.yaml"
 python3 "$P/scripts/rmul2027/cad_recovery_profile.py" --source "$input" --output "$prepared" > "$profile-recovery.json"
 input="$prepared"
fi
python3 "$P/scripts/rmul2027/cad_mid_odin_profile.py" --output-root "$profile" --params-file "$input" > "$profile.json"
if [ "${RMUL_MID_ODIN_CLEAN_SHUTDOWN:-0}" = 1 ]; then
 python3 "$P/scripts/rmul2027/verify_c075_shutdown_install.py" --output "$profile/shutdown-executable-selection.json"
fi
export AMENT_PREFIX_PATH="$profile/prefix:$AMENT_PREFIX_PATH"
export IGN_GAZEBO_RESOURCE_PATH="$profile/prefix/share/pb2025_robot_description/resource/models:${IGN_GAZEBO_RESOURCE_PATH:-}"
export RMUL_SENSOR_PROFILE_MANIFEST="$profile/profile.json"
if [ "${#filtered[@]}" = 0 ]; then
 filtered=(nav --render-mode gpu --field-contact cad-flat-floor --gz-world-path "$P/scripts/rmul2027/config/gz_world_test.yaml" --tf2-library-dir "$W/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib")
fi
exec python3 "$P/scripts/rmul2027/run_simulation.py" "${filtered[@]}" "${extra[@]}" --params-file "$profile/nav2-dual-mid.yaml" --prior-pcd-file "$profile/prior-base-frame.pcd" --map-file "$profile/map-boundary-margin.yaml"
