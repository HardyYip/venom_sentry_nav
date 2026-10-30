#!/usr/bin/env bash
set -eo pipefail
W="/home/hardy/pb2025_nav_ws"
P="$W/src/pb2025_sentry_nav"
R="${RMUL_TEST_ARTIFACT_ROOT:?Set the fixed C070 artifact root}"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
  exec python3 "$P/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --artifact-root "$R" --state-dir "$R/storage-state" --observe-root "$R" --observe-root "$W/logs" --profile light -- bash "$0" "$@"
fi
source /opt/ros/humble/setup.bash
source "$W/install/setup.bash"
source "$W/artifacts/rmul_2027_3v3/timing-C067/install/local_setup.bash"
source "$P/scripts/rmul2027/configure_dds.sh"
export RMUL_SENSOR_PROFILE=dual-mid360 RMUL_DECISION_BUILD_ROOT="$W/artifacts/rmul_2027_3v3/timing-C067"
export RMUL_NAV_BASE_BUILD_ROOT="$W/artifacts/rmul_2027_3v3/decision-C062"
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2
kind="$1"
shift
case "$kind" in
  long) exec python3 "$P/rmul2027_decision/tests/dual_mid_crossfield_integration.py" "$@" ;;
  low) exec python3 "$P/rmul2027_decision/tests/low_speed_integration.py" "$@" ;;
  imu) exec python3 "$P/rmul2027_decision/tests/recovery_integration.py" "$@" ;;
  timing) exec python3 "$P/rmul2027_decision/tests/timing_integration.py" --mode cycle --fault-min-distance 1.2 "$@" ;;
  persistent) exec python3 "$P/rmul2027_decision/tests/timing_integration.py" --mode persistent "$@" ;;
  fixed) exec python3 "$P/scripts/rmul2027/run_fixed_obstacle_acceptance.py" --test-decision-permit --launcher "$P/scripts/rmul2027/start_dual_mid360.sh" --params-file "$R/preflight-profile/nav2-dual-mid.yaml" "$@" ;;
  moving) exec python3 "$P/scripts/rmul2027/run_moving_obstacle_acceptance.py" --test-decision-permit --launcher "$P/scripts/rmul2027/start_dual_mid360.sh" --params-file "$R/preflight-profile/nav2-dual-mid.yaml" "$@" ;;
  *) printf 'Unknown test kind: %s\n' "$kind" >&2; exit 2 ;;
esac
