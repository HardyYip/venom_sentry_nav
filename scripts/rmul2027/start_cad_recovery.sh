#!/usr/bin/env bash
set -eo pipefail
W="${RMUL_WORKSPACE:-/home/hardy/pb2025_nav_ws}"
P="$W/src/pb2025_sentry_nav"
B="$W/artifacts/rmul_2027_3v3/cad-dual-mid-C072"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
 R="${RMUL_TEST_ARTIFACT_ROOT:?Set the bounded task root}"
 exec python3 "$P/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --artifact-root "$R" --state-dir "$R/storage-state" --observe-root "$R" --observe-root "$B" --observe-root "$W/logs" --profile light -- bash "$0" "$@"
fi
export RMUL_IMU_RECOVERY=1 RMUL_SENSOR_PROFILE=cad-dual-mid360
export RMUL_RECOVERY_BUILD_ROOT="$B/cad-metric1-runtime"
export RMUL_CAD_RECOVERY_PROFILE=budget-overlay
export RMUL_CAD_GICP_MAX_ITERATIONS="${RMUL_CAD_GICP_MAX_ITERATIONS:-64}"
test -r "$RMUL_RECOVERY_BUILD_ROOT/localization-install/local_setup.bash"
params="$RMUL_ARTIFACT_DIR/cad-recovery-base.yaml"
python3 "$P/scripts/rmul2027/cad_recovery_profile.py" --source "$P/scripts/rmul2027/config/nav2_test.yaml" --output "$params" --max-iterations "$RMUL_CAD_GICP_MAX_ITERATIONS"
exec bash "$P/scripts/rmul2027/start_cad_dual_mid360.sh" --params-file "$params" "$@"
