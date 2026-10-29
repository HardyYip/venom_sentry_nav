#!/usr/bin/env bash
set -eo pipefail
W=/home/hardy/pb2025_nav_ws
P="$W/src/pb2025_sentry_nav"
B="${RMUL_DECISION_BUILD_ROOT:-$W/artifacts/rmul_2027_3v3/imu-recovery-C064}"
export RMUL_TEST_ARTIFACT_ROOT="${RMUL_TEST_ARTIFACT_ROOT:-/mnt/e/RMULArtifacts/pb2025_nav_ws-imu-recovery-C064-20261028}"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
 exec python3 "$P/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --state-dir /mnt/e/RMULStorageState/pb2025_nav_ws --artifact-root "$RMUL_TEST_ARTIFACT_ROOT" --observe-root "$B" --observe-root "$W/logs" --profile light -- bash "$0" "$@"
fi
source /opt/ros/humble/setup.bash
source "$W/install/setup.bash"
source "$B/install/local_setup.bash"
source "$P/scripts/rmul2027/configure_dds.sh"
export RMUL_DECISION_BUILD_ROOT="$B"
export OMP_NUM_THREADS=2
exec python3 "$P/rmul2027_decision/tests/recovery_integration.py" "$@"
