#!/usr/bin/env bash
set -eo pipefail
W="/home/hardy/pb2025_nav_ws"
P="$W/src/pb2025_sentry_nav"
B="$W/artifacts/rmul_2027_3v3/timing-C067"
R="${RMUL_TEST_ARTIFACT_ROOT:?Set one fixed bounded artifact root}"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
 exec python3 "$P/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --state-dir /mnt/e/RMULStorageState/timing-C067 --artifact-root "$R" --observe-root "$B" --observe-root "$W/logs" --profile light -- bash "$0" "$@"
fi
source /opt/ros/humble/setup.bash
source "$W/install/setup.bash"
source "$B/install/local_setup.bash"
source "$P/scripts/rmul2027/configure_dds.sh"
export RMUL_DECISION_BUILD_ROOT="$B" OMP_NUM_THREADS=2 PYTHONDONTWRITEBYTECODE=1
exec python3 -B "$P/rmul2027_decision/tests/timing_integration.py" "$@"
