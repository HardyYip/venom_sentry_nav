#!/usr/bin/env bash
set -eo pipefail
task_ws="${RMUL_WORKSPACE:-/home/hardy/pb2025_nav_ws}"
task_build="${RMUL_DECISION_BUILD_ROOT:-$task_ws/artifacts/rmul_2027_3v3/decision-C062}"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
  exec bash "$task_ws/src/pb2025_sentry_nav/rmul2027_decision/tools/guarded_run.sh" bash "$0" "$@"
fi
source /opt/ros/humble/setup.bash
source "$task_build/install/local_setup.bash"
source "$task_ws/src/pb2025_sentry_nav/scripts/rmul2027/configure_dds.sh"
exec "$task_build/install/rmul2027_decision/lib/rmul2027_decision/decision_node" "$@"
