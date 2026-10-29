#!/usr/bin/env bash
set -eo pipefail
export RMUL_IMU_RECOVERY=1
export RMUL_NAV_BASE_BUILD_ROOT="${RMUL_NAV_BASE_BUILD_ROOT:-/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/decision-C062}"
exec bash "$(dirname "$0")/start_low_speed.sh" "$@"
