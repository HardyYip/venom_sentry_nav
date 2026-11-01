#!/usr/bin/env bash
set -eo pipefail
W="${RMUL_WORKSPACE:-/home/hardy/pb2025_nav_ws}"
P="$W/src/pb2025_sentry_nav"
B="$W/artifacts/rmul_2027_3v3/cad-dual-mid-C072"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
  R="${RMUL_TEST_ARTIFACT_ROOT:?Set the fixed task artifact root}"
  exec python3 "$P/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --artifact-root "$R" --state-dir "$R/storage-state" --observe-root "$R" --observe-root "$B" --observe-root "$W/logs" --profile light -- bash "$0" "$@"
fi
source /opt/ros/humble/setup.bash
source "$W/install/setup.bash"
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 OMP_NUM_THREADS=2 PYTHONDONTWRITEBYTECODE=1
copy="$B/controller-trajectory4-source/pb_omni_pid_pursuit_controller"
if [ -d "$copy" ]; then
  python3 "$P/scripts/rmul2027/prepare_cad_controller.py" --output-dir "$copy" --verify-only
else
  python3 "$P/scripts/rmul2027/prepare_cad_controller.py" --output-dir "$copy"
fi
colcon --log-base "$B/controller-trajectory4-colcon-log" build --base-paths "$copy" --build-base "$B/controller-trajectory4-build" --install-base "$B/controller-trajectory4-install" --executor sequential --allow-overriding pb_omni_pid_pursuit_controller --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
ctest --test-dir "$B/controller-trajectory4-build/pb_omni_pid_pursuit_controller" -R '^test_(pose_projection|geodesic_goal_field|planning_clearance|goal_field_cell_coverage)$' --output-on-failure
