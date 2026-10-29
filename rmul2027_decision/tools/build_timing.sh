#!/usr/bin/env bash
set -eo pipefail
W="/home/hardy/pb2025_nav_ws"
P="$W/src/pb2025_sentry_nav"
B="$W/artifacts/rmul_2027_3v3/timing-C067"
OLD="$W/artifacts/rmul_2027_3v3/decision-C062"
R="${RMUL_TEST_ARTIFACT_ROOT:?Set one fixed bounded artifact root}"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
 exec python3 "$P/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --state-dir /mnt/e/RMULStorageState/timing-C067 --artifact-root "$R" --observe-root "$B" --observe-root "$W/logs" --profile light -- bash "$0"
fi
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2 PYTHONDONTWRITEBYTECODE=1
test -f "$OLD/bt4/lib/cmake/behaviortree_cpp/behaviortree_cppConfig.cmake"
mkdir -p "$B"
cd "$B"
colcon --log-base "$B/colcon-log" build --base-paths "$P/rmul2027_decision" --build-base "$B/build" --install-base "$B/install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release "-Dbehaviortree_cpp_DIR=$OLD/bt4/lib/cmake/behaviortree_cpp" "-DCMAKE_BUILD_RPATH=$OLD/bt4/lib" "-DCMAKE_INSTALL_RPATH=$OLD/bt4/lib"
ctest --test-dir "$B/build/rmul2027_decision" --output-on-failure
