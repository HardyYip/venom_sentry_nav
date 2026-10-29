#!/usr/bin/env bash
set -eo pipefail
W=/home/hardy/pb2025_nav_ws
P="$W/src/pb2025_sentry_nav"
B="$W/artifacts/rmul_2027_3v3/imu-recovery-C064"
OLD="$W/artifacts/rmul_2027_3v3/decision-C062"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
 exec python3 "$P/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --state-dir /mnt/e/RMULStorageState/pb2025_nav_ws --artifact-root /mnt/e/RMULArtifacts/pb2025_nav_ws-imu-recovery-C064-20261028 --observe-root "$B" --profile light -- bash "$0"
fi
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
cd "$B"
colcon --log-base "$B/decision-colcon-log" build --base-paths "$P/rmul2027_decision" --build-base "$B/build" --install-base "$B/install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release "-Dbehaviortree_cpp_DIR=$OLD/bt4/lib/cmake/behaviortree_cpp" "-DCMAKE_BUILD_RPATH=$OLD/bt4/lib" "-DCMAKE_INSTALL_RPATH=$OLD/bt4/lib"
colcon --log-base "$B/safety-colcon-log" build --base-paths "$P/fake_vel_transform" --build-base "$B/safety-build" --install-base "$B/safety-install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$W/install/local_setup.bash"
colcon --log-base "$B/localization-colcon-log" build --base-paths "$P/small_gicp_relocalization" --build-base "$B/localization-build" --install-base "$B/localization-install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "-DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=$W/deps/small_gicp" -DFETCHCONTENT_FULLY_DISCONNECTED=ON -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
ctest --test-dir "$B/build/rmul2027_decision" --output-on-failure
