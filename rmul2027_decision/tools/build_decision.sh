#!/usr/bin/env bash
set -eo pipefail
task_ws="${RMUL_WORKSPACE:-/home/hardy/pb2025_nav_ws}"
task_repo="$task_ws/src/pb2025_sentry_nav"
task_package="$task_repo/rmul2027_decision"
task_build="${RMUL_DECISION_BUILD_ROOT:-$task_ws/artifacts/rmul_2027_3v3/decision-C062}"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then exec bash "$task_package/tools/guarded_run.sh" bash "$0"; fi
source /opt/ros/humble/setup.bash
mkdir -p "$task_build"
if [ ! -d "$task_build/BehaviorTree.CPP/.git" ]; then
  git clone --depth 1 --branch 4.6.2 https://github.com/BehaviorTree/BehaviorTree.CPP.git "$task_build/BehaviorTree.CPP"
fi
test "$(git -C "$task_build/BehaviorTree.CPP" rev-parse HEAD)" = 2a8a226fbbd99f524f0796e0d8a3145773c61c06
cmake -S "$task_build/BehaviorTree.CPP" -B "$task_build/bt-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_build/bt4" -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON -DBTCPP_UNIT_TESTS=OFF -DBTCPP_EXAMPLES=OFF -DBTCPP_GROOT_INTERFACE=OFF -DBTCPP_SQLITE_LOGGING=OFF -DBTCPP_BUILD_TOOLS=OFF
cmake --build "$task_build/bt-build" -j2
cmake --install "$task_build/bt-build"
export CMAKE_BUILD_PARALLEL_LEVEL=2
cd "$task_build"
colcon --log-base "$task_build/colcon-log" build --base-paths "$task_package" --build-base "$task_build/build" --install-base "$task_build/install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_PREFIX_PATH="$task_build/bt4;/opt/ros/humble" -Dbehaviortree_cpp_DIR="$task_build/bt4/lib/cmake/behaviortree_cpp" -DCMAKE_BUILD_RPATH="$task_build/bt4/lib" -DCMAKE_INSTALL_RPATH="$task_build/bt4/lib"
colcon --log-base "$task_build/safety-colcon-log" build --base-paths "$task_repo/fake_vel_transform" --build-base "$task_build/safety-build" --install-base "$task_build/safety-install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
colcon --log-base "$task_build/nav-safety-colcon-log" build --base-paths "$task_repo/rmul2027_nav_safety" --build-base "$task_build/nav-safety-build" --install-base "$task_build/nav-safety-install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ctest --test-dir "$task_build/build/rmul2027_decision" --output-on-failure
