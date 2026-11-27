#!/bin/bash
set -eo pipefail
out=/tmp/manual
cd "$out"
pids=()
cleanup() {
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
start() { local log=$1; shift; "$@" > "$log" 2>&1 & pids+=("$!"); }
start gazebo.log ign gazebo -s -r --headless-rendering "$out/world.sdf"
start bridge.log /opt/ros/humble/lib/ros_gz_bridge/parameter_bridge \
 '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
 '/cart/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
 '/box/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
 '/cart/cmd@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
 '/box/cmd@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
 '/world/navigation/model/cart/link/body/sensor/contacts/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts' \
 '/odin/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked' \
 '/mid/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
start planner.log /opt/ros/humble/lib/nav2_planner/planner_server --ros-args --params-file "$out/nav.yaml"
start controller.log /opt/ros/humble/lib/nav2_controller/controller_server --ros-args --params-file "$out/nav.yaml" -r cmd_vel:=nav_cmd
start smoother.log /baseline/planner-install/nav2_velocity_smoother/lib/nav2_velocity_smoother/velocity_smoother --ros-args --params-file "$out/nav.yaml" -r cmd_vel:=profiled_nav_cmd -r cmd_vel_smoothed:=/cart/cmd
start bt.log /opt/ros/humble/lib/nav2_bt_navigator/bt_navigator --ros-args --params-file "$out/nav.yaml"
start behaviors.log /opt/ros/humble/lib/nav2_behaviors/behavior_server --ros-args --params-file "$out/nav.yaml" -r cmd_vel:=nav_cmd
start lifecycle.log /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -p autostart:=true -p 'node_names:=[planner_server, controller_server, velocity_smoother, behavior_server, bt_navigator]' -p use_sim_time:=true
start fixture.log python3 -u /tmp/manual/manual_fixture.py /tmp/manual
fixture_pid=$!
for attempt in {1..800}; do
    if grep -q '^MANUAL_READY$' /tmp/manual/fixture.log; then break; fi
    kill -0 "$fixture_pid" || exit 1
    sleep .1
done
grep -q '^MANUAL_READY$' /tmp/manual/fixture.log
mkdir -m 700 -p /tmp/rviz-runtime
start rviz.log env AMENT_PREFIX_PATH="/tmp/viewer:$AMENT_PREFIX_PATH" DISPLAY="$CART_DISPLAY" XAUTHORITY=/tmp/cart-xauth QT_QPA_PLATFORM=xcb QT_X11_NO_MITSHM=1 XDG_RUNTIME_DIR=/tmp/rviz-runtime LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LP_NUM_THREADS=2 /tmp/viewer/lib/rviz2/rviz2 -d /tmp/manual/cart-body.rviz --ros-args -p use_sim_time:=true
viewer_pid=$!
(
    sleep 4
    ros2 action info /navigate_to_pose > /tmp/manual/action-info.txt
    ros2 node list > /tmp/manual/nodes.txt
    ros2 topic info /goal_pose -v > /tmp/manual/goal-topic.txt
) > /tmp/manual/readiness.log 2>&1 &
pids+=("$!")
wait -n "$viewer_pid" "$fixture_pid"
