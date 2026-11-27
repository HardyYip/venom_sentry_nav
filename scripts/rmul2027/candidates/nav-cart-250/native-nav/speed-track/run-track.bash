#!/bin/bash
set -eo pipefail
group=$1; name=$2; out=/trial/$3/$name
mkdir -p "$out"
python3 /trial/scene.py "$out" "/trial/$group/cases/$name"
python3 /trial/speed-track/prepare.py "$out"
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
start smoother.log /baseline/planner-install/nav2_velocity_smoother/lib/nav2_velocity_smoother/velocity_smoother --ros-args --params-file "$out/nav.yaml" -r cmd_vel:=nav_cmd -r cmd_vel_smoothed:=/cart/cmd
start bt.log /opt/ros/humble/lib/nav2_bt_navigator/bt_navigator --ros-args --params-file "$out/nav.yaml"
start behaviors.log /opt/ros/humble/lib/nav2_behaviors/behavior_server --ros-args --params-file "$out/nav.yaml" -r cmd_vel:=nav_cmd
start lifecycle.log /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -p autostart:=true -p 'node_names:=[planner_server, controller_server, velocity_smoother, behavior_server, bt_navigator]' -p use_sim_time:=true
timeout 180 python3 -u /trial/speed-track/evaluate-lane.py "$out" > "$out/evaluate.log" 2>&1
