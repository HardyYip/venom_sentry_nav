#!/bin/bash
set -eo pipefail
mkdir -p /trial/probe
python3 /trial/scene.py /trial/probe /baseline/fixed-left/cases/left-near-01
ign gazebo -s -r --headless-rendering /trial/probe/world.sdf > /trial/probe/gazebo.log 2>&1 &
sim=$!
trap 'kill -TERM "$sim" 2>/dev/null || true' EXIT
for n in $(seq 1 30); do
    if ign topic -l 2>/dev/null | grep -q '^/odin/scan/points$'; then break; fi
    sleep 1
done
ign topic -l > /trial/probe/topics
timeout 12 ign topic -e -t /cart/odom -n 1 > /trial/probe/odom
timeout 12 ign topic -e -t /odin/scan/points -n 1 | head -c 300 > /trial/probe/points || test -s /trial/probe/points
ros2 pkg prefix nav2_controller
ldd /baseline/planner-install/rmul_c072_planner/lib/libc072_navigation_plugins.so | grep 'not found' || true
