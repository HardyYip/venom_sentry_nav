#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="$(cd -- "$task_dir/../../../.." && pwd)"
task_output="$task_ws/artifacts/rmul_2027_3v3/tools/obstacle_cart_actor"
mkdir -p -- "$(dirname -- "$task_output")"
g++ -std=c++17 -O2 -pthread "$task_dir/obstacle_cart_actor.cpp" -o "$task_output" $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8)
printf '%s\n' "$task_output"
