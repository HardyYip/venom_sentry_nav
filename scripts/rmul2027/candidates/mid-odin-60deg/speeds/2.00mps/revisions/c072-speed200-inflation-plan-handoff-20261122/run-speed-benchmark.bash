#!/usr/bin/env bash
set -euo pipefail
bench=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
profile=$(realpath -- "${1:?expected a profile directory}")
[[ "$(dirname -- "$profile")" == "$(dirname -- "$bench")" ]]
test -f "$profile/params.yaml"
exec 9>/tmp/rmul-fixed-moving-20261120.lock
flock -w 240 9
out=$(mktemp -d "$profile/speed-benchmark-XXXXXX")
printf '%s\n' "$out"
env RMUL_HEADLESS_NVIDIA=1 RMUL_SOFTWARE_THREADS=12 RMUL_NAV_SENSOR_ONLY=1 RMUL_NAV_RENDER_LOD=1     bash /tmp/rmul-jammy/sim_shell.bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    bench=$1
    run=$2
    out=$3
    export RMUL_TEST_ARTIFACT_ROOT="$out" RMUL_STORAGE_STATE_DIR="$out/storage-state"
    export RMUL_MID_ODIN_INITIAL_VIEW=1 RMUL_SENSOR_PROFILE=cad-mid-odin RMUL_MID_ODIN_VIEW=1 RMUL_MID_ODIN_CLEAN_SHUTDOWN=1
    export RMUL_MID_GEOMETRIC_GUARD=1 RMUL_MID_ODIN_C072_FULL=1 RMUL_C072_TRANSFER_ROOT="$run" RMUL_MID_ODIN_C072_COSTMAP=1
    export RMUL_IMU_RECOVERY=1 RMUL_MID_ODIN_C072_LOCAL_PREDICTION=1 RMUL_MID_ODIN_C072_SWEPT_CLEARANCE=1
    export RMUL_ODIN_BOUNDED_SOURCE_TF=1 RMUL_ODIN_PLANAR_RETURNS=1 RMUL_MID_ODIN_SIM_SPEED_MPS=2.00
    scripts=$RMUL_WORKSPACE/src/pb2025_sentry_nav/scripts/rmul2027
    export RMUL2027_DDS_PROFILE="$run/source/fastdds_local_64m.xml"
    source "$scripts/configure_dds.sh"
    bash "$scripts/start_cad_mid_odin_initial_tf_20261121.sh" nav --render-mode gpu       --field-contact cad-flat-floor --gz-world-path "$scripts/config/gz_world_test.yaml"       --params-file "$run/params.yaml" --tf2-library-dir "$RMUL_WORKSPACE/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib" > "$out/launcher.log" 2>&1 &
    launcher=$!
    trap "kill -TERM $launcher 2>/dev/null || true; wait $launcher 2>/dev/null || true" EXIT
    python3 "$scripts/verify_simulation.py" --mode ready --record-dir "$out/ready"       --startup-timeout 110 --stability-seconds 2 > "$out/ready.log" 2>&1
    python3 -u "$bench/speed-benchmark.py" "$out" > "$out/benchmark.log" 2>&1
' speed_benchmark "/evidence/$(basename -- "$bench")" "/evidence/$(basename -- "$profile")"     "/evidence/$(basename -- "$profile")/$(basename -- "$out")"
