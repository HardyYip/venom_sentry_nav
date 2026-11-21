#!/usr/bin/env bash
set -eo pipefail
package=$(cd "$(dirname "$0")" && pwd)
output=$(mktemp -d "$package/trial-XXXXXX")
exec 9>/tmp/rmul-fixed-moving-20261120.lock
flock -n 9
env RMUL_HEADLESS_NVIDIA=1 RMUL_SOFTWARE_THREADS=12 RMUL_NAV_SENSOR_ONLY=1 RMUL_NAV_RENDER_LOD=1 bash /tmp/rmul-jammy/sim_shell.bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    pkg=$1
    out=$2
    run=/evidence/c072-speed200-from-qualified150-20261121
    export RMUL_TEST_ARTIFACT_ROOT="$out" RMUL_STORAGE_STATE_DIR="$out/storage-state"
    export RMUL_MID_ODIN_INITIAL_VIEW=1 RMUL_SENSOR_PROFILE=cad-mid-odin RMUL_MID_ODIN_VIEW=1 RMUL_MID_ODIN_CLEAN_SHUTDOWN=1
    export RMUL_MID_GEOMETRIC_GUARD=1 RMUL_MID_ODIN_C072_FULL=1 RMUL_C072_TRANSFER_ROOT="$run" RMUL_MID_ODIN_C072_COSTMAP=1
    export RMUL_IMU_RECOVERY=1 RMUL_MID_ODIN_C072_LOCAL_PREDICTION=1 RMUL_MID_ODIN_C072_SWEPT_CLEARANCE=1
    export RMUL_ODIN_BOUNDED_SOURCE_TF=1 RMUL_ODIN_PLANAR_RETURNS=1 RMUL_MID_ODIN_SIM_SPEED_MPS=2.00
    scripts=$RMUL_WORKSPACE/src/pb2025_sentry_nav/scripts/rmul2027
    export RMUL2027_DDS_PROFILE="$run/source/fastdds_local_64m.xml"
    source "$scripts/configure_dds.sh"
    bash "$scripts/start_cad_mid_odin_initial_tf_20261121.sh" nav --render-mode gpu --field-contact cad-flat-floor --gz-world-path "$scripts/config/gz_world_test.yaml" --params-file "$run/params.yaml" --tf2-library-dir "$RMUL_WORKSPACE/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib" > "$out/launcher.log" 2>&1 &
    launcher=$!
    trap "kill -TERM $launcher 2>/dev/null || true" EXIT
    python3 "$scripts/verify_simulation.py" --mode ready --record-dir "$out/ready" --startup-timeout 110 --stability-seconds 2 > "$out/ready.log" 2>&1
    python3 -u "$pkg/recall_client.py" "$out" > "$out/operator.log" 2>&1 &
    operator=$!
    python3 -u "$pkg/mission.py" "$out" > "$out/mission.log" 2>&1
    wait "$operator"
' isolated "/evidence/$(basename "$package")" "/evidence/$(basename "$package")/$(basename "$output")"
