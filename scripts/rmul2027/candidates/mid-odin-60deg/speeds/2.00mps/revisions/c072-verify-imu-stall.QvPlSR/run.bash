#!/usr/bin/env bash
set -euo pipefail
entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec 9>/tmp/rmul-fixed-moving-20261120.lock
flock -n 9
export RMUL_HEADLESS_NVIDIA=1 RMUL_SOFTWARE_THREADS=12 RMUL_NAV_SENSOR_ONLY=1 RMUL_NAV_RENDER_LOD=1
exec bash "$entry/sim-shell.bash" -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    out=$1
    run=/evidence/c072-verify-imu-stall.QvPlSR
    export RMUL_TEST_ARTIFACT_ROOT="$out" RMUL_STORAGE_STATE_DIR="$out/storage-state"
    export RMUL_MID_ODIN_INITIAL_VIEW=1 RMUL_SENSOR_PROFILE=cad-mid-odin RMUL_MID_ODIN_VIEW=1 RMUL_MID_ODIN_CLEAN_SHUTDOWN=1
    export RMUL_MID_GEOMETRIC_GUARD=1 RMUL_MID_ODIN_C072_FULL=1 RMUL_C072_TRANSFER_ROOT="$run" RMUL_MID_ODIN_C072_COSTMAP=1
    export RMUL_IMU_RECOVERY=1 RMUL_MID_ODIN_C072_LOCAL_PREDICTION=1 RMUL_MID_ODIN_C072_SWEPT_CLEARANCE=1
    export RMUL_ODIN_BOUNDED_SOURCE_TF=1 RMUL_ODIN_PLANAR_RETURNS=1 RMUL_MID_ODIN_SIM_SPEED_MPS=2.00
    scripts=$RMUL_WORKSPACE/src/pb2025_sentry_nav/scripts/rmul2027
    export RMUL2027_DDS_PROFILE="$run/source/fastdds_local_64m.xml"
    source "$scripts/configure_dds.sh"
    python3 - "$out" "$run" <<"PY"
import json, sys
from pathlib import Path
import yaml
out, run = map(Path, sys.argv[1:])
x, y = json.loads((out / "routes.json").read_text())["start"]
world = yaml.safe_load((out / "manual-world.yaml").read_text())
world["robots"][world["world"]][0].update(x_pose=str(x-4.8), y_pose=str(y+3.0))
(out / "world.yaml").write_text(yaml.safe_dump(world, sort_keys=False))
params = yaml.safe_load((run / "params.yaml").read_text())
params["small_gicp_relocalization"]["ros__parameters"]["init_pose"][:2] = [x, y]
(out / "fixture-params.yaml").write_text(yaml.safe_dump(params, sort_keys=False))
PY
    bash "$scripts/start_cad_mid_odin_initial_tf_20261121.sh" nav --render-mode gpu \
        --field-contact cad-flat-floor --gz-world-path "$out/world.yaml" \
        --params-file "$out/fixture-params.yaml" \
        --tf2-library-dir "$RMUL_WORKSPACE/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib" > "$out/launcher.log" 2>&1 &
    simulation=$!
    trap "kill -TERM $simulation 2>/dev/null || true; wait $simulation 2>/dev/null || true" EXIT
    trap "exit 130" INT TERM
    python3 "$scripts/verify_simulation.py" --mode ready --record-dir "$out/ready" \
        --startup-timeout 110 --stability-seconds 2 > "$out/ready.log" 2>&1
    request="sdf_filename: \"$out/fixed-obstacles.sdf\", allow_renaming: false"
    ign service -s /world/default/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 6000 --req "$request" > "$out/fixed-obstacles-create.log"
    grep -q "data: true" "$out/fixed-obstacles-create.log"
    ign model -m c020_opponent_cart --link > "$out/fixed-obstacles-links.log"
    ign topic -l > "$out/gazebo-topics.log"
    for attempt in {1..180}; do
        [[ -f "$out/benchmark.ready" ]] && break
        sleep 1
    done
    test -f "$out/benchmark.ready"
    python3 -u "$out/benchmark.py" "$out" > "$out/benchmark.log" 2>&1
' rmul_long_routes "/evidence/$(basename -- "$entry")"
