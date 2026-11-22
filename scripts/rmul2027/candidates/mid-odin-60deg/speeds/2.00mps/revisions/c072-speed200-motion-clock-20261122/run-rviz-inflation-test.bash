#!/usr/bin/env bash
set -euo pipefail
trial=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
x=${1:?map x}; y=${2:?map y}
exec 9>/tmp/rmul-fixed-moving-20261120.lock
flock -w 1200 9
out=$(mktemp -d "$trial/rviz-inflation-XXXXXX")
printf '%s\n' "$out"
env RMUL_HEADLESS_NVIDIA=1 RMUL_SOFTWARE_THREADS=12 RMUL_NAV_SENSOR_ONLY=1 RMUL_NAV_RENDER_LOD=1     bash "$trial/sim-shell.bash" -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    run=$1
    out=$2
    x=$3
    y=$4
    export RMUL_TEST_ARTIFACT_ROOT="$out" RMUL_STORAGE_STATE_DIR="$out/storage-state"
    export RMUL_MID_ODIN_INITIAL_VIEW=1 RMUL_SENSOR_PROFILE=cad-mid-odin RMUL_MID_ODIN_VIEW=1 RMUL_MID_ODIN_CLEAN_SHUTDOWN=1
    export RMUL_MID_GEOMETRIC_GUARD=1 RMUL_MID_ODIN_C072_FULL=1 RMUL_C072_TRANSFER_ROOT="$run" RMUL_MID_ODIN_C072_COSTMAP=1
    export RMUL_IMU_RECOVERY=1 RMUL_MID_ODIN_C072_LOCAL_PREDICTION=1 RMUL_MID_ODIN_C072_SWEPT_CLEARANCE=1
    export RMUL_ODIN_BOUNDED_SOURCE_TF=1 RMUL_ODIN_PLANAR_RETURNS=1 RMUL_MID_ODIN_SIM_SPEED_MPS=2.00
    scripts=$RMUL_WORKSPACE/src/pb2025_sentry_nav/scripts/rmul2027
    export RMUL2027_DDS_PROFILE="$run/source/fastdds_local_64m.xml"
    source "$scripts/configure_dds.sh"
    python3 - "$run" "$out" "$scripts" "$x" "$y" <<"PY"
import sys
from pathlib import Path
import yaml
run,out,scripts=map(Path,sys.argv[1:4])
x,y=map(float,sys.argv[4:])
world=yaml.safe_load((scripts/"config/gz_world_test.yaml").read_text())
robot=world["robots"][world["world"]][0]
robot.update(x_pose=str(x-4.8), y_pose=str(y+3.0))
(out/"world.yaml").write_text(yaml.safe_dump(world,sort_keys=False))
params=yaml.safe_load((run/"params.yaml").read_text())
init=params["small_gicp_relocalization"]["ros__parameters"]["init_pose"]
init[:2]=[x,y]
(out/"fixture-params.yaml").write_text(yaml.safe_dump(params,sort_keys=False))
PY
    bash "$scripts/start_cad_mid_odin_initial_tf_20261121.sh" nav --render-mode gpu       --field-contact cad-flat-floor --gz-world-path "$out/world.yaml"       --params-file "$out/fixture-params.yaml" --tf2-library-dir "$RMUL_WORKSPACE/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib" > "$out/launcher.log" 2>&1 &
    launcher=$!
    trap "kill -TERM $launcher 2>/dev/null || true; wait $launcher 2>/dev/null || true" EXIT
    python3 "$scripts/verify_simulation.py" --mode ready --record-dir "$out/ready"       --startup-timeout 110 --stability-seconds 2 > "$out/ready.log" 2>&1
    python3 -u "$run/rviz-inflation-test.py" "$out" "$x" "$y" > "$out/replay.log" 2>&1
' inflation_test "/evidence/$(basename -- "$trial")" "/evidence/$(basename -- "$trial")/$(basename -- "$out")" "$x" "$y"
