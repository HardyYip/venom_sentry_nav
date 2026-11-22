#!/usr/bin/env bash
set -euo pipefail
dest=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
role=${1:?Expected test stage}
[[ "$role" =~ ^[a-z0-9-]+$ ]]
shift
render=${RMUL_TEST_RENDER:-gpu}
headless_nvidia=0
case "$render" in
    gpu) headless_nvidia=1 ;;
    software) ;;
    *) printf 'Unsupported renderer: %s\n' "$render" >&2; exit 2 ;;
esac
mkdir "$dest/$role"
exec 9>/tmp/rmul-fixed-moving-20261120.lock
flock -n 9
set +e
env RMUL_HEADLESS_NVIDIA="$headless_nvidia" RMUL_SOFTWARE_THREADS=${RMUL_BENCH_THREADS:-12} RMUL_NAV_SENSOR_ONLY=1 RMUL_NAV_RENDER_LOD=1     bash "$dest/sim-shell.bash" -c '
        set -eo pipefail
        ulimit -c 0
        role=$1; render=$2; shift 2
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        run=/evidence/c072-speed200-motion-clock-20261122
        export RMUL_TEST_ARTIFACT_ROOT=$run/$role
        export RMUL_STORAGE_STATE_DIR=$RMUL_TEST_ARTIFACT_ROOT/storage-state
        export RMUL_MID_ODIN_INITIAL_VIEW=1 RMUL_SENSOR_PROFILE=cad-mid-odin RMUL_MID_ODIN_VIEW=1 RMUL_MID_ODIN_CLEAN_SHUTDOWN=1
        export RMUL_MID_GEOMETRIC_GUARD=1 RMUL_MID_ODIN_C072_FULL=1 RMUL_C072_TRANSFER_ROOT="$run" RMUL_MID_ODIN_C072_COSTMAP=1
        export RMUL_IMU_RECOVERY=1 RMUL_MID_ODIN_C072_LOCAL_PREDICTION=1
        export RMUL_MID_ODIN_C072_SWEPT_CLEARANCE=1
        export RMUL_ODIN_BOUNDED_SOURCE_TF=1
        export RMUL_ODIN_PLANAR_RETURNS=1
        export RMUL_MID_ODIN_SIM_SPEED_MPS=2.00
        scripts=$RMUL_WORKSPACE/src/pb2025_sentry_nav/scripts/rmul2027
        export RMUL2027_DDS_PROFILE="$run/source/fastdds_local_64m.xml"
        source "$scripts/configure_dds.sh"
        if [ "$role" = moving-standard ]; then
            fixture=$scripts/run_mid_odin_moving_acceptance.py
            set -- --fixed-acceptance "$run/fixed-acceptance.json" --complete-round --render-mode "$render" "$@"
        elif [[ "$role" = moving-diagnostic* ]]; then
            fixture=$scripts/run_mid_odin_moving_acceptance.py
            set -- --diagnostic-one-case --render-mode "$render" "$@"
        else
            fixture=$scripts/run_mid_odin_fixed_acceptance.py
            set -- --render-mode "$render" "$@"
        fi
        exec python3 -u "$fixture" --launcher "$scripts/start_cad_mid_odin_initial_tf_20261121.sh"           --record-dir "$RMUL_TEST_ARTIFACT_ROOT/cases" --test-decision-permit           --params-file "$run/params.yaml" "$@"
    ' isolated-costmap "$role" "$render" "$@" > "$dest/$role/run.log" 2>&1
code=$?
set -e
printf '%s\n' "$code" > "$dest/$role/exit-code.txt"
exit "$code"
