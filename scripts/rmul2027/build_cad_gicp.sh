#!/usr/bin/env bash
set -eo pipefail
W="${RMUL_WORKSPACE:-/home/hardy/pb2025_nav_ws}"
P="$W/src/pb2025_sentry_nav"
B="$W/artifacts/rmul_2027_3v3/cad-dual-mid-C072"
if [ -z "${RMUL_STORAGE_RUN_ID:-}" ]; then
 R="${RMUL_TEST_ARTIFACT_ROOT:?Set the fixed task root}"
 exec python3 "$P/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --artifact-root "$R" --state-dir "$R/storage-state" --observe-root "$R" --observe-root "$B" --observe-root "$W/logs" --profile light -- bash "$0" "$@"
fi
source /opt/ros/humble/setup.bash
source "$W/install/setup.bash"
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 OMP_NUM_THREADS=2 PYTHONDONTWRITEBYTECODE=1
copy="$B/gicp-metric1-source/small_gicp_relocalization"
if [ -d "$copy" ]; then
 python3 "$P/scripts/rmul2027/prepare_cad_gicp.py" --output-dir "$copy" --verify-only
else
 python3 "$P/scripts/rmul2027/prepare_cad_gicp.py" --output-dir "$copy"
fi
colcon --log-base "$B/gicp-metric1-colcon-log" build --base-paths "$copy" --build-base "$B/gicp-metric1-build" --install-base "$B/gicp-metric1-install" --executor sequential --allow-overriding small_gicp_relocalization --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_SOURCE_DIR_SMALL_GICP="$W/deps/small_gicp" -DFETCHCONTENT_FULLY_DISCONNECTED=ON
ctest --test-dir "$B/gicp-metric1-build/small_gicp_relocalization" -R '^test_(registration_budget|global_sampling|metric_correspondence)$' --output-on-failure
python3 - "$B" "$W" <<'PY'
from pathlib import Path
import sys
b=Path(sys.argv[1]);w=Path(sys.argv[2]);root=b/'cad-metric1-runtime'
root.mkdir(exist_ok=True)
for name,target in [('safety-install',w/'artifacts/rmul_2027_3v3/imu-recovery-C064/safety-install'),('localization-install',b/'gicp-metric1-install')]:
 link=root/name
 if link.exists():assert link.is_symlink() and link.resolve()==target.resolve()
 else:link.symlink_to(target,target_is_directory=True)
print('CAD controlled-recovery-only runtime ready; original safety library reused')
PY
