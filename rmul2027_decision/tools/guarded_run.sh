#!/usr/bin/env bash
set -eo pipefail
task_ws="${RMUL_WORKSPACE:-/home/hardy/pb2025_nav_ws}"
task_repo="$task_ws/src/pb2025_sentry_nav"
task_build="${RMUL_DECISION_BUILD_ROOT:-$task_ws/artifacts/rmul_2027_3v3/decision-C062}"
task_root="${RMUL_TEST_ARTIFACT_ROOT:?Set one fixed RMUL_TEST_ARTIFACT_ROOT for this decision session}"
if [ -n "${RMUL_STORAGE_RUN_ID:-}" ]; then
  python3 - "${RMUL_STORAGE_PARENT_PID:-}" <<'PY'
import sys
from pathlib import Path
assert b'storage_guard.py' in Path('/proc',sys.argv[1],'cmdline').read_bytes(),'Storage guardian absent'
PY
  exec "$@"
fi
python3 - "$task_build" "$task_root" <<'PY'
import os,sys
from pathlib import Path
total=0
for base in map(Path,sys.argv[1:]):
 for folder,dirs,files in os.walk(base):
  dirs[:]=[d for d in dirs if not (Path(folder)/d).is_symlink()]
  total+=sum((Path(folder)/f).stat().st_size for f in files if not (Path(folder)/f).is_symlink())
assert total+512*1024**2<4*1024**3,'Combined task budget lacks one bounded-run reserve'
print('Combined dependency/build/evidence bytes:',total,flush=True)
PY
exec python3 "$task_repo/scripts/rmul2027/storage_guard.py" --host-root /mnt/d --artifact-root "$task_root" --state-dir /mnt/e/RMULStorageState/decision-C062 --observe-root "$task_build" --observe-root "$task_root" --observe-root "$task_ws/logs" --profile light -- "$@"
