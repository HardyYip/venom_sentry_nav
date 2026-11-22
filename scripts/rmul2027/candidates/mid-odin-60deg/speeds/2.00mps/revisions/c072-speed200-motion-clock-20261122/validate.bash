#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$here"
python3 "$here/record-build-inputs.py" --verify "$here"
flock -w 180 /tmp/rmul-fixed-moving-20261120.lock true
bash run.bash > long-routes-run.log 2>&1
python3 review.py
python3 - <<'PY'
import json
from pathlib import Path
r=json.loads(Path('review.json').read_text())
rows=r['legs']
assert len(rows)==7 and all(x['goal_status']==4 and x['stop_verified'] for x in rows)
assert all(x['contacts']['ground'] and not any(x['contacts'][k] for k in ('wall','box','unknown')) for x in rows)
assert not r['contacts_decode_errors']
print('seven long routes passed')
PY
for point in '3.801 -1.942' '3.785 -1.942' '3.788 -2.1'; do
    read -r x y <<< "$point"
    bash run-rviz-inflation-test.bash "$x" "$y"
done
bash run-standard-round.bash
python3 audit-round.py "$here"
python3 audit-runtime.py "$here"
python3 audit-speed-round.py "$here" "$here/../c072-speed200-inflation-plan-handoff-20261122"
