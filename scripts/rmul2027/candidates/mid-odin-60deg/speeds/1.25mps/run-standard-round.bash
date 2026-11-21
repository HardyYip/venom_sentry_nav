#!/usr/bin/env bash
set -euo pipefail
run=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run_fixed() {
    local label=$1
    shift
    local result=0
    bash "$run/run-suite.bash" "$label" --repeats 3 --groups "$@" || result=$?
    [[ "$result" == 0 || "$result" == 1 ]]
    test -s "$run/$label/cases/static-acceptance.json"
}
run_fixed fixed-left left-near left-far
run_fixed fixed-reverse-right left-reverse right-control
python3 - "$run" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path(sys.argv[1])
labels=['fixed-left','fixed-reverse-right']
paths=[root/label/'cases/static-acceptance.json' for label in labels]
receipts=[json.loads(path.read_text()) for path in paths]
params=hashlib.sha256((root/'params.yaml').read_bytes()).hexdigest()
assert all(r['cases']==6 and r['params_sha256']==params and r.get('preposition_behavior_tree_sha256') is None
           for r in receipts)
results=[case for receipt in receipts for case in receipt['results']]
expected=[f'{route}-{repeat:02d}' for route in ('left-near','left-far','left-reverse','right-control')
          for repeat in range(1,4)]
assert [case['case'] for case in results]==expected
assert all(case['status'] in ('passed','failed') and not case['process_cleanup']['remaining'] for case in results)
passed=all(case['status']=='passed' for case in results)
(root/'fixed-acceptance.json').write_text(json.dumps({'passed':passed,'cases':12,'params_sha256':params,
    'preposition_behavior_tree_sha256':None,'results':results,
    'paired_receipts':[str(path.relative_to(root)) for path in paths]},indent=2)+'\n')
PY
exec bash "$run/run-suite.bash" moving-standard
