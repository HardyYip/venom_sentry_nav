from datetime import datetime
from pathlib import Path
import json
import subprocess

folder = Path(__file__).resolve().parent
native = folder.parent
out = folder / datetime.now().strftime('results-%Y%m%d-%H%M%S-%f')
out.mkdir()
successes = []
for case in sorted((folder / 'cases').iterdir()):
    cmd = [str(native / 'isolated.bash'), 'bash', f'/trial/{folder.name}/run-case.bash',
           folder.name, case.name, f'{folder.name}/{out.name}']
    with (out / (case.name+'.log')).open('w') as log:
        code = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT).returncode
    path = out / case.name / 'result.json'
    result = json.loads(path.read_text()) if path.is_file() else {'error':'result missing'}
    success = code==0 and result.get('passed',False)
    successes.append(success)
    print(case.name, success, result.get('error'),flush=True)
print(out, flush=True)
raise SystemExit(0 if len(successes) == 5 and all(successes) else 1)
