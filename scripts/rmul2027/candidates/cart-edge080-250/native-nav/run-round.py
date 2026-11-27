"""Run each case in a fresh, network-isolated process group and retain all outcomes."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
root=Path(__file__).resolve().parent
name=sys.argv[1]
if not name.replace('-','').isalnum():raise ValueError('Invalid run label')
out=root/name
out.mkdir()
paths=('scene.py','evaluate.py','run-case.bash','isolated.bash','odin-final.sdf','speed.yaml')
hashes={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in paths}
base=root.parent.parent/'c072-verify-headon-planar-accel-20261123'
cases=[(group,p.parent.name) for group in ['fixed-left','fixed-reverse-right','moving-standard'] for p in sorted((base/group/'cases').glob('*/session.json'))]
assert len(cases)==15
if len(sys.argv)>2:
    cases=[(g,c) for g,c in cases if c in sys.argv[2:]]
    assert len(cases)==len(sys.argv)-2
reports=[]
for group,case in cases:
    assert all(hashlib.sha256((root/f).read_bytes()).hexdigest()==v for f,v in hashes.items())
    with (out/(case+'.log')).open('w') as log:
        proc=subprocess.run([str(root/'isolated.bash'),'bash','/trial/run-case.bash',group,case,name],stdout=log,stderr=subprocess.STDOUT)
    source=out/case/'result.json'
    report=json.loads(source.read_text()) if source.exists() else {'passed':False,'case':case,'error':'No result produced'}
    report['exit']=proc.returncode
    report['passed']=report['passed'] and proc.returncode==0
    reports.append(report)
    status={'requested':len(cases),'finished':len(reports),'passed':sum(r['passed'] for r in reports),'sha256':hashes,'results':reports}
    (out/'summary.json').write_text(json.dumps(status,indent=2))
    print(case,report['passed'],report['error'],flush=True)
sys.exit(0 if status['passed']==len(cases) else 1)
