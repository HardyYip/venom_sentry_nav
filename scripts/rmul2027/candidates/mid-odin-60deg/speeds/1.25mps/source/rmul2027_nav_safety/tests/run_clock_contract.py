from pathlib import Path
import argparse,subprocess,json,os,hashlib
p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--library',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False);env=os.environ.copy();env['ROS_DOMAIN_ID']='77'
result=subprocess.run([str(a.binary.resolve()),str(a.library.resolve())],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env,timeout=30)
(a.output/'actual-plugin.log').write_text(result.stdout);assert result.returncode==0
rows=json.loads(next(l.split('CLOCK_PROBE_JSON ',1)[1] for l in result.stdout.splitlines() if l.startswith('CLOCK_PROBE_JSON ')));assert len(rows)==13
for row in rows:assert row['status']==('FAILURE' if 'rejected' in row['case'] or row['case']=='clock_publisher_stopped_750ms' else 'SUCCESS'),row
(a.output/'contract.json').write_text(json.dumps({'passed':True,'actual_plugin_ticks':13,'library_sha256':hashlib.sha256(a.library.read_bytes()).hexdigest(),'rows':rows,'scope':'isolated actual PoseFresh clock-consumer safety; no historical navigation root-cause claim'},indent=2));print('CLOCK13 PASS')
