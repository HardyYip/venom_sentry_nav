if __name__ == "__main__":
    from storage_entry import ensure_storage_guard
    ensure_storage_guard(__file__)

import argparse
from pathlib import Path
import datetime
import hashlib
import json
import os
import shlex
import signal
import subprocess
import sys
import time

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
args=parser.parse_args()
folder=Path(__file__).resolve().parent;workspace=folder.parents[3];repository=folder.parents[1]
record=args.record_dir.resolve();record.mkdir(parents=True,exist_ok=False)
original=repository/'scripts/venom/start_headless.sh'
wrapper=original.read_text().replace('task_ws=/home/hardy/pb2025_nav_ws','task_ws='+shlex.quote(str(workspace)))
wrapper=wrapper.replace('source "$task_ws/install/setup.bash"','source "$task_ws/install/setup.bash"\nexport LD_LIBRARY_PATH="$task_ws/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib:${LD_LIBRARY_PATH:-}"')
wrapper=wrapper.replace('export LIBGL_ALWAYS_SOFTWARE=1\nexport GALLIUM_DRIVER=llvmpipe','unset LIBGL_ALWAYS_SOFTWARE\nexport GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA')
(record/'launcher.sh').write_text(wrapper)
if not (workspace/'artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib/libtf2.so').is_file():raise SystemExit('Missing local TF2 runtime')
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(folder/'config/fastdds_local_64m.xml')
os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
meta={'owner':'Codex C022','passed':False,'started_at':datetime.datetime.now().astimezone().isoformat(),
      'scope':'Original rmuc_2025 readiness and near-goal verifier; original spin=3.14 retained. Local TF2/DDS and GPU environment only.',
      'launcher_source_sha256':hashlib.sha256(original.read_bytes()).hexdigest(),'processes':{}}
children={};streams=[]
def save():(record/'session.json').write_text(json.dumps(meta,indent=2))
def launch(name,command):
 stream=(record/(name+'.log')).open('w');streams.append(stream)
 proc=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
 children[name]=proc;meta['processes'][name]={'pid':proc.pid,'command':command};save();return proc
try:
 launcher=launch('launcher',['bash',str(record/'launcher.sh'),'nav'])
 deadline=time.monotonic()+60
 while time.monotonic()<deadline:
  if launcher.poll() is not None:raise RuntimeError('Legacy launcher exited')
  if 'Headless nav started.' in (record/'launcher.log').read_text():break
  time.sleep(.2)
 else:raise TimeoutError('Legacy startup timeout')
 check=launch('verify',[sys.executable,str(repository/'scripts/venom/verify_headless.py'),'--mode','nav'])
 deadline=time.monotonic()+300
 while check.poll() is None:
  if launcher.poll() is not None:raise RuntimeError('Legacy launcher exited during verification')
  if time.monotonic()>deadline:raise TimeoutError('Legacy verifier timeout')
  time.sleep(.2)
 entries=[]
 for line in (record/'verify.log').read_text().splitlines():
  try:entries.append(json.loads(line))
  except ValueError:pass
 meta.update(events=entries,passed=check.returncode==0 and any(x.get('event')=='PASS' for x in entries))
except BaseException as error:meta['error']=repr(error)
finally:
 for name in ['verify','launcher']:
  proc=children.get(name)
  if proc and proc.poll() is None:
   proc.send_signal(signal.SIGINT)
   try:proc.wait(timeout=45)
   except subprocess.TimeoutExpired:meta.setdefault('cleanup_pending',[]).append(name)
 meta['finished_at']=datetime.datetime.now().astimezone().isoformat();save()
 for stream in streams:stream.close()
print(json.dumps(meta,indent=2))
raise SystemExit(0 if meta['passed'] and not meta.get('cleanup_pending') else 1)
