if __name__ == '__main__':
 from storage_entry import ensure_storage_guard
 ensure_storage_guard(__file__)

import argparse,json,mmap,os,subprocess,sys,uuid
from pathlib import Path
from dds_shm_cleanup import SHM,SharedMemoryScope,identity,unlink_unused
parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
options=parser.parse_args()
root=options.record_dir.resolve()
root.mkdir(parents=True,exist_ok=False)
fixtures=[];out={}
def fixture():
 p=SHM/('fastrtps_'+uuid.uuid4().hex[:16])
 with p.open('xb') as f:f.write(bytes(4096))
 fixtures.append(p)
 return p
try:
 p=fixture();before=identity(p);f=p.open('r+b');mapped=mmap.mmap(f.fileno(),0);f.close()
 try:
  out['mmap_only_protected']=unlink_unused(p,before)
  assert not out['mmap_only_protected']['deleted'] and p.exists()
 finally:mapped.close()
 out['closed_mapping_deleted']=unlink_unused(p,before)
 assert out['closed_mapping_deleted']['deleted']
 p=fixture();before=identity(p)
 with p.open('r+b'):
  out['open_descriptor_protected']=unlink_unused(p,before)
  assert not out['open_descriptor_protected']['deleted'] and p.exists()
 p=fixture();before=identity(p);p.unlink();p.write_bytes(bytes(4096))
 out['replacement_protected']=unlink_unused(p,before)
 assert not out['replacement_protected']['deleted'] and p.exists()
 baseline=fixture();scope=SharedMemoryScope('rmul-contract-'+uuid.uuid4().hex,root/'scope-details.json')
 scope._add(baseline.name,os.getpid());assert not scope.records
 tracked=fixture();untracked=fixture()
 child=subprocess.Popen([sys.executable,'-c',"import sys,mmap,time;f=open(sys.argv[1],'r+b');m=mmap.mmap(f.fileno(),0);f.close();print('ready',flush=True);time.sleep(30)",str(tracked)],env={**os.environ,'RMUL_STORAGE_RUN_ID':scope.token},stdout=subprocess.PIPE,text=True)
 try:
  assert child.stdout.readline().strip()=='ready'
  scope.capture();assert len(scope.records)==1
  out['live_owned_protected']=scope.cleanup()
  assert out['live_owned_protected']['deleted_files']==0 and tracked.exists()
 finally:
  child.terminate();child.wait(timeout=5);child.stdout.close()
 out['closed_owned_deleted']=scope.cleanup()
 assert out['closed_owned_deleted']['deleted_files']==1 and not tracked.exists()
 assert baseline.exists() and untracked.exists()
 out['baseline_and_untracked_preserved']=True;out['passed']=True
 (root/'result.json').write_text(json.dumps(out,indent=2)+'\n')
 print(json.dumps({'passed':True,'checks':list(out)}))
finally:
 for p in fixtures:
  if p.exists():p.unlink()
