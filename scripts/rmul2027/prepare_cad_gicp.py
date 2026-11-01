"""Copy the protected GICP source into a bounded private recovery-only build."""
import argparse,hashlib,json,shutil,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1];W=REPO.parents[1];ROOT=W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072';OVERLAYS=HERE/'overlays'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def baseline():
 data=json.loads((OVERLAYS/'gicp_c064_baseline.json').read_text());source=REPO/'small_gicp_relocalization'
 for name,digest in data['source_sha256'].items():
  if sha(source/name)!=digest:raise ValueError('protected GICP source changed: '+name)
 if sha(data['original_recovery_library'])!=data['original_recovery_library_sha256']:raise ValueError('original recovery library changed')
 dep=Path(data['small_gicp_dependency'])
 if subprocess.check_output(['git','rev-parse','HEAD'],cwd=dep,text=True).strip()!=data['small_gicp_git_sha'] or subprocess.check_output(['git','status','--porcelain'],cwd=dep):raise ValueError('pinned small_gicp dependency changed')
 return source,data
def checked(path):
 path=Path(path).resolve()
 if path==ROOT.resolve() or not path.is_relative_to(ROOT.resolve()):raise ValueError('GICP copy must stay within private task root')
 return path
def verify(path):
 path=checked(path);baseline();data=json.loads((path/'cad-recovery-copy-manifest.json').read_text())
 for name,digest in data['private_files_sha256'].items():
  if sha(path/name)!=digest:raise ValueError('private source changed: '+name)
 for name,digest in data['overlay_inputs_sha256'].items():
  if sha(OVERLAYS/name)!=digest:raise ValueError('GICP overlay changed: '+name)
 return data
def prepare(path):
 path=checked(path);source,base=baseline()
 if path.exists():raise FileExistsError('refusing to overwrite GICP source copy')
 path.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(source,path,ignore=shutil.ignore_patterns('.git','__pycache__'))
 subprocess.run(['patch','--batch','--forward','--fuzz=0','-p1','--input',str(OVERLAYS/'gicp_iteration_budget.patch')],cwd=path,check=True)
 subprocess.run(['patch','--batch','--forward','--fuzz=0','-p1','--input',str(OVERLAYS/'gicp_deterministic_map.patch')],cwd=path,check=True)
 subprocess.run(['patch','--batch','--forward','--fuzz=0','-p1','--input',str(OVERLAYS/'gicp_metric_correspondence.patch')],cwd=path,check=True)
 shutil.copy2(OVERLAYS/'registration_budget.hpp',path/'include/small_gicp_relocalization/registration_budget.hpp')
 shutil.copy2(OVERLAYS/'metric_correspondence.hpp',path/'include/small_gicp_relocalization/metric_correspondence.hpp')
 (path/'test').mkdir(exist_ok=False);shutil.copy2(OVERLAYS/'test_registration_budget.cpp',path/'test/test_registration_budget.cpp')
 shutil.copy2(OVERLAYS/'test_global_sampling.cpp',path/'test/test_global_sampling.cpp')
 shutil.copy2(OVERLAYS/'test_metric_correspondence.cpp',path/'test/test_metric_correspondence.cpp')
 baseline()
 names=['gicp_c064_baseline.json','gicp_iteration_budget.patch','registration_budget.hpp','test_registration_budget.cpp','gicp_deterministic_map.patch','test_global_sampling.cpp','gicp_metric_correspondence.patch','metric_correspondence.hpp','test_metric_correspondence.cpp']
 data=dict(private_copy=str(path),original_read_only=str(source),normal_runtime_unchanged=True,dependency_git=base['small_gicp_git_sha'],private_files_sha256={str(x.relative_to(path)):sha(x) for x in sorted(path.rglob('*')) if x.is_file()},overlay_inputs_sha256={name:sha(OVERLAYS/name) for name in names})
 (path/'cad-recovery-copy-manifest.json').write_text(json.dumps(data,indent=2)+'\n');return verify(path)
if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-dir',type=Path,required=True);parser.add_argument('--verify-only',action='store_true');args=parser.parse_args()
 data=verify(args.output_dir) if args.verify_only else prepare(args.output_dir)
 print(json.dumps(dict(passed=True,private_copy=data['private_copy'],files=len(data['private_files_sha256']),dependency_git=data['dependency_git'])))
