"""Prepare a hash-checked private controller copy. Original submodule is read-only."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
WORKSPACE=REPO.parents[1]
OVERLAYS=HERE/'overlays'
ALLOWED=WORKSPACE/'artifacts/rmul_2027_3v3/cad-dual-mid-C072'
def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def baseline():
    data=json.loads((OVERLAYS/'controller_c070_baseline.json').read_text())
    source=REPO/'pb_omni_pid_pursuit_controller'
    for name,digest in data['source_sha256'].items():
        if sha(source/name)!=digest:raise ValueError('original controller baseline changed: '+name)
    library=WORKSPACE/'install/pb_omni_pid_pursuit_controller/lib/libpb_omni_pid_pursuit_controller.so'
    if sha(library)!=data['original_library_sha256']:raise ValueError('original production controller changed')
    return source,data
def checked_output(path):
    path=Path(path).resolve()
    if not path.is_relative_to(ALLOWED.resolve()) or path==ALLOWED.resolve():
        raise ValueError('controller copy must be under the task private artifact root')
    return path
def verify(path):
    path=checked_output(path);source,data=baseline()
    manifest=json.loads((path/'projection-copy-manifest.json').read_text())
    for name,digest in manifest['private_files_sha256'].items():
        if sha(path/name)!=digest:raise ValueError('private source changed: '+name)
    for name,digest in manifest['overlay_inputs_sha256'].items():
        if sha(OVERLAYS/name)!=digest:raise ValueError('overlay input changed: '+name)
    return manifest
def prepare(path):
    path=checked_output(path);source,data=baseline()
    if path.exists():raise FileExistsError('refusing to overwrite an existing source copy')
    path.parent.mkdir(parents=True,exist_ok=True)
    shutil.copytree(source,path,ignore=shutil.ignore_patterns('.git','__pycache__'))
    subprocess.run(['patch','--batch','--forward','--fuzz=0','-p1','--input',str(OVERLAYS/'controller_projection.patch')],cwd=path,check=True)
    subprocess.run(['patch','--batch','--forward','--fuzz=0','-p1','--input',str(OVERLAYS/'controller_cell_coverage.patch')],cwd=path,check=True)
    shutil.copy2(OVERLAYS/'pose_projection.hpp',path/'include/pb_omni_pid_pursuit_controller/pose_projection.hpp')
    shutil.copy2(OVERLAYS/'test_pose_projection.cpp',path/'test/test_pose_projection.cpp')
    shutil.copy2(OVERLAYS/'planning_clearance.hpp',path/'include/pb_omni_pid_pursuit_controller/planning_clearance.hpp')
    shutil.copy2(OVERLAYS/'test_planning_clearance.cpp',path/'test/test_planning_clearance.cpp')
    shutil.copy2(OVERLAYS/'test_goal_field_cell_coverage.cpp',path/'test/test_goal_field_cell_coverage.cpp')
    baseline()
    names=['controller_projection.patch','pose_projection.hpp','test_pose_projection.cpp','controller_c070_baseline.json','planning_clearance.hpp','test_planning_clearance.cpp','controller_cell_coverage.patch','test_goal_field_cell_coverage.cpp']
    manifest=dict(original_read_only=str(source),private_copy=str(path),original_unchanged=True,
        baseline_source_sha256=data['source_sha256'],overlay_inputs_sha256={name:sha(OVERLAYS/name) for name in names},
        private_files_sha256={str(file.relative_to(path)):sha(file) for file in sorted(path.rglob('*')) if file.is_file()})
    (path/'projection-copy-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return verify(path)
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True);parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args()
    data=verify(args.output_dir) if args.verify_only else prepare(args.output_dir)
    print(json.dumps(dict(passed=True,private_copy=data['private_copy'],files=len(data['private_files_sha256']),original_unchanged=True)))
