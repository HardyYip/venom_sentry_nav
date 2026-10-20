import argparse
import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path
import yaml

parser=argparse.ArgumentParser()
parser.add_argument('--source-cache',type=Path)
parser.add_argument('--resources-only',action='store_true')
parser.add_argument('--legacy-pcd',type=Path)
args=parser.parse_args()
folder=Path(__file__).resolve().parent
root=folder.parents[1]
workspace=folder.parents[3]
cache=args.source_cache.resolve() if args.source_cache else None
assets=folder/'assets'
patches=folder/'patches'
external=json.loads((assets/'external-resources.json').read_text())

def git(path,*command,check=True):
    return subprocess.run(['git','-C',str(path),*command],check=check,capture_output=True,text=True,timeout=240)

def ensure_repo(destination,url,revision):
    if destination.exists() and (destination/'.git').exists():
        current=git(destination,'rev-parse','HEAD').stdout.strip()
        if current!=revision:raise RuntimeError('Existing repository has a different revision: '+str(destination))
        return
    if destination.exists() and any(destination.iterdir()):raise RuntimeError('Preserve nonempty directory: '+str(destination))
    destination.parent.mkdir(parents=True,exist_ok=True)
    relative=destination.relative_to(workspace)
    source=cache/relative if cache else None
    command=['git','clone','--no-hardlinks','--no-checkout',str(source) if source and (source/'.git').exists() else url,str(destination)]
    subprocess.run(command,check=True,timeout=240)
    git(destination,'remote','set-url','origin',url)
    git(destination,'checkout','--detach',revision)

def submodules(repository):
    config=repository/'.gitmodules'
    if not config.exists():return
    lines=git(repository,'config','-f',str(config),'--get-regexp',r'^submodule\..*\.path$').stdout.splitlines()
    for line in lines:
        key,path=line.split(None,1)
        url=git(repository,'config','-f',str(config),'--get',key[:-4]+'url').stdout.strip()
        destination=(repository/path).resolve()
        if not destination.is_relative_to(workspace):raise RuntimeError('Submodule outside workspace')
        entry=git(repository,'ls-tree','HEAD',path).stdout.split()
        if len(entry)<3 or entry[0]!='160000':raise RuntimeError('Expected pinned submodule: '+path)
        ensure_repo(destination,url,entry[2])
        submodules(destination)

def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
    return value.hexdigest()

def restore_file(source,destination,expected=None):
    expected=expected or digest(source)
    if digest(source)!=expected:raise RuntimeError('Source hash mismatch: '+str(source))
    if destination.exists():
        if digest(destination)!=expected:raise RuntimeError('Preserve conflicting resource: '+str(destination))
        return
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination)

if not args.resources_only:
    submodules(root)
    specifications=yaml.safe_load((folder.parent/'venom/dependencies.lock.repos').read_text())['repositories']
    for path,spec in specifications.items():
        destination=(workspace/path).resolve()
        if not destination.is_relative_to(workspace):raise RuntimeError('Dependency outside workspace')
        ensure_repo(destination,spec['url'],spec['version'])
        submodules(destination)
    operations=[]
    for spec in json.loads((patches/'manifest.json').read_text())['repositories']:
        destination=workspace/spec['path']
        patch=patches/spec['patch']
        if digest(patch)!=spec['sha256']:raise RuntimeError('Patch hash mismatch')
        if git(destination,'rev-parse','HEAD').stdout.strip()!=spec['base_commit']:raise RuntimeError('Patch base mismatch')
        if git(destination,'apply','--reverse','--check',str(patch),check=False).returncode==0:
            print('Patch already present: '+spec['path'],flush=True)
            continue
        check=git(destination,'apply','--check',str(patch),check=False)
        if check.returncode:raise RuntimeError('Conflicting dependency changes: '+spec['path']+' '+check.stderr)
        operations.append((destination,patch))
    for destination,patch in operations:
        git(destination,'apply',str(patch))
        print('Applied '+patch.name,flush=True)
artifact=workspace/'artifacts/rmul_2027_3v3'
for source in sorted((assets/'maps-cad').iterdir()):restore_file(source,artifact/'maps-cad'/source.name)
restore_file(assets/'height_preview.npz',artifact/'height_preview.npz')
restore_file(assets/'step-source.json',artifact/'source/manifest.json')
info=external['tf2']
package=assets/'runtime'/info['package']
if digest(package)!=info['sha256']:raise RuntimeError('TF2 package hash mismatch')
library=artifact/'runtime/tf2-0.25.24/lib/libtf2.so'
if library.exists():
    if digest(library)!=info['library_sha256']:raise RuntimeError('Preserve conflicting TF2 library')
else:
    with tempfile.TemporaryDirectory(prefix='rmul2027-tf2-') as temp:
        subprocess.run(['dpkg-deb','-x',str(package),temp],check=True)
        source=Path(temp)/'opt/ros/humble/lib/libtf2.so'
        restore_file(source,library,info['library_sha256'])
legacy=args.legacy_pcd.resolve() if args.legacy_pcd else None
if legacy is None and cache:
    candidate=cache/external['legacy_pcd']['relative_path']
    if candidate.is_file():legacy=candidate
if legacy is not None:restore_file(legacy,workspace/external['legacy_pcd']['relative_path'],external['legacy_pcd']['sha256'])
print(json.dumps({'workspace':str(workspace),'maps':'verified','tf2':'verified local extraction only','legacy_pcd_restored':legacy is not None,'source_cache_is_not_runtime_overlay':True},indent=2))
