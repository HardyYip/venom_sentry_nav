"""Fetch the locked official simulator and companion dependencies safely."""
import pathlib
import re
import subprocess

import yaml

workspace = pathlib.Path('/home/hardy/pb2025_nav_ws')
with pathlib.Path(__file__).with_name('dependencies.lock.repos').open() as stream:
    repositories = yaml.safe_load(stream)['repositories']


def git(folder, *args):
    return subprocess.run(['git', '-C', str(folder), *args], check=True, timeout=180)


for name, spec in repositories.items():
    destination = (workspace / name).resolve()
    if not destination.is_relative_to(workspace) or destination == workspace:
        raise RuntimeError(f'Unsafe repository path: {name}')
    revision = spec['version']
    if spec['type'] != 'git' or not re.fullmatch(r'[0-9a-f]{40}', revision):
        raise RuntimeError(f'Expected a pinned Git commit: {name}')
    print(f'Preparing {name} at {revision}', flush=True)
    destination.mkdir(parents=True, exist_ok=True)
    if not (destination / '.git').exists():
        if any(destination.iterdir()):
            raise RuntimeError(f'Refusing nonempty non-Git directory: {destination}')
        git(destination, 'init', '-q')
        git(destination, 'remote', 'add', 'origin', spec['url'])
    else:
        origin = subprocess.check_output(
            ['git', '-C', str(destination), 'remote', 'get-url', 'origin'], text=True
        ).strip()
        if origin != spec['url']:
            raise RuntimeError(f'Unexpected origin: {destination}: {origin}')
        current = subprocess.run(
            ['git', '-C', str(destination), 'rev-parse', 'HEAD'],
            capture_output=True, text=True,
        )
        if current.returncode == 0 and current.stdout.strip() == revision:
            print('Pinned commit already present; preserving any local changes.', flush=True)
            continue
        status = subprocess.check_output(
            ['git', '-C', str(destination), 'status', '--porcelain'], text=True
        )
        if status.strip():
            raise RuntimeError(f'Preserve existing changes in {destination}')
    git(destination, 'fetch', '--depth', '1', 'origin', revision)
    git(destination, 'checkout', '--detach', revision)
