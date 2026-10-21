import argparse
import base64
import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

GIB = 1024 ** 3
MIB = 1024 ** 2
HERE = Path(__file__).resolve().parent


def tree_bytes(root):
    total = 0
    if not root.exists():
        return 0
    for folder, dirs, files in os.walk(root, followlinks=False):
        dirs[:] = [name for name in dirs if not (Path(folder) / name).is_symlink()]
        for name in files:
            path = Path(folder) / name
            if path.is_symlink():
                continue
            try:
                total += path.stat().st_size
            except FileNotFoundError:
                pass
    return total


def existing_parent(path):
    path = path.resolve()
    while not path.exists():
        path = path.parent
    return path


def check_reasons(free, policy, starting=False):
    limit = policy['start_free_gib' if starting else 'stop_free_gib'] * GIB
    return [f'{path}: free={amount / GIB:.3f} GiB < {limit / GIB:g} GiB'
            for path, amount in free.items() if amount < limit]


def validate_policy(policy):
    if not (policy['start_free_gib'] > policy['warning_free_gib'] > policy['stop_free_gib'] > 0):
        raise ValueError('Require start > warning > stop > 0')
    if not (0 < policy['poll_seconds'] <= 1):
        raise ValueError('Poll interval must be in (0, 1] seconds')
    if policy['max_session_gib'] <= 0:
        raise ValueError('Session budget must be positive')
    for profile in policy['profiles'].values():
        if profile['max_run_mib'] <= 0 or profile['max_seconds'] <= 0:
            raise ValueError('Run budgets must be positive')


class Events:
    def __init__(self, folder, notify=True):
        self.folder = folder
        self.notify = notify
        folder.mkdir(parents=True, exist_ok=True)
        if notify:
            shutil.copyfile(HERE / 'notify_storage.py', folder / 'notify_storage.py')

    def emit(self, kind, detail, **extra):
        value = {'time': datetime.datetime.now().astimezone().isoformat(),
                 'kind': kind, 'detail': detail, **extra}
        data = json.dumps(value, ensure_ascii=False)
        with (self.folder / 'events.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(data + '\n')
        temporary = self.folder / ('status-' + uuid.uuid4().hex + '.tmp')
        temporary.write_text(data + '\n', encoding='utf-8')
        temporary.replace(self.folder / 'status.json')
        print(data, flush=True)
        if kind in ('blocked', 'warning', 'stop', 'error') and self.notify:
            self.desktop_alert(f'{kind}: {detail}')

    def desktop_alert(self, message):
        encoded = base64.b64encode(message.encode('utf-8')).decode('ascii')
        helper = self.folder / 'notify_storage.py'
        try:
            if os.name == 'nt':
                subprocess.Popen([sys.executable, '-B', str(helper), '--message-base64', encoded],
                                 creationflags=subprocess.CREATE_NO_WINDOW,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                windows_path = subprocess.check_output(['wslpath', '-w', str(helper)],
                                                       text=True, timeout=2).strip()
                windows_python = os.environ.get('RMUL_WINDOWS_PYTHON') or shutil.which('python.exe')
                if not windows_python:
                    raise FileNotFoundError('Windows Python is not available for desktop notification')
                subprocess.Popen([windows_python, '-B', windows_path, '--message-base64', encoded],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, subprocess.SubprocessError):
            print('\aDesktop alert unavailable; see status.json and console.', flush=True)


def tagged_processes(token):
    result = []
    if not token:
        return result
    marker = ('RMUL_STORAGE_RUN_ID=' + token).encode()
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            if marker in (item / 'environ').read_bytes().split(b'\0'):
                stamp = (item / 'stat').read_text().rsplit(')', 1)[1].split()[19]
                result.append((int(item.name), stamp))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return result


def stop_owned(process):
    if process is None:
        return
    if os.name == 'nt':
        if process.poll() is None:
            subprocess.run(['taskkill.exe', '/PID', str(process.pid), '/T', '/F'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        return
    token = getattr(process, '_storage_token', None)
    for sig, grace in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        targets = tagged_processes(token)
        if process.poll() is None:
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
        for pid, stamp in targets:
            try:
                current = Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()[19]
                if current == stamp:
                    os.kill(pid, sig)
            except (FileNotFoundError, ProcessLookupError):
                pass
        until = time.monotonic() + grace
        while time.monotonic() < until:
            if process.poll() is not None and not tagged_processes(token):
                return
            time.sleep(0.1)
    try:
        process.wait(timeout=1)
    except subprocess.TimeoutExpired:
        pass


def run(args):
    policy = json.loads(args.policy.read_text(encoding='utf-8-sig'))
    validate_policy(policy)
    root = args.artifact_root.resolve()
    wsl = os.name != 'nt' and 'microsoft' in Path('/proc/sys/kernel/osrelease').read_text().lower()
    if wsl and not (len(root.parts) > 3 and root.parts[1] == 'mnt' and len(root.parts[2]) == 1 and Path('/mnt', root.parts[2]).is_mount()):
        raise ValueError('WSL artifacts must be on a mounted Windows volume, outside the VHD')
    events = Events(args.state_dir.resolve(), not args.no_notify)
    paths = [Path(path).resolve() for path in args.host_root]
    paths.append(existing_parent(root))
    process = None
    try:
        def sample_free():
            return {str(path): shutil.disk_usage(path).free for path in paths}

        free = sample_free()
        reasons = check_reasons(free, policy, starting=True)
        session_size = tree_bytes(root)
        if session_size >= policy['max_session_gib'] * GIB:
            reasons.append(f'Artifact root has reached the {policy["max_session_gib"]:g} GiB session budget')
        if reasons:
            events.emit('blocked', '; '.join(reasons), free_bytes=free)
            return 73
        if args.check_only:
            events.emit('finished', 'Preflight passed; no command started', free_bytes=free)
            return 0
        command = args.command
        if command and command[0] == '--':
            command = command[1:]
        if not command:
            raise ValueError('Missing command after --')
        if os.name == 'nt' and Path(command[0]).stem.lower() in ('wsl', 'bash'):
            raise ValueError('Run this guard inside WSL to own the Linux process group')
        root.mkdir(parents=True, exist_ok=True)
        run_dir = root / (datetime.datetime.now().strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:8])
        run_dir.mkdir()
        token = uuid.uuid4().hex
        env = dict(os.environ, RMUL_ARTIFACT_DIR=str(run_dir), RMUL_TEST_ARTIFACT_ROOT=str(root),
                   RMUL_STORAGE_RUN_ID=token, RMUL_STORAGE_PARENT_PID=str(os.getpid()))
        profile = policy['profiles'][args.profile]
        kwargs = {'start_new_session': True} if os.name != 'nt' else {
            'creationflags': subprocess.CREATE_NEW_PROCESS_GROUP}
        process = subprocess.Popen(command, env=env, **kwargs)
        process._storage_token = token
        started = time.monotonic()
        baseline = free
        observed_baseline = {str(path): tree_bytes(path) for path in getattr(args, 'observe_root', [])}
        warned = False
        events.emit('start', 'Guarded command started', pid=process.pid,
                    artifact_dir=str(run_dir), command=command, profile=args.profile,
                    free_bytes=free, policy=policy)
        while True:
            free = sample_free()
            reasons = check_reasons(free, policy)
            growth = max([baseline[path] - free[path] for path in baseline] + [0])
            recorded = tree_bytes(run_dir)
            for observed in getattr(args, 'observe_root', []):
                if not observed.resolve().is_relative_to(run_dir):
                    recorded += max(0, tree_bytes(observed) - observed_baseline[str(observed)])
            if recorded >= profile['max_run_mib'] * MIB:
                reasons.append(f'Run budget reached: {profile["max_run_mib"]} MiB')
            if growth >= policy.get('max_host_growth_gib', 2) * GIB:
                reasons.append('Host disk growth budget reached, including writes outside artifacts')
            if tree_bytes(root) >= policy['max_session_gib'] * GIB:
                reasons.append('Artifact root session budget reached')
            if time.monotonic() - started >= profile['max_seconds']:
                reasons.append('Wall-clock run duration limit reached')
            if reasons:
                events.emit('stop', '; '.join(reasons), free_bytes=free,
                            artifact_bytes=recorded, observed_disk_growth=growth)
                stop_owned(process)
                events.emit('stopped', 'Owned process group and environment-tagged detached descendants stopped',
                            child_returncode=process.poll(), free_bytes=sample_free())
                return 74
            if not warned and any(value < policy['warning_free_gib'] * GIB for value in free.values()):
                events.emit('warning', f'Less than {policy["warning_free_gib"]:g} GiB free; no further test should start', free_bytes=free)
                warned = True
            result = process.poll()
            if result is not None:
                stop_owned(process)
                events.emit('finished', 'Command ended; no files deleted',
                            child_returncode=result, free_bytes=free, artifact_bytes=recorded)
                return result if result >= 0 else 1
            time.sleep(policy['poll_seconds'])
    except BaseException as error:
        stop_owned(process)
        events.emit('error', str(error))
        return 75


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--policy', type=Path, default=HERE / 'storage-policy.json')
    parser.add_argument('--host-root', action='append', default=None)
    parser.add_argument('--artifact-root', type=Path, required=True)
    parser.add_argument('--state-dir', type=Path, default=HERE / 'state')
    parser.add_argument('--observe-root', type=Path, action='append', default=[])
    parser.add_argument('--profile', choices=['light', 'raw', 'interactive'], default='light')
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--no-notify', action='store_true')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if os.name == 'nt':
        primary = os.environ.get('RMUL_STORAGE_HOST_ROOT', 'D:\\\\' if Path('D:\\\\').exists() else 'C:\\\\')
    else:
        wsl = 'microsoft' in Path('/proc/sys/kernel/osrelease').read_text().lower()
        primary = os.environ.get('RMUL_STORAGE_HOST_ROOT', '/mnt/d' if wsl and Path('/mnt/d').is_mount() else '/mnt/c' if wsl else '/')
        if wsl and not Path(primary).is_mount():
            parser.error('The host volume must be mounted; virtual WSL root free space is insufficient')
        if wsl and not (str(args.state_dir.resolve()).startswith(('/mnt/e/', '/mnt/c/'))):
            parser.error('--state-dir must be on an independent E: or C: Windows volume')
    args.host_root = [primary] + (args.host_root or [])
    sys.exit(run(args))


if __name__ == '__main__':
    main()
