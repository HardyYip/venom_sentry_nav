import os
import json
from pathlib import Path
import sys


def ensure_storage_guard(script):
    if '--help' in sys.argv or '-h' in sys.argv:
        return
    if '--record-pipeline' in sys.argv:
        raise SystemExit('Full-route raw recording is disabled. Use record_raw_window.py for a bounded diagnostic window.')
    script = Path(script).resolve()
    workspace = script.parents[4]
    wsl = 'microsoft' in Path('/proc/sys/kernel/osrelease').read_text().lower()
    local_path = workspace / '.rmul-storage.json'
    local = json.loads(local_path.read_text()) if local_path.exists() else {}
    if wsl:
        host = Path(os.environ.get('RMUL_STORAGE_HOST_ROOT', local.get('host_root', '/mnt/d' if Path('/mnt/d').is_mount() else '/mnt/c')))
        mount = '/mnt/e' if Path('/mnt/e').is_mount() else str(host)
        default_root = Path(mount) / 'RMULArtifacts' / workspace.name
    else:
        host = Path(os.environ.get('RMUL_STORAGE_HOST_ROOT', local.get('host_root', str(workspace))))
        default_root = Path.home() / '.local/state/rmul2027' / workspace.name
    root = Path(os.environ.get('RMUL_TEST_ARTIFACT_ROOT', local.get('artifact_root', str(default_root)))).expanduser().resolve()
    record = None
    for index, value in enumerate(sys.argv[1:], 1):
        if value == '--record-dir' and index + 1 < len(sys.argv):
            record = Path(sys.argv[index + 1]).expanduser().resolve()
        elif value.startswith('--record-dir='):
            record = Path(value.split('=', 1)[1]).expanduser().resolve()
    if record is not None and (record == root or not record.is_relative_to(root)):
        raise SystemExit(f'Use --record-dir beneath {root}; recording inside the WSL VHD is disabled. Configure RMUL_TEST_ARTIFACT_ROOT explicitly if needed.')
    token = os.environ.get('RMUL_STORAGE_RUN_ID')
    parent = os.environ.get('RMUL_STORAGE_PARENT_PID')
    if token and parent:
        try:
            command = Path('/proc', parent, 'cmdline').read_bytes()
            if b'storage_guard.py' in command:
                return
        except FileNotFoundError:
            pass
        raise SystemExit('Storage guardian is absent; start from the normal guarded entry again.')
    if wsl:
        state_default = Path('/mnt/e/RMULStorageState') / workspace.name if Path('/mnt/e').is_mount() else Path('/mnt/c/RMULStorageState') / workspace.name
    else:
        state_default = root / '.guard-state'
    state = Path(os.environ.get('RMUL_STORAGE_STATE_DIR', local.get('state_dir', str(state_default)))).expanduser().resolve()
    profile = 'raw' if script.name == 'record_raw_window.py' else 'interactive' if script.name == 'run_simulation.py' else 'light'
    command = [sys.executable, str(script.parent / 'storage_guard.py'), '--host-root', str(host),
               '--artifact-root', str(root), '--state-dir', str(state), '--profile', profile,
               '--observe-root', str(workspace / 'logs')]
    if record is not None:
        command += ['--observe-root', str(record)]
    command += ['--', sys.executable, str(script), *sys.argv[1:]]
    if local.get('windows_python'):
        os.environ['RMUL_WINDOWS_PYTHON'] = local['windows_python']
    os.environ['RMUL_STORAGE_HOST_ROOT'] = str(host)
    os.execv(sys.executable, command)
