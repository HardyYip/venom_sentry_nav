import json
import os
import signal
import time
from pathlib import Path

workspace = Path(__file__).resolve().parents[4]
for run in sorted((workspace/'logs').glob('*-rmul2027-*'),reverse=True):
    identity_path=run/'launches.json'
    if not identity_path.is_file():
        continue
    identity=json.loads(identity_path.read_text())
    pid=identity['wrapper_pid']
    folder=Path(f'/proc/{pid}')
    try:
        command=(folder/'cmdline').read_bytes()
        if folder.stat().st_uid != os.getuid() or b'run_simulation.py' not in command:
            continue
        if (folder/'stat').read_text().rsplit(') ',1)[1].split()[0]=='Z':
            continue
    except FileNotFoundError:
        continue
    os.kill(pid,signal.SIGINT)
    deadline=time.monotonic()+40
    while folder.exists() and time.monotonic()<deadline:
        try:
            if (folder/'stat').read_text().rsplit(') ',1)[1].split()[0]=='Z':
                break
        except FileNotFoundError:
            break
        time.sleep(.2)
    print(f'Stopped RMUL 2027 {identity["mode"]} wrapper {pid}.')
    break
else:
    print('No running RMUL 2027 wrapper found.')
