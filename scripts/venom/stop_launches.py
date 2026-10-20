"""Stop only the launch trees owned by this wrapper, including detached children."""
import os
import pathlib
import signal
import sys
import time

user_id = os.getuid()
tracked = {}


def identity(pid):
    try:
        folder = pathlib.Path('/proc') / str(pid)
        if folder.stat().st_uid != user_id:
            return None
        fields = (folder / 'stat').read_text().rsplit(') ', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return fields[19]
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None


def capture(pid):
    stamp = identity(pid)
    if stamp is None or pid in tracked:
        return
    tracked[pid] = stamp
    try:
        children = pathlib.Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        children = []
    for child in children:
        capture(int(child))


roots = []
for value in sys.argv[1:]:
    pid = int(value)
    try:
        command = pathlib.Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ')
    except FileNotFoundError:
        continue
    if b'ros2 launch' not in command:
        raise RuntimeError(f'Refusing unrelated root PID {pid}')
    if b'headless_sim.launch.py' not in command and b'headless_rmul2027.launch.py' not in command and b'pb2025_nav_bringup' not in command:
        raise RuntimeError(f'Refusing unknown launch PID {pid}')
    roots.append(pid)
    capture(pid)


def send(pids, signum):
    for pid in pids:
        stamp = tracked.get(pid)
        if stamp is None or identity(pid) != stamp:
            continue
        try:
            os.kill(pid, signum)
        except ProcessLookupError:
            pass


def wait_until_stopped(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not any(identity(pid) == stamp for pid, stamp in tracked.items()):
            return True
        time.sleep(.1)
    return False


send(roots, signal.SIGINT)
if not wait_until_stopped(12):
    send(reversed(list(tracked)), signal.SIGTERM)
    if not wait_until_stopped(5):
        send(reversed(list(tracked)), signal.SIGKILL)
        wait_until_stopped(2)
