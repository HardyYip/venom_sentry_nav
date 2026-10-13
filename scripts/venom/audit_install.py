"""Check dynamic dependencies of installed workspace ELF files (read-only)."""
import json
import os
from pathlib import Path
import subprocess

install = Path('/home/hardy/pb2025_nav_ws/install')
seen = set()
missing = []
checked = []
for folder, _, names in os.walk(install, followlinks=True):
    for name in names:
        path = Path(folder) / name
        resolved = path.resolve()
        if resolved in seen or not path.is_file():
            continue
        seen.add(resolved)
        with path.open('rb') as stream:
            if stream.read(4) != b'\x7fELF':
                continue
        result = subprocess.run(['ldd', str(path)], capture_output=True, text=True)
        output = result.stdout + result.stderr
        checked.append(str(path.relative_to(install)))
        if 'not found' in output or (result.returncode and 'not a dynamic executable' not in output):
            missing.append({'file': str(path), 'ldd': output})
print(json.dumps({'checked_elf_files': len(checked), 'files': checked,
                  'missing': missing, 'pass': not missing}, indent=2))
raise SystemExit(bool(missing))
