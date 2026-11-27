"""Use the same cart evaluation with the synthetic straight-lane occupancy map."""
from pathlib import Path
import runpy
import sys
from unittest.mock import patch

out = Path(sys.argv[1])
original_glob = Path.glob

def track_map(path, pattern):
    if path == Path('/baseline/fixed-left') and pattern == '*/cad-mid-odin-*/map-boundary-margin.yaml':
        return iter([out / 'lane.yaml'])
    return original_glob(path, pattern)

with patch.object(Path, 'glob', track_map):
    runpy.run_path('/trial/evaluate.py', run_name='__main__')
