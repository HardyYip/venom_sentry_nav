"""Verify combined x/y velocity and contiguous cruise time on the diagonal navigation route."""
from itertools import groupby
import json
from pathlib import Path
import sys

import numpy as np

record = Path(sys.argv[1]) / 'diagonal'
read = lambda name: json.loads((record / name).read_text())
result = read('result.json')
assert result['passed'] and result['contacts'] == 0 and result['distance_to_goal'] < .12
assert result['stop_seconds'] >= 1 and 'Goal succeeded' in (record/'bt.log').read_text()
commands = np.asarray(read('commands.json'))[:, 1:]
assert np.linalg.norm(commands, axis=1).max() <= 2.500001
assert np.linalg.norm(np.diff(commands,axis=0),axis=1).max() <= .08750001
trace = read('trajectory.json')
intervals = [list(rows) for fast, rows in groupby(trace, key=lambda row: row[3] >= 2.48) if fast]
seconds = max((rows[-1][0] - rows[0][0] for rows in intervals), default=0)
assert seconds >= 1
report = {'odom_peak_mps': result['odom_peak'], 'continuous_seconds_at_or_above_2_48_mps': seconds,
          'goal_error_m': result['distance_to_goal'], 'contacts': result['contacts']}
(record.parent/'diagonal-audit.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report))
