"""Cross-check closed-loop lane and velocity bench results from their recorded samples."""
import json
from pathlib import Path
import sys

import numpy as np
import yaml

def load(path):
    return json.loads(path.read_text())

lane, bench = [Path(folder) / 'straight' for folder in sys.argv[1:]]
lane_result = load(lane / 'result.json')
assert lane_result['passed'] and lane_result['contacts'] == 0
assert lane_result['distance_to_goal'] < .12 and lane_result['stop_seconds'] >= 1
assert 'Goal succeeded' in (lane / 'bt.log').read_text()
assert yaml.safe_load((lane/'nav.yaml').read_text()) == yaml.safe_load((bench/'nav.yaml').read_text())
cfg = yaml.safe_load((lane/'nav.yaml').read_text())
smoother = cfg['velocity_smoother']['ros__parameters']
ctrl = cfg['controller_server']['ros__parameters']['FollowPath']
assert ctrl['max_accel_xy'] == smoother['max_accel_xy'] == 1.75
assert ctrl['max_speed_xy'] == smoother['max_speed_xy'] == 2.5
assert smoother['smoothing_frequency'] == 20
ramp = load(bench/'ramp-trace.json')
dmax = 0.
for commands in (load(lane/'commands.json'), ramp['commands']):
    velocities = np.asarray(commands)[:, 1:]
    dmax = max(dmax, float(np.linalg.norm(np.diff(velocities, axis=0), axis=1).max()))
    assert np.linalg.norm(velocities, axis=1).max() <= 2.500001
assert dmax <= 1.75 / 20 + 1e-8
odom = np.asarray(ramp['odometry'])
t = odom[:, 0] - ramp['start']
speeds = np.linalg.norm(odom[:, 1:], axis=1)
cruise = (t >= 5) & (t <= 7.8)
duration = float(np.ptp(t[cruise]))
assert duration >= 2.7 and speeds[cruise].min() >= 2.499 and speeds.max() <= 2.501
assert np.ptp(t[t >= 10.5]) >= 1 and speeds[t >= 10.5].max() < .03
assert not ramp['contacts'] and load(bench/'ramp-result.json')['passed']
report = {'lane_odom_peak_mps': lane_result['odom_peak'], 'lane_length_m': 28,
          'lane_goal_error_m': lane_result['distance_to_goal'], 'bench_odom_peak_mps': float(speeds.max()),
          'bench_cruise_s': duration, 'max_command_delta_mps': dmax, 'contacts': 0}
for name, lower, upper in [('acceleration_fit_mps2', 2., 4.), ('deceleration_fit_mps2', 8., 10.)]:
    mask = (t > lower) & (t < upper) & (speeds > .5) & (speeds < 2.)
    assert mask.sum() >= 20
    report[name] = float(np.polyfit(t[mask], speeds[mask], 1)[0])
(bench.parent/'speed-audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
