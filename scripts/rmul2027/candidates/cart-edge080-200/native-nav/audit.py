"""Audit the recorded native runs independently of the online pass flag."""
import hashlib
import json
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np
import yaml

root = Path(__file__).resolve().parent
baseline = root.parent.parent / 'c072-verify-headon-planar-accel-20261123'
round_dir = root / sys.argv[1]
summary = json.loads((round_dir / 'summary.json').read_text())
assert summary['requested'] == summary['finished'] == 15
assert all(hashlib.sha256((root / path).read_bytes()).hexdigest() == sha
           for path, sha in summary['sha256'].items())
contact_dir = root / (sys.argv[2] if len(sys.argv) > 2 else 'contact-positive')
assert json.loads((contact_dir / 'check.json').read_text())['passed']
preflight = json.loads((root / 'preflight.json').read_text())
for path, sha in preflight['baseline_inputs'].items():
    assert hashlib.sha256((baseline / path).read_bytes()).hexdigest() == sha
for path, sha in preflight['other_runtime_inputs'].items():
    assert hashlib.sha256((root / path).read_bytes()).hexdigest() == sha, path
reference_root = baseline / 'fixed-left/cases/left-near-01/stack-evidence'
reference = yaml.safe_load((reference_root / 'nav2_params.yaml').read_text())
resources = json.loads((reference_root / 'resource-hashes.json').read_text())
for path, sha in resources.items():
    if '/scene/models/' in path and path.endswith('.stl'):
        mesh = root.parent / 'runtime/field-models' / path.split('/scene/models/')[1]
        assert hashlib.sha256(mesh.read_bytes()).hexdigest() == sha
limits = yaml.safe_load((root/'speed.yaml').read_text())
reference['controller_server']['ros__parameters']['FollowPath'].update(limits['controller_server']['FollowPath'])
reference['velocity_smoother']['ros__parameters'].update(limits['velocity_smoother'])
ids = set()
rows = []
for item in summary['results']:
    case_id = item['case']
    assert case_id not in ids and item['exit'] == 0
    ids.add(case_id)
    record = round_dir / case_id
    result = json.loads((record / 'result.json').read_text())
    scene = ET.parse(record / 'world.sdf')
    sensor = scene.find(".//sensor[@name='odin_lidar']")
    elevation = -math.degrees(float(sensor.findtext('pose').split()[4]))
    assert abs(elevation + 10) < 1e-8 and not any('projectile' in str(el.attrib).lower() for el in scene.iter())
    cfg = yaml.safe_load((record / 'nav.yaml').read_text())
    for node,plugin in [('controller_server','FollowPath'),('planner_server','GridBased')]:
        assert cfg[node]['ros__parameters'][plugin] == reference[node]['ros__parameters'][plugin]
    assert cfg['controller_server']['ros__parameters']['FollowPath']['max_speed_xy'] == 2.0
    assert cfg['velocity_smoother']['ros__parameters']['max_accel'] == reference['velocity_smoother']['ros__parameters']['max_accel']
    for node in ('local_costmap','global_costmap'):
        costmap = cfg[node][node]['ros__parameters']
        assert costmap['footprint_padding'] == 0.02
        assert all(costmap['inflation_layer'][k] == v for k,v in limits['inflation_layer'].items())
    smoother = cfg['velocity_smoother']['ros__parameters']
    assert smoother['max_speed_xy'] == 2.0 and smoother['max_accel_xy'] == 1.75
    assert cfg['controller_server']['ros__parameters']['FollowPath']['max_accel_xy'] == 1.75
    commands = np.asarray(json.loads((record/'commands.json').read_text()))
    assert len(commands) > 20 and np.all(np.isfinite(commands))
    max_step = float(np.max(np.linalg.norm(np.diff(commands[:,1:],axis=0),axis=1)))
    assert max_step <= 1.75 / smoother['smoothing_frequency'] + 1e-8
    trace = np.asarray(json.loads((record / 'trajectory.json').read_text()))
    case = json.loads((record / 'case.json').read_text())
    final_error = np.linalg.norm(trace[-1,1:3] - case['goal'])
    stop = trace[trace[:,0] >= trace[-1,0] - 1.11]
    assert len(stop) >= 6 and stop[-1,0] - stop[0,0] >= .999 and np.max(stop[:,3:5]) < .03
    assert final_error < .12 and result['gap'] > .05 and result['contacts'] == 0
    assert 0.1 < result['odom_peak'] <= 2.015 and result['cmd_peak'] <= 2.015
    assert result['plans'] > 0 and all(result['cloud_frames'].values()) and result['obstacle_returns']['odin'] > 0
    assert 'Goal succeeded' in (record / 'bt.log').read_text()
    assert result['passed'] and result['error'] is None
    if case['moving']:
        assert abs(result['box_speed']-.50) < .05 and result['box_travel'] >= 3.4
    rows.append({'case':case_id,'moving':case['moving'],'goal_error_m':float(final_error),
                 'stationary_seconds':result['stop_seconds'],'min_box_gap_m':result['gap'],
                 'max_command_step_mps':max_step,'peak_mps':result['odom_peak'],'odin_frames':result['cloud_frames']['odin']})
fixed = [r for r in rows if not r['moving']]
moving = [r for r in rows if r['moving']]
assert len(fixed) == 12 and len(moving) == 3
report = {'release':preflight['release'],'checked':len(rows),'fixed':len(fixed),'moving':len(moving),
          'min_fixed_gap_m':min(r['min_box_gap_m'] for r in fixed),
          'min_moving_gap_m':min(r['min_box_gap_m'] for r in moving),
          'max_goal_error_m':max(r['goal_error_m'] for r in rows),
          'min_stopped_s':min(r['stationary_seconds'] for r in rows),
          'max_command_step_mps':max(r['max_command_step_mps'] for r in rows),
          'max_measured_mps':max(r['peak_mps'] for r in rows),'rows':rows}
(round_dir / 'audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
