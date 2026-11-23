"""Compare the measured speed of each formal case with its qualified baseline."""
import hashlib
import json
import math
import sys
from pathlib import Path

import yaml


def read(path):
    return json.loads(path.read_text())


def speeds(path):
    peaks = {phase: {'truth_mps': 0.0, 'controller_mps': 0.0, 'output_mps': 0.0}
             for phase in ('preposition', 'goal')}
    with path.open() as stream:
        for line in stream:
            row = json.loads(line)
            phase = row.get('phase')
            if phase not in peaks:
                continue
            peak = peaks[phase]
            if row.get('kind') == 'motion':
                peak['truth_mps'] = max(peak['truth_mps'], row['truth_speed'])
                peak['output_mps'] = max(peak['output_mps'], math.hypot(*row['cmd_vel'][:2]))
            elif row.get('kind') == 'cmd_vel_controller':
                peak['controller_mps'] = max(peak['controller_mps'], math.hypot(*row['value'][:2]))
    return peaks


def verify_configuration(path, expected):
    active = yaml.safe_load(path.read_text())
    controller = active['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    expected_controller = expected['controller_server']['ros__parameters']
    for key in ('acc_lim_x', 'acc_lim_y', 'decel_lim_x', 'decel_lim_y', 'sim_time',
                'forward_prune_distance', 'limit_vel_cmd_in_traj', 'max_speed_xy',
                'trajectory_generator_name', 'vx_samples', 'vy_samples', 'plugin'):
        assert follow[key] == expected_controller['FollowPath'][key], (path, key)
    if 'max_accel_xy' in expected_controller['FollowPath']:
        assert follow['max_accel_xy'] == expected_controller['FollowPath']['max_accel_xy'], path
    assert follow['discretize_by_time'] is True, path
    assert math.isclose(follow['time_granularity'], 1. / controller['controller_frequency']), path
    assert follow['planning_clearance_radius'] == 0.5 and follow['planning_cell_coverage'], path
    smoother = active['velocity_smoother']['ros__parameters']
    expected_smoother = expected['velocity_smoother']['ros__parameters']
    for key in ('smoothing_frequency', 'feedback', 'scale_velocities', 'max_velocity',
                'min_velocity', 'max_accel', 'max_decel'):
        assert smoother[key] == expected_smoother[key], (path, key)
    if 'max_accel_xy' in expected_smoother:
        assert smoother['max_accel_xy'] == expected_smoother['max_accel_xy'], path
    planner = active['planner_server']['ros__parameters']['GridBased']
    assert planner.get('allow_inflation_escape') == expected['planner_server']['ros__parameters'][
        'GridBased'].get('allow_inflation_escape'), path
    assert controller['general_goal_checker']['xy_goal_tolerance'] == 0.08, path
    critics = expected_controller['FollowPath']['critics']
    for critic in critics:
        assert critic in follow['critics'], (path, critic)
        key = critic + '.scale'
        if key in expected_controller['FollowPath']:
            assert follow[key] == expected_controller['FollowPath'][key], (path, key)
    for key in ('width', 'height', 'resolution', 'footprint', 'footprint_padding'):
        local = active['local_costmap']['local_costmap']['ros__parameters']
        source = expected['local_costmap']['local_costmap']['ros__parameters']
        if key in source:
            assert local[key] == source[key], (path, key)


def audit(root, base):
    stages = {'fixed-left': 6, 'fixed-reverse-right': 6, 'moving-standard': 3}
    expected = yaml.safe_load((root / 'params.yaml').read_text())
    param_digest = hashlib.sha256((root / 'params.yaml').read_bytes()).hexdigest()
    receipts = read(root / 'qualification-summary.json')
    assert receipts['completed_standard_round'], root
    rows = []
    for stage, total in stages.items():
        cases = receipts['stages'][stage]['results']
        assert len(cases) == total, stage
        for case in cases:
            name = case['case']
            candidate = root / stage / 'cases' / name
            historical = base / stage / 'cases' / name
            session = read(candidate / 'session.json')
            assert session['params_sha256'] == param_digest and case['process_remaining'] == [], name
            stack = candidate / 'stack-evidence'
            active_files = [stack / 'nav2_params.yaml']
            active_files.extend(sorted((stack / 'imu-recovery').glob('nav-epoch-*.yaml')))
            for active in active_files:
                verify_configuration(active, expected)
            rows.append({
                'case': name, 'stage': stage, 'result': case['status'], 'error': case['error'],
                'contacts': case['contacts'], 'minimum_gap_m': case['minimum_gap_m'],
                'baseline_gap_m': read(historical / 'session.json').get(
                    'opponent_min_gap_m' if stage == 'moving-standard' else 'minimum_clearance_m'),
                'baseline': speeds(historical / 'statistics.jsonl'),
                'candidate': speeds(candidate / 'statistics.jsonl'),
                'effective_configuration_files': [str(p.relative_to(root)) for p in active_files],
            })
    assert len(rows) == 15
    (root / 'standard-round-speed-comparison.json').write_text(json.dumps(rows, indent=2) + '\n')


if __name__ == '__main__':
    audit(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
