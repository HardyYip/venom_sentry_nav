from collections import Counter
from datetime import datetime
from pathlib import Path
import hashlib
import json
import re
import shutil
import sys

import yaml

root = Path(sys.argv[1]).resolve()
workspace = Path('/tmp/rmul-original-20261119/workspace')
previous = root.parent / 'c072-fixed-moving-20261120'
reference = previous / 'fixed/cases/left-near-02/stack-evidence'
params_hash = hashlib.sha256((root/'params.yaml').read_bytes()).hexdigest()
fixtures = yaml.safe_load((root/'params.yaml').read_bytes())
expected_controller = fixtures['controller_server']['ros__parameters']
expected_residual = fixtures['small_gicp_relocalization']['ros__parameters']['max_registration_error_per_inlier']
expected_follow_path = expected_controller['FollowPath']
expected_speed = expected_follow_path['max_speed_xy']

def read(path):
    return json.loads(path.read_text()) if path.is_file() else {}

def log_results(path):
    if not path.is_file():
        return []
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.startswith('{')]

def action(path, kind):
    events = log_results(path/f'{kind}.log')
    result = next((x for x in reversed(events) if x['event']=='result'), {})
    stop = read(path/kind/'stop-state.json')
    return {'accepted':any(x['event']=='goal_accepted' for x in events),
            'fixture_passed':any(x['event']=='PASS' for x in events),
            'ros_status':result.get('status'),
            'truth_error_m':result.get('ground_truth_goal_error'),
            'strict_one_second_stop_passed': stop.get('original_1s_stop_passed'),
            'stable_stop_confirmed':stop.get('stable_stop_confirmed')}

def copy_stack(path, rel):
    stack = path/'stack-evidence'
    stack.mkdir(exist_ok=True)
    source = workspace/'logs'/Path(rel).parent.name
    for key in ('robot-red_standard_robot1.sdf','nav2_params.yaml','navigation.log','run-options.json',
                'dependency-runtime.json','goal-preparation.log','resource-hashes.json','rendering-runtime.json','shutdown-audit.json'):
        if (source/key).is_file():
            shutil.copy2(source/key,stack/key)
    epochs=source/'imu-recovery'
    if epochs.is_dir():
        shutil.copytree(epochs,stack/'imu-recovery',dirs_exist_ok=True)
    return stack

def source_health_records(stack):
    marks=[]
    files=list((stack/'imu-recovery').glob('nav-epoch-*.log'))
    for logfile in files or [stack/'navigation.log']:
        if not logfile.is_file():
            continue
        for line in logfile.open(errors='replace'):
            prefix='[red_standard_robot1.mid_odin_view]: '
            if prefix not in line:
                continue
            payload=line.partition(prefix)[2]
            payload=re.sub(r'\x1b\[[\d;]+m','',payload).strip()
            try:
                row=json.loads(payload)
            except ValueError:
                continue
            if row.get('mode')=='costmap_navigation':
                marks.append(row)
    return Counter(v['state'] for v in marks)

def inspect(path):
    session=read(path/'session.json')
    stack = copy_stack(path,session['navigation_log']) if session.get('navigation_log') else path/'stack-evidence'
    health=source_health_records(stack)
    actions={kind:action(path,kind) for kind in ('preposition','goal')}
    topology=read(path/'topology.json').get('passed')
    p=stack/'nav2_params.yaml'
    cfg=yaml.safe_load(p.read_bytes()) if p.is_file() else {}
    controller=cfg.get('controller_server',{}).get('ros__parameters',{})
    ctrl=controller.get('FollowPath',{})
    nav=cfg.get('bt_navigator',{}).get('ros__parameters',{})
    grid=cfg.get('planner_server',{}).get('ros__parameters',{}).get('GridBased',{})
    gicp=cfg.get('small_gicp_relocalization',{}).get('ros__parameters',{})
    lio=cfg.get('point_lio',{}).get('ros__parameters',{})
    smoother=cfg.get('velocity_smoother',{}).get('ros__parameters',{})
    model='robot-red_standard_robot1.sdf'
    input_matches = {
        'original_robot_sdf':(stack/model).is_file() and (reference/model).is_file() and (stack/model).read_bytes()==(reference/model).read_bytes(),
        'c072_3hz_tree_with_map_refresh':Path(nav.get('default_nav_to_pose_bt_xml','')).name=='navigate_field_refresh.xml',
        'configured_behavior_tree':nav.get('default_nav_to_pose_bt_xml')==
              fixtures['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'],
        'c072_base_obstacle_04':ctrl.get('BaseObstacle.scale')==.4 or ctrl.get('BaseObstacle',{}).get('scale')==.4,
        'c072_local_clearance_05':ctrl.get('planning_clearance_radius')==.5 and ctrl.get('planning_cell_coverage') is True,
        'global_plugin':grid.get('plugin')=='rmul_c072_planner/ReserveNavfn',
        'preferred_static_route':grid.get('static_route_preference_m')==
              fixtures['planner_server']['ros__parameters']['GridBased']['static_route_preference_m'],
        'controller_speed':ctrl.get('max_speed_xy')==expected_speed and
              [ctrl.get(k) for k in ('max_vel_x','max_vel_y','min_vel_x','min_vel_y')]==
              [expected_speed,expected_speed,-expected_speed,-expected_speed],
        'smoother_speed':smoother.get('max_velocity',[])[:2]==[expected_speed,expected_speed]
              and smoother.get('min_velocity',[])[:2]==[-expected_speed,-expected_speed],
        'velocity_sampling': all(ctrl.get(key)==expected_follow_path[key]
              for key in ('vx_samples','vy_samples')),
        'goal_checker':controller.get('general_goal_checker',{}).get('xy_goal_tolerance')==
              expected_controller['general_goal_checker']['xy_goal_tolerance'],
        'local_prediction':'rmul_c072_planner::PredictedObstacle' in ctrl.get('critics',[]),
        'swept_clearance':'rmul_c072_planner::SweptClearance' in ctrl.get('critics',[])
              and ctrl.get('rmul_c072_planner::SweptClearance.scale')==160.,
        'controlled_recovery':read(stack/'run-options.json').get('controlled_imu_recovery') is True
              and gicp.get('registration_max_iterations')==64 and gicp.get('metric_correspondences') is True
              and gicp.get('max_registration_error_per_inlier')==expected_residual
              and gicp.get('registration_worker_enabled') is True
              and gicp.get('max_reacquired_pose_jump_m')==
                  fixtures['small_gicp_relocalization']['ros__parameters']['max_reacquired_pose_jump_m']
              and lio.get('max_imu_gap')==.1,
    }
    expected_smoother = fixtures['velocity_smoother']['ros__parameters']
    expected_gicp = fixtures['small_gicp_relocalization']['ros__parameters']
    if 'max_speed_xy' in expected_smoother:
        input_matches['smoother_planar_speed'] = (
            smoother.get('max_speed_xy') == expected_smoother['max_speed_xy'])
    if 'tracking_correspondence_distance_m' in expected_gicp:
        input_matches['bounded_tracking_correspondence'] = (
            gicp.get('tracking_correspondence_distance_m') ==
            expected_gicp['tracking_correspondence_distance_m'])
    if 'recovery_correspondence_distance_m' in expected_gicp:
        input_matches['recovery_correspondence_distance'] = (
            gicp.get('recovery_correspondence_distance_m') ==
            expected_gicp['recovery_correspondence_distance_m'])
    progress_critic = 'rmul_c072_planner::PathProgress'
    if progress_critic in expected_follow_path.get('critics', []):
        input_matches['stationary_path_progress'] = (
            progress_critic in ctrl.get('critics', []) and
            ctrl.get(progress_critic + '.scale') == expected_follow_path[progress_critic + '.scale'])
    recovery=[]
    events=stack/'imu-recovery/events.jsonl'
    if events.is_file():
        for line in events.read_text().splitlines():
            try:
                row=json.loads(line)
            except ValueError:
                continue
            if row['kind'] in ('release_verified','transition','restart_authorized','owned_tree_audit'):
                recovery.append(row)
    contact=(session.get('contact_health') or {}).get('contacts')
    status=session['status']
    return {'case':session['case'],'status':status,'error':session.get('error'),'actions':actions,
            'obstacle_goal_started':session.get('goal_started_wall') is not None,
            'opponent_movement_started':session.get('opponent_move_started_sim') is not None,
            'opponent_fitted_speed_mps':session.get('opponent_motion_regression_speed_mps'),
            'contacts':contact,
            'minimum_gap_m':session.get('opponent_min_gap_m',session.get('minimum_clearance_m')),
            'goal_robot_speed_peak_mps':session.get('robot_goal_speed_peak_mps'),
            'topology_passed':topology,'params_match':session.get('params_sha256')==params_hash,
            'gate_state_transition_counts':health,'configuration_checks':input_matches, 'recovery_events':recovery,
            'process_remaining':(session.get('process_cleanup') or {}).get('remaining'),
            'started_at':session['started_at'],'ended_at':session.get('ended_at')}

stages={}
diagnostics=sorted(p.name for p in root.iterdir()
                   if p.is_dir() and (p.name.startswith('moving-diagnostic') or
                                      p.name.startswith(('fixed-tf-diagnostic', 'fixed-diagnostic'))))
for role in (*diagnostics,'fixed-left','fixed-reverse-right','moving-standard'):
    stage=root/role/'cases'
    paths=[p for p in sorted(stage.glob('*/session.json')) if read(p).get('status') in ('passed','failed') and read(p).get('ended_at')]
    if not paths:
        continue
    rows=[inspect(path.parent) for path in paths]
    accepted=[row for row in rows if row['status']=='passed']
    for row in accepted:
        assert all(row['configuration_checks'].values()),row['configuration_checks']
        assert row['gate_state_transition_counts']['ready']>0
        assert row['params_match'] and row['process_remaining']==[],row['case']
        if not role.startswith('moving-diagnostic'):
            assert row['topology_passed'],row['case']
        assert row['contacts']==0
        assert all(action['fixture_passed'] and action['ros_status']==4 and
                   action['strict_one_second_stop_passed'] and action['stable_stop_confirmed']
                   for action in row['actions'].values())
    stages[role]={'completed':len(rows),'passed':len(accepted),'results':rows}
    for row in rows:
        print(role,row['case'],row['status'],row['error'],'goal',row['obstacle_goal_started'],
              'contacts',row['contacts'],'min_gap',row['minimum_gap_m'])

fixed_receipt=read(root/'fixed-acceptance.json')
moving_receipt=read(root/'moving-standard/cases/moving-acceptance.json')
full= bool(fixed_receipt) and bool(moving_receipt)
summary={'time':datetime.now().astimezone().isoformat(),'configured_params_sha256':params_hash,
    'completed_standard_round':full,'qualified_standard_round':full and fixed_receipt['passed'] and moving_receipt['passed'],
    'stages':stages}
(root/'qualification-summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
