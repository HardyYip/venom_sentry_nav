import argparse
import collections
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

P = Path(__file__).resolve().parents[2]
W = P.parents[1]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def audit(root):
    checks = []
    navlogs = set()
    events = []
    stats = {}
    def check(name, condition, **facts):
        if not condition:
            raise AssertionError((name, facts))
        checks.append(dict(name=name, passed=True, **facts))
    def load(path):
        return json.loads(Path(path).read_text())
    for name, count in [('low-speed-02',22),('crossfield-01',20),('imu-02',16),('timing-01',17),('persistent-01',6)]:
        value = load(root/name/'result.json')
        check(name, value['passed'] and len(value['checks']) >= count,
              count=len(value['checks']))
        if value.get('navigation_log'):
            navlogs.add(Path(value['navigation_log']).parent)
        if value.get('navigation_recovery_logs'):
            recovery = Path(value['navigation_recovery_logs'])
            navlogs.add(recovery.parent)
        events.extend((root/name).glob('*events.jsonl'))
    for number in range(1,4):
        name = 'round-'+str(number)
        value = load(root/name/'result.json')
        item = next(c for c in value['checks'] if c['name']=='five_minutes_real_crossfield_patrol_retreat_resume')
        check(name, value['passed'] and item['simulation_duration_s']>=300 and item['successful_goals']>=6,
              simulation_duration_s=item['simulation_duration_s'],wall_duration_s=item['wall_duration_s'],successful_goals=item['successful_goals'])
        navlogs.add(Path(value['navigation_log']).parent)
        events.extend((root/name).glob('*events.jsonl'))
    action = load(root/'action-stub-01/results.json')
    check('action_stub_22',len(action)==22 and all(v['passed'] for v in action))
    check('original_ctest_4','100% tests passed, 0 tests failed out of 4' in (root/'ctest-original.log').read_text())
    low = load(root/'low-box-25cm-02/left-near-01/session.json')
    check('ground_25cm_geometry',low['ground_box_geometry']==dict(height=.25,center_z=.125,bottom_z=0.))
    check('ground_25cm_detection_and_clearance',low['passed'] and low['goal_exit_code']==0 and low['after_clear_navigation_exit_code']==0 and
          all(v['lethal_cells']==0 and v['center']==0 and v['max']==0 for v in low['cleared_maps'].values()))
    navlogs.add(Path(low['navigation_log']).parent)
    fixed = load(root/'fixed-01/static-acceptance.json')
    moving = load(root/'moving-01/moving-acceptance.json')
    check('fixed_12',fixed['passed'] and fixed['cases']==12)
    check('moving_3_same_config',moving['passed'] and moving['cases']==3 and moving['params_sha256']==fixed['params_sha256'])
    for name,data in [('fixed',fixed),('moving',moving)]:
        for trial in data['results']:
            check(name+'_'+trial['case'],trial['passed'] and not trial['process_cleanup']['remaining'] and trial['contact_health']['contacts']==0)
            navlogs.add(Path(trial['navigation_log']).parent)
        gaps = [c['opponent_min_gap_m'] if name=='moving' else c['minimum_clearance_m'] for c in data['results']]
        stats[name+'_minimum_gap_m'] = min(gaps)
        check(name+'_positive_clearance',min(gaps)>.05)
    direct_trials = [root/'low-box-25cm-02/left-near-01']
    direct_trials += [root/'fixed-01'/c['case'] for c in fixed['results']]
    direct_trials += [root/'moving-01'/c['case'] for c in moving['results']]
    direct_count = 0
    direct_max_error = 0.
    for trial in direct_trials:
        names = ['preposition','goal'] + (['after-clear-return'] if trial == direct_trials[0] else [])
        for name in names:
            rows = []
            for line in (trial/(name+'.log')).read_text().splitlines():
                if line.startswith('{'):
                    rows.append(json.loads(line))
            accepted = [v for v in rows if v.get('event')=='goal_accepted']
            results = [v for v in rows if v.get('event')=='result']
            check('direct_action_'+str(trial.relative_to(root))+'_'+name,
                  len(accepted)==len(results)==1 and results[0]['status']==4 and
                  results[0]['goal_error']<=.35 and results[0]['ground_truth_goal_error']<=.35 and
                  results[0]['elapsed_wall_s']<=180 and results[0]['stop_state']['original_1s_stop_passed'])
            direct_count += 1
            direct_max_error = max(direct_max_error,results[0]['ground_truth_goal_error'])
    stats['direct_action_successes'] = direct_count
    stats['direct_action_max_truth_error_m'] = direct_max_error
    for f in ['live-topology-01.json','mechanics-invariance.json','model-contract.json','ros-contract-01/result.json','python-build-result.json']:
        check(f,load(root/f)['passed'])
    for path,sha in load(root/'python-build-result.json')['files'].items():
        check('current_python_'+Path(path).name,digest(path)==sha)
    replay = P/'rmul2027_decision/tools/replay_trace.py'
    event_stats = collections.Counter()
    for path in sorted(set(events)):
        result = subprocess.run(['python3',str(replay),str(path)],capture_output=True,text=True,check=True)
        value = json.loads(result.stdout)
        check('event_'+str(path.relative_to(root)),value['passed'] and value['unresolved_token'] is None)
        rows = [json.loads(s) for s in path.read_text().splitlines() if s.strip()]
        event_stats['sends'] += value['sends']
        event_stats['terminals'] += value['terminals']
        for row in rows:
            if row['kind']=='action_result':event_stats['terminal_code_'+str(row['code'])]+=1
    check('all_real_goals_terminated',event_stats['sends']==event_stats['terminals'],**event_stats)
    failed = load(root/'imu-01/result.json')
    result = subprocess.run(['python3',str(replay),str(root/'imu-01/decision-events.jsonl')],capture_output=True,text=True,check=True)
    failed_replay = json.loads(result.stdout)
    check('failed_imu_attempt_retained_with_closed_ownership',not failed['passed'] and failed_replay['unresolved_token'] is None and failed_replay['sends']==failed_replay['terminals'])
    rows = [json.loads(line) for line in (root/'low-box-25cm-01/left-near-01/preposition.log').read_text().splitlines() if line.startswith('{')]
    accepted = [v for v in rows if v.get('event')=='goal_accepted']
    terminal = [v for v in rows if v.get('event')=='result']
    check('failed_direct_attempt_retained_with_aborted_terminal',len(accepted)==len(terminal)==1 and terminal[0]['status']==6 and not load(root/'low-box-25cm-01/left-near-01/session.json')['passed'])

    for log in sorted(navlogs):
        manifest = load(log/'sensor-profile.json')
        for path,sha in manifest['files'].items():
            check('profile_identity_'+str(log.name)+'_'+Path(path).name,digest(path)==sha)
        robot = (log/'robot-red_standard_robot1.sdf').read_text()
        check('actual_model_'+log.name,'aux_mid360_lidar' in robot and 'front_mid360_lidar' in robot and 'rplidar' not in robot)
        check('actual_dds_'+log.name,load(log/'dds-runtime.json')['profile'].endswith('fastdds_local_udp_4m.xml'))
    critical = {
        W/'src/pb2025_robot_description/resource/xmacro/simulation_robot.sdf.xmacro':'22f6a6cad6e1b1ba5e0d5f9713220615580aa37ad978bc7e96379e84f7170e2d',
        W/'src/pb2025_robot_description/resource/models/mid360/model.sdf.xmacro':'e4d75370281553b12cc1e71a0dbe7c7b5fc8a8a27abb6ab8695be5b534647dfe',
        W/'src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/ros_gz_bridge.yaml':'6d6253bd4f3f536b05222877984e26fd536401534189932c6656366062a24621',
        P/'scripts/rmul2027/config/nav2_test.yaml':'3dd610e809375a29f5467790fb00bcccdfb03666d569b4c0fa312088e7df3a10',
        W/'install/pb_omni_pid_pursuit_controller/lib/libpb_omni_pid_pursuit_controller.so':'bf6b4c496aa4713f4615e38d41ded64bb6fbc13c035678aa8cb02fc3abdbb611'}
    for path,sha in critical.items():
        check('original_preserved_'+str(path),digest(path)==sha)
    def library_checks(value):
        if isinstance(value,dict):
            if 'libraries' in value and isinstance(value['libraries'],dict):
                for path,sha in value['libraries'].items():
                    check('epoch_library_'+path,Path(path).is_file() and digest(path)==sha)
            if 'exe_sha256' in value and value.get('command'):
                path=value['command'][0]
                check('epoch_executable_'+path,Path(path).is_file() and digest(path)==value['exe_sha256'])
            if 'path' in value:
                sha=value.get('sha256',value.get('library_sha256'))
                if sha and Path(value['path']).is_file():
                    check('runtime_library_'+value['path'],digest(value['path'])==sha)
            for child in value.values():library_checks(child)
        elif isinstance(value,list):
            for child in value:library_checks(child)
    for pattern in ['**/actual-runtime.json','**/runtime-epoch-*.json']:
        for path in root.glob(pattern):library_checks(load(path))
    from storage_guard import tree_bytes
    owned_logs = []
    for manifest in (W/'logs').glob('*/sensor-profile.json'):
        value = load(manifest)
        if Path(value['prefix']).is_relative_to(root):
            owned_logs.append(manifest.parent)
    combined = tree_bytes(root)+sum(tree_bytes(path) for path in owned_logs)
    check('combined_task_evidence_and_logs_budget',combined<4*1024**3,bytes=combined,owned_log_directories=len(owned_logs))
    guard_names = ['low-speed-02','crossfield-01','low-box-25cm-02','fixed-01','moving-01','imu-02','timing-01','persistent-01','round-1','round-2','round-3']
    for name in guard_names:
        rows = [json.loads(line) for line in (root/(name+'-guard.log')).read_text().splitlines() if line.startswith('{')]
        start = next(row for row in rows if row.get('kind')=='start')
        final = [row for row in rows if row.get('kind')=='finished'][-1]
        policy = start['policy']
        check('guard_'+name,final['child_returncode']==0 and final['artifact_bytes']<512*1024**2 and policy['start_free_gib']==50 and policy['stop_free_gib']==30 and policy['profiles']['light']['max_run_mib']==512 and policy['profiles']['light']['max_seconds']==1800 and policy['max_host_growth_gib']==2,
              bytes=final['artifact_bytes'])
    check('conversion_logic_22','Ran 22 tests' in (root/'unit-01.log').read_text() and '\nOK' in (root/'unit-01.log').read_text())
    free={s:shutil.disk_usage('/mnt/'+s).free for s in ['c','d','e']}
    check('disk_above_stop',all(v>=30*1024**3 for v in free.values()),free_bytes=free)
    return dict(passed=True,scope='C070 finite simulation-only functional auxiliary MID replacement',checks=checks,stats=stats,events=dict(event_stats),nav_runs=len(navlogs),
        limits=dict(real_vehicle=False,orin=False,formal_referee=False,fast=False,new_localization=False,auxiliary_dropout_interlock=False),
        preserved_failures=['model-contract-01 numeric-string mismatch','low-speed-01 navigation base path','low-box-25cm-01 missing TEST direct-Action heartbeat','imu-01: post-recovery route failed twice; root cause and arbitrary-location stability remain unresolved'])

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    value=audit(args.root.resolve())
    args.output.write_text(json.dumps(value,ensure_ascii=False,indent=2))
    print(json.dumps(dict(passed=value['passed'],checks=len(value['checks']),stats=value['stats'],events=value['events'],nav_runs=value['nav_runs'])))
