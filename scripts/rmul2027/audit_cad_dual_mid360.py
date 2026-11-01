"""Final C072 audit; requires completed accepted runs, never starts simulation."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

P=Path(__file__).resolve().parents[2]
W=P.parents[1]

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def audit(root):
    accepted=json.loads((root/'accepted-attempts.json').read_text())
    checks=[];navlogs=set();events=set();statistics={}
    def load(path):
        return json.loads(Path(path).read_text())
    def check(name,value,**facts):
        if not value:raise AssertionError((name,facts))
        checks.append(dict(name=name,passed=True,**facts))
    def run(role,count):
        path=root/accepted[role];data=load(path/'result.json')
        check(role,data['passed'] and len(data['checks'])>=count,count=len(data['checks']))
        if data.get('navigation_log'):navlogs.add(Path(data['navigation_log']).parent)
        if data.get('navigation_recovery_logs'):navlogs.add(Path(data['navigation_recovery_logs']).parent)
        events.update(path.glob('*events.jsonl'))
        return data
    for role,count in [('stationary',5),('motion',20),('topology',6),('low',22),('crossfield',20),('imu',16),('imu_confirmation',16),('timing',17),('timing_confirmation',17),('persistent',6)]:
        run(role,count)
    check('independent_imu_confirmation',accepted['imu']!=accepted['imu_confirmation'] and load(root/accepted['imu']/'result.json')['navigation_recovery_logs']!=load(root/accepted['imu_confirmation']/'result.json')['navigation_recovery_logs'])
    check('independent_timing_confirmation',accepted['timing']!=accepted['timing_confirmation'] and load(root/accepted['timing']/'result.json')['navigation_recovery_logs']!=load(root/accepted['timing_confirmation']/'result.json')['navigation_recovery_logs'])
    check('three_independent_rounds',len({accepted['round'+str(i)] for i in range(1,4)})==3)
    for number in range(1,4):
        data=run('round'+str(number),1)
        records=[json.loads(line) for line in (root/accepted['round'+str(number)]/'decision-events.jsonl').read_text().splitlines()]
        actual_succeeded=[row['goal'] for index,row in enumerate(records) if row.get('kind')=='send' and any(x.get('kind')=='action_result' and x.get('token')==row['token'] and x['code']==1 for x in records[index+1:])]
        completed_patrol=[g for g in actual_succeeded if g['key'].startswith('patrol:')]
        check('round_real_field_span_'+str(number),len({(g['x'],g['y']) for g in completed_patrol})>=3 and max(g['x'] for g in completed_patrol)-min(g['x'] for g in completed_patrol)>=7.5)
        restored=False
        for index,row in enumerate(records):
            if row.get('kind')!='send' or row['goal']['key']!='retreat':continue
            ends=[j for j,x in enumerate(records) if j>index and x.get('kind')=='action_result' and x.get('token')==row['token'] and x['code']==1]
            if ends and any(x.get('kind')=='send' and x['goal']['key'].startswith('patrol:') and any(y.get('kind')=='action_result' and y.get('token')==x['token'] and y['code']==1 for y in records[j+1:]) for j,x in enumerate(records) if j>ends[0]):restored=True
        check('round_actual_retreat_then_restored_patrol_'+str(number),restored and 0<data['retreat_trigger_sim']<data['retreat_completed_sim'])

        value=next(x for x in data['checks'] if x['name']=='five_minutes_real_crossfield_patrol_retreat_resume')
        check('round_duration_'+str(number),value['simulation_duration_s']>=300 and value['successful_goals']>=6,**{k:v for k,v in value.items() if k not in ('name','passed')})
    for name in ['map-frame-contract.json','coarse-envelope-check.json','boundary-map-contract.json','cell-cover3-model-config-contract.json']:
        check(name,load(root/name)['passed'])
    model_contract=load(root/'cell-cover3-model-config-contract.json')
    check('explicit_thirteen_parameter_changes',len(model_contract['params_delta'])==13 and len(model_contract['changed_mount_blocks'])==2 and len(model_contract['native_models'])==2 and all(x['physical_pose_mass_collision_unchanged'] for x in model_contract['native_models']))
    for name,count in [('strict-tf-02/result.json',11),('aux-ros-contract-01/result.json',10)]:
        data=load(root/name)
        check(name,data['passed'] and len(data['checks'])==count)
    stubs=load(root/'action-stub-01/results.json')
    check('action_stub_22',len(stubs)==22 and all(x['passed'] for x in stubs))
    check('decision_ctest_4','100% tests passed, 0 tests failed out of 4' in (root/'ctest-01.log').read_text())
    check('mount_map_logic_30','Ran 30 tests' in (root/'cell-cover-mount-logic-01.log').read_text() and '\nOK' in (root/'cell-cover-mount-logic-01.log').read_text())
    check('old_aux_logic_22','Ran 22 tests' in (root/'c070-logic-01.log').read_text() and '\nOK' in (root/'c070-logic-01.log').read_text())
    check('native_scan_logic_7','Ran 7 tests' in (root/'native-frame-logic-01.log').read_text() and '\nOK' in (root/'native-frame-logic-01.log').read_text())
    check('cell_cover_planning_geometry_6','Ran 6 tests' in (root/'trajectory4-planning-logic-final-01.log').read_text() and '\nOK' in (root/'trajectory4-planning-logic-final-01.log').read_text())
    feasibility=load(root/'coherent-planning-feasibility-01.json')
    check('coherent_planning_offline_bound',feasibility['passed'] and feasibility['old_kernel_accepts'] and feasibility['new_planning_kernel_rejects'] and feasibility['chosen_planning_radius_m']==.5)
    check('coherent_fixed_boxes_offline_connectivity',load(root/'coherent-fixed-box-feasibility-01.json')['passed'])
    cell_cover=load(root/'cell-cover-offline-feasibility-01.json')
    check('complete_cell_cover_original_routes_still_connected',cell_cover['passed'] and cell_cover['trajectory_radius_m']==.5 and abs(cell_cover['goal_field_cell_cover_radius_m']-(.5+math.sqrt(2)*.05/2))<1e-12 and len(cell_cover['normal']['goals'])==13 and cell_cover['fixed_boxes']['passed'])
    native_ros=load(root/'native-frame-ros-01/result.json')
    check('native_scan_ROS_7',native_ros['passed'] and len(native_ros['checks'])==7)
    normal_sources=load(root/'runtime-source-freeze.json');recovery_sources=load(root/'recovery-source-sha256.json')
    qualified_sources=load(root/'qualification-public-source-sha256.json')
    now_sources={**normal_sources,**recovery_sources}
    normalization=load(root/'source-handoff-normalization.json')
    test_file='scripts/rmul2027/tests/cad_planning_clearance_test.py'
    source_deltas=[name for name in now_sources if now_sources[name]!=qualified_sources[name]]
    check('qualified_materials_preserved_except_test_EOF',source_deltas==[test_file] and normalization['passed'] and normalization['pure_logic_retested']==6 and not normalization['recompiled'] and not normalization['resimulated'])
    original=(root/'qualification-public-source-46'/test_file).read_bytes()
    check('test_EOF_only_exact_bytes',(P/test_file).read_bytes()==original.rstrip(b'\n')+b'\n' and digest(root/'qualification-public-source-46'/test_file)==qualified_sources[test_file])
    check('private_compilation_inputs_unchanged_after_style',normalization['private_materials']==[dict(family='controller',private_compilation_input_files=31,byte_identical=True),dict(family='GICP',private_compilation_input_files=20,byte_identical=True)])
    for filename,expected in normalization['library_sha256'].items():check('style_cleanup_library_identity_'+filename,digest(filename)==expected)
    check('explicit_separate_runtime_sources',len(normal_sources)==28 and len(recovery_sources)==18 and not set(normal_sources).intersection(recovery_sources))
    for path,sha in normal_sources.items():
        check('runtime_frozen_'+path,digest(P/path)==sha)
    for path,sha in load(root/'recovery-source-sha256.json').items():
        check('recovery_frozen_'+path,digest(P/path)==sha)
    check('recovery_parameters_equal_legacy_except_budget_and_target_sampling','Ran 5 tests' in (root/'metric1-recovery-profile-logic-01.log').read_text() and '\nOK' in (root/'metric1-recovery-profile-logic-01.log').read_text())
    from prepare_cad_gicp import verify as verify_gicp
    gicp_manifest=verify_gicp(W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072/gicp-metric1-source/small_gicp_relocalization')
    check('recovery_gicp_source_reproducible',len(gicp_manifest['private_files_sha256'])==20 and load(root/'cad-gicp-source-reproducibility.json')['passed'] and gicp_manifest['dependency_git']=='fa0cfc983417c044b6036f71dfa448038bb9c820')
    cad_gicp_receipt=load(root/'cad-gicp-library.json');cad_gicp=Path(cad_gicp_receipt['path'])
    check('recovery_private_GICP',digest(cad_gicp)==cad_gicp_receipt['sha256']=='c4e292caca5d8d1f207a400f12f9af65174677092fcc79e4e42eb94111464062' and cad_gicp_receipt['default_iterations']==10 and cad_gicp_receipt['tested_iterations']==64 and cad_gicp_receipt['max_iterations']==64 and cad_gicp_receipt['criteria_unchanged'] and cad_gicp_receipt['deterministic_global_sampling'] and cad_gicp_receipt['quality_body_byte_identical'] and cad_gicp_receipt['metric_correspondences'] and cad_gicp_receipt['correspondence_neighbor_cap']==8 and cad_gicp_receipt['correspondence_extra_distance_m']==.025 and cad_gicp_receipt['target_leaf_size_m']==.02)
    import xml.etree.ElementTree as ET
    files=list((W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072/gicp-metric1-build/small_gicp_relocalization/test_results').rglob('test_registration_budget.gtest.xml'))
    cases=[case for file in files for case in ET.parse(file).getroot().iter('testcase')]
    check('recovery_budget_cpp4',len(cases)==4 and all(case.find('failure') is None and case.find('error') is None for case in cases))
    sampling_files=list((W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072/gicp-metric1-build/small_gicp_relocalization/test_results').rglob('test_global_sampling.gtest.xml'))
    sampling_cases=[case for file in sampling_files for case in ET.parse(file).getroot().iter('testcase')]
    check('full_voxel_sampling_cpp5',len(sampling_cases)==5 and all(case.find('failure') is None and case.find('error') is None for case in sampling_cases))
    metric_files=list((W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072/gicp-metric1-build/small_gicp_relocalization/test_results').rglob('test_metric_correspondence.gtest.xml'))
    metric_cases=[case for file in metric_files for case in ET.parse(file).getroot().iter('testcase')]
    check('metric_correspondence_cpp6',len(metric_cases)==6 and all(case.find('failure') is None and case.find('error') is None for case in metric_cases))
    product=load(root/'metric1-product-replay-01.json')
    check('public_factor70_replay',product['passed'] and product['total_cases']==70 and product['historical_cases']==56 and product['fresh_actual_failed_cases']==14 and product['actual_budget']==64 and product['original_criteria_unchanged'] and product['truth_max_xy_error_m']<=.35 and product['public_factor_sha256']==digest(P/'scripts/rmul2027/overlays/metric_correspondence.hpp'))
    preprocessing=load(root/'exact-native-prior-04.json')
    full_cases=[row for row in preprocessing['results'] if row['mode']=='serial02']
    check('historical_sampling56_preserved_not_final_metric_evidence',len(full_cases)==56 and preprocessing['summary']['serial02']['converged']==56 and {r['input'] for r in full_cases}==set(range(8)) and {r['seed_case'] for r in full_cases}==set(range(7)) and all(r['converged'] and 1<=r['iterations']<=32 and r['inliers']>0 and math.isfinite(r['error_per_inlier']) and r['error_per_inlier']<=.5 and r['hessian_ratio']>=1e-6 for r in full_cases))
    expected_target_points=full_cases[0]['target_points']
    check('deterministic_target_point_count',len({r['target_points'] for r in full_cases})==1 and expected_target_points<=152326)
    exact=load(root/'exact-replay-01-summary.json')['results']
    failing=[r for r in exact if not r['original']['converged'] and r['replay']['budget']==32]
    check('captured_real_nonconvergence_reproduced',len(failing)==4 and all(not r['replay']['converged'] and r['replay']['iterations']==r['original']['optimizer_iterations']==32 and r['replay']['source_points']==r['original']['source_points'] and r['replay']['inliers']==r['original']['inliers'] and abs(r['replay']['error_per_inlier']-r['original']['error_per_inlier'])<1e-12 for r in failing))
    check('extra_iterations_alone_do_not_fix_cycle',sum(not r['replay']['converged'] and not r['original']['converged'] and r['replay']['budget']==64 for r in exact)==2)
    for row in load(root/'exact-gicp-window-01-input-manifest.json')['windows']:
        for path,receipt in row['files'].items():check('exact_input_preserved_'+path,digest(path)==receipt['sha256'])
    evidence=load(root/'gicp-offline-iteration-comparison.json')['results']
    check('real_windows_iteration_limit_evidence',len(evidence)==2 and all(not next(x for x in row['replay'] if x['budget']==10)['converged'] and next(x for x in row['replay'] if x['budget']==10)['iterations']==10 and next(x for x in row['replay'] if x['budget']==32)['converged'] and 10<next(x for x in row['replay'] if x['budget']==32)['iterations']<=32 and all(x['source_points']==row['actual_quality']['source_points'] and x['inliers']==row['actual_quality']['inliers'] for x in row['replay']) for row in evidence))
    sampling=load(root/'gicp-offline-target-resolution-comparison.json')['results']
    denser=[x for x in sampling if x['target_leaf']==.05]
    check('historical_approximate_denser_comparison_preserved',len(denser)==4 and all(x['source_leaf']==.05 and x['replay']['converged'] and x['replay']['iterations']<=32 for x in denser))
    check('coarse_target_cycle_reproduced',any(x['kind']=='imu' and x['target_leaf']==.1 and not x['replay']['converged'] and x['replay']['iterations']==32 for x in sampling))
    check('original_recovery_and_decision_core_unchanged',not subprocess.check_output(['git','diff','a2aaa651e36e73e979866f4852ebd8d1f4edc2aa','--','rmul2027_decision/recovery','rmul2027_decision/src','rmul2027_decision/include','rmul2027_decision/config','rmul2027_decision/behavior_trees'],cwd=P))
    ground=root/accepted['ground']
    low=load(ground/'left-near-01/session.json')
    check('ground_25cm_complete',low['passed'] and low['ground_box_geometry']==dict(height=.25,center_z=.125,bottom_z=0.) and low['goal_exit_code']==0 and low['after_clear_navigation_exit_code']==0)
    check('ground_maps_clear',all(v['lethal_cells']==0 and v['center']==v['max']==0 for v in low['cleared_maps'].values()))
    navlogs.add(Path(low['navigation_log']).parent)
    direct=[ground/'left-near-01']
    fixed=load(root/accepted['fixed']/'static-acceptance.json')
    moving=load(root/accepted['moving']/'moving-acceptance.json')
    batches=fixed.get('source_batches',[accepted['fixed']])
    originals=[load(root/batch/'static-acceptance.json') for batch in batches]
    check('fixed_batch_provenance',sum(batch['cases'] for batch in originals)==12 and all(batch['passed'] and batch['params_sha256']==fixed['params_sha256'] for batch in originals) and fixed['results']==[trial for batch in originals for trial in batch['results']],batches=batches)
    check('fixed_12',fixed['passed'] and fixed['cases']==12)
    check('moving_3_same_parameters',moving['passed'] and moving['cases']==3 and fixed['params_sha256']==moving['params_sha256'])
    lower=load(root/accepted['lower_route']/'static-acceptance.json')
    check('additional_lower_three_same_parameters',lower['passed'] and lower['cases']==3 and lower['params_sha256']==fixed['params_sha256'])
    for trial in lower['results']:
        check('lower_entry_actual_action_'+trial['case'],trial['lower_entry_goal']==[3.,-6.4] and trial['lower_entry_exit_code']==0 and trial['lower_entry_contact_health']['contacts']==0 and trial['lower_entry_stop_state']['classification']=='safety_stop')
        check('lower_original_final_goal_'+trial['case'],trial['start']==[1.0,-6.2] and trial['goal']==[5.0,-5.5] and trial['boxes']==[[3.0,-5.5],[6.6,-.2]])
        rows=[json.loads(line) for line in (root/accepted['lower_route']/trial['case']/'statistics.jsonl').read_text().splitlines()]
        route=[r['truth_map'][1] for r in rows if r['kind']=='motion' and r['phase']=='goal' and 2.5<=r['truth_map'][0]<=3.5]
        check('lower_actual_truth_route_'+trial['case'],bool(route) and max(route)<-5.75,minimum_y=min(route) if route else None,maximum_y=max(route) if route else None)
    for name,data in [('fixed',fixed),('moving',moving),('lower_route',lower)]:
        gaps=[]
        for trial in data['results']:
            check(name+'_'+trial['case'],trial['passed'] and not trial['process_cleanup']['remaining'] and trial['contact_health']['contacts']==0)
            gap=trial['opponent_min_gap_m'] if name=='moving' else trial['minimum_clearance_m']
            gaps.append(gap);navlogs.add(Path(trial['navigation_log']).parent)
            direct.append(root/accepted[name]/trial['case'])
        check(name+'_clearance',min(gaps)>.05,minimum_gap_m=min(gaps))
        statistics[name+'_minimum_gap_m']=min(gaps)
    for trial in direct:
        names=['preposition','goal']+(['after-clear-return'] if trial==direct[0] else [])+(['lower-entry'] if (trial/'lower-entry.log').is_file() else [])
        for name in names:
            rows=[json.loads(line) for line in (trial/(name+'.log')).read_text().splitlines() if line.startswith('{')]
            requests=[x for x in rows if x.get('event')=='goal_accepted']
            results=[x for x in rows if x.get('event')=='result']
            check('direct_'+str(trial.relative_to(root))+'_'+name,len(requests)==len(results)==1 and results[0]['status']==4 and results[0]['goal_error']<=.35 and results[0]['ground_truth_goal_error']<=.35 and results[0]['elapsed_wall_s']<=180 and results[0]['stop_state']['original_1s_stop_passed'])
    all_direct=collections.Counter()
    for path in root.rglob('*.log'):
        if path.name not in ('preposition.log','goal.log','after-clear-return.log','lower-entry.log'):continue
        rows=[json.loads(line) for line in path.read_text().splitlines() if line.startswith('{')]
        owned=[row for row in rows if row.get('event')=='goal_accepted']
        if not owned:continue
        terminal=[row for row in rows if row.get('event')=='result']
        check('all_direct_ownership_'+str(path.relative_to(root)),len(owned)==len(terminal)==1 and terminal[0]['status'] in (4,5,6),uuid=owned[0]['goal_uuid'],status=terminal[0]['status'] if terminal else None)
        all_direct['accepted']+=1;all_direct['terminal']+=1;all_direct['status_'+str(terminal[0]['status'])]+=1
    statistics['all_direct_including_preserved_candidates']=dict(all_direct)
    replay=P/'rmul2027_decision/tools/replay_trace.py'
    totals=collections.Counter();current_totals=collections.Counter()
    # Include failed decision attempts as well; ownership must close even when tests fail.
    for candidate in root.glob('*/*events.jsonl'):
        metadata=candidate.parent/'result.json'
        if metadata.exists():
            value=load(metadata)
            if value.get('navigation_log') or value.get('navigation_recovery_logs'):
                events.add(candidate)
    for path in sorted(events):
        data=json.loads(subprocess.check_output(['python3',str(replay),str(path)],text=True))
        check('ownership_'+str(path.relative_to(root)),data['passed'] and data['unresolved_token'] is None)
        totals['sends']+=data['sends'];totals['terminals']+=data['terminals']
        if path.parent.name in set(accepted.values()):
            current_totals['sends']+=data['sends'];current_totals['terminals']+=data['terminals']
        for line in path.read_text().splitlines():
            if not line:continue
            row=json.loads(line)
            if row.get('kind')=='action_result':
                totals['terminal_'+str(row['code'])]+=1
                if path.parent.name in set(accepted.values()):current_totals['terminal_'+str(row['code'])]+=1
    check('all_goals_terminal',totals['sends']==totals['terminals'],**totals)
    private=W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072/install/sensor_scan_generation/lib/libsensor_scan_generation.so'
    check('strict_private_library',digest(private)=='4b628719df9ac7ed37c2d95d67f429749aa93c9886c21a767f8b27d6872c32a7')
    controller_receipt=load(root/'controller-library-trajectory4.json')
    controller_private=Path(controller_receipt['path'])
    build_guard=[json.loads(line) for line in (root/'controller-trajectory4-build-01.log').read_text().splitlines() if line.startswith('{')]
    build_end=[row for row in build_guard if row.get('kind')=='finished']
    check('controller_independent_build_guard0',bool(build_end) and build_end[-1]['child_returncode']==0)
    check('continuous_trajectory_reserve_opt_in',controller_receipt['continuous_trajectory_cell_reserve'] and controller_receipt['inside_reserve_outward_only'] and controller_receipt['cpp_tests']==35)
    counterexample=load(root/'cell-cover3-mid-trajectory-counterexample.json')
    check('old_mid_trajectory_gap_recorded',counterexample['prepared']==1 and counterexample['legacy_trajectory_clear']==1 and counterexample['minimum_center_distance']<counterexample['covered_radius'] and counterexample['endpoint_score']<counterexample['unreachable'])
    check('private_projection_controller',digest(controller_private)==controller_receipt['sha256']=='2e64a25111879d4b9a6ca16dc9be4107fab87aab4c583c92f96d588c596dafc0')
    from prepare_cad_controller import verify
    manifest=verify(W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072/controller-trajectory4-source/pb_omni_pid_pursuit_controller')
    check('private_source_reproducible',load(root/'controller-source-reproducibility-trajectory4.json')['passed'] and len(manifest['private_files_sha256'])==31)
    import xml.etree.ElementTree as ET
    testroot=W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072/controller-trajectory4-build/pb_omni_pid_pursuit_controller/test_results'
    for target,required in [('test_pose_projection',4),('test_geodesic_goal_field',14),('test_planning_clearance',4),('test_goal_field_cell_coverage',13)]:
        files=list(testroot.rglob(target+'.gtest.xml'))
        cases=[case for file in files for case in ET.parse(file).getroot().iter('testcase')]
        check(target,len(cases)>=required and all(case.find('failure') is None and case.find('error') is None for case in cases),cases=len(cases))
    actual_maps=[json.loads(line) for file in [root/'actual-private-library-observations.jsonl',root/'actual-private-library-observations-native-final.jsonl',root/'actual-private-library-observations-coherent.jsonl',root/'actual-private-library-observations-final.jsonl',root/'actual-private-library-observations-cell-cover3.jsonl',root/'actual-private-library-observations-mapstable1.jsonl',root/'actual-private-library-observations-metric1.jsonl',root/'actual-private-library-observations-trajectory4.jsonl',root/'actual-private-library-observations-trajectory4-resume-01.jsonl',root/'actual-private-library-observations-trajectory4-lower.jsonl',root/'actual-private-library-observations-trajectory4-lower-via-02.jsonl'] for line in file.read_text().splitlines() if line]
    check('actual_private_map_observer_has_coverage',bool(actual_maps))
    topology=load(root/accepted['motion']/'live-topology.json')
    check('actual_topology_contract6',topology['passed'] and len(topology['checks'])==6)
    for log in sorted(navlogs):
        sensor=load(log/'sensor-profile.json')
        check('profile_'+log.name,sensor['profile']=='cad-dual-mid360' and sensor['coarse_robot_unchanged'])
        for path,sha in sensor['files'].items():
            check('source_'+log.name+'_'+Path(path).name,digest(path)==sha)
        check('prior_frame_'+log.name,sensor['prior_map_import']['encoding'].startswith('base/map coordinates'))
        selected=load(log/'selected-map.json')
        boundary=sensor['occupancy_boundary_guard']
        check('actual_boundary_map_'+log.name,boundary['cells']==1 and selected['yaml']==boundary['yaml'] and selected['image']==boundary['image'])
        check('boundary_image_identity_'+log.name,digest(selected['image'])==selected['image_sha256'])
        model=(log/'robot-red_standard_robot1.sdf').read_text()
        check('two_actual_MID_'+log.name,'front_mid360_lidar' in model and 'aux_mid360_lidar' in model and 'rplidar' not in model)
        import xml.etree.ElementTree as ET
        sdf=ET.fromstring(model)
        gpu=sdf.findall(".//sensor[@type='gpu_lidar']")
        check('actual_scan_sensor_origins_'+log.name,len(gpu)==2 and sorted(x.findtext('ignition_frame_id') for x in gpu)==['aux_mid360_scan','front_mid360_scan'] and all([float(v) for v in x.findtext('pose').split()]==[0,0,.03,0,0,0] for x in gpu))
        check('no_extra_physical_scan_links_'+log.name,not any(x.get('name','').endswith('_mid360_scan') for x in sdf.findall('.//link')))
        check('scan_frame_manifest_'+log.name,sensor['physical_scan_pose_unchanged'] and sensor['native_scan_origin_m']==[0,0,.03] and sensor['scan_frames']==[dict(mount=x,frame=x+'_scan',xyz=[0.,0.,.03]) for x in ['front_mid360','aux_mid360']])
        import yaml
        params=yaml.safe_load((log/'nav2_params.yaml').read_text())
        check('primary_LIO_native_extrinsic_'+log.name,params['point_lio']['ros__parameters']['mapping']['extrinsic_T']==[0,0,.03] and params['point_lio']['ros__parameters']['mapping']['extrinsic_est_en'] is False)
        check('aux_actual_scan_source_'+log.name,params['aux_mid_obstacle_cloud']['ros__parameters']['source_frame']=='aux_mid360_scan')
        for which,source in [('local','terrain_map'),('global','terrain_map_ext')]:
            layer=params[which+'_costmap'][which+'_costmap']['ros__parameters']
            check('costmap_scan_sources_'+which+'_'+log.name,layer['intensity_voxel_layer'][source]['sensor_frame']=='front_mid360_scan' and layer['aux_mid_obstacle_layer']['aux_mid_cloud']['sensor_frame']=='aux_mid360_scan')
        check('strict_enabled_'+log.name,params['sensor_scan_generation']['ros__parameters']['require_valid_tf'] is True)
        check('adopted_padding_'+log.name,params['global_costmap']['global_costmap']['ros__parameters']['footprint_padding']==.02 and params['local_costmap']['local_costmap']['ros__parameters']['footprint_padding']==.02)
        check('adopted_base_score_'+log.name,params['controller_server']['ros__parameters']['FollowPath']['BaseObstacle.scale']==.4)
        for which in ['global','local']:
            cp=params[which+'_costmap'][which+'_costmap']['ros__parameters']
            check('adopted_primary_footprint_clear_'+which+'_'+log.name,cp['intensity_voxel_layer']['footprint_clearing_enabled'] is True)
        check('distinct_adopted_planning_footprints_'+log.name,params['local_costmap']['local_costmap']['ros__parameters']['footprint']!=params['global_costmap']['global_costmap']['ros__parameters']['footprint'])
        follow=params['controller_server']['ros__parameters']['FollowPath']
        check('original_critics_and_projection_'+log.name,follow['critics']==['BaseObstacle','ObstacleFootprint','PathDist','GoalDist'] and follow['pose_projection_horizon']==.1 and sensor['controller_pose_projection']['source_stamps_unchanged'])
        planning=sensor['controller_planning_clearance']
        check('shared_conservative_planning_'+log.name,follow['planning_clearance_radius']==planning['radius_m']==.5 and planning['physical_hard_floor_m']==.432883794 and planning['shared_trajectory_and_goal_field'] and planning['required_bound_m']<=.5)
        check('actual_complete_cell_coverage_'+log.name,follow['planning_cell_coverage'] is True and planning['complete_goal_field_cell_coverage'] and abs(planning['goal_field_cell_cover_radius_m']-(.5+planning['goal_field_cell_half_diagonal_m']))<1e-12)
        mapping=[row for row in actual_maps if Path(row.get('ros_log_dir','/nonexistent')).is_relative_to(log)]
        check('actual_two_private_libraries_'+log.name,bool(mapping) and all(row['actual_proc_maps'] and row['libraries'].get(str(controller_private))==controller_receipt['sha256'] and row['libraries'].get(str(private))=='4b628719df9ac7ed37c2d95d67f429749aa93c9886c21a767f8b27d6872c32a7' for row in mapping),observed_process_epochs=len(mapping))
        gicp=params['small_gicp_relocalization']['ros__parameters']
        check('identity_import_'+log.name,gicp['base_frame']==gicp['lidar_frame']=='front_mid360' and gicp['robot_base_frame']=='gimbal_yaw')
        recovery_mode='registration_max_iterations' in gicp
        if recovery_mode:
            check('explicit_recovery_budget_'+log.name,gicp['registration_max_iterations']==64 and gicp['metric_correspondences'] is True and gicp['global_leaf_size']==.02 and gicp['deterministic_global_sampling'] is True and gicp['registered_leaf_size']==.05)
            receipt=load(Path(sensor['source_params']).with_suffix('.profile.json'))
            check('recovery_builder_binding_'+log.name,receipt['registration_max_iterations']==64 and receipt['metric_correspondences'] is True and receipt['correspondence_neighbor_cap']==8 and receipt['correspondence_extra_distance_m']==.025 and receipt['target_leaf_size_m']==.02 and receipt['deterministic_global_sampling'] is True and receipt['source_leaf_size_m']==.05 and receipt['legacy_target_leaf_size_m']==.1 and receipt['params_sha256']==digest(sensor['source_params']) and receipt['source_sha256']==digest(P/'scripts/rmul2027/config/nav2_test.yaml') and receipt['ordinary_builder_sha256']==load(root/'runtime-source-freeze.json')['rmul2027_decision/tools/start_low_speed.sh'])
            check('actual_recovery_gicp_mapping_'+log.name,bool(mapping) and all(row.get('gicp_libraries',{}).get(str(cad_gicp))==cad_gicp_receipt['sha256'] for row in mapping))
        else:
            check('normal_mode_gicp_unchanged_'+log.name,all(cad_gicp_receipt['sha256'] not in row.get('gicp_libraries',{}).values() for row in mapping))
        epoch_params=list((log/'imu-recovery').glob('nav-epoch-*.yaml'))
        for epoch_file in epoch_params:
            value=yaml.safe_load(epoch_file.read_text())
            ep=value.get('/red_standard_robot1',value.get('red_standard_robot1',value))
            check('epoch_native_calibration_'+log.name+'_'+epoch_file.name,ep['point_lio']['ros__parameters']['mapping']['extrinsic_T']==[0,0,.03] and ep['point_lio']['ros__parameters']['mapping']['gravity_init']==params['point_lio']['ros__parameters']['mapping']['gravity_init'] and ep['aux_mid_obstacle_cloud']['ros__parameters']['source_frame']=='aux_mid360_scan' and ep['controller_server']['ros__parameters']['FollowPath']['pose_projection_horizon']==.1 and ep['controller_server']['ros__parameters']['FollowPath']['planning_clearance_radius']==.5 and ep['controller_server']['ros__parameters']['FollowPath']['planning_cell_coverage'] is True and ep['small_gicp_relocalization']['ros__parameters']['base_frame']=='front_mid360')
            check('epoch_budget_'+log.name+'_'+epoch_file.name,recovery_mode and ep['small_gicp_relocalization']['ros__parameters']['registration_max_iterations']==64 and ep['small_gicp_relocalization']['ros__parameters']['metric_correspondences'] is True and ep['small_gicp_relocalization']['ros__parameters']['global_leaf_size']==.02 and ep['small_gicp_relocalization']['ros__parameters']['deterministic_global_sampling'] is True and ep['small_gicp_relocalization']['ros__parameters']['registered_leaf_size']==.05)
        epoch_logs=list((log/'imu-recovery').glob('nav-epoch-*.log'))
        texts=[path.read_text() for path in epoch_logs] if epoch_logs else [(log/'navigation.log').read_text()]
        if recovery_mode:
            import re
            witnesses=[re.search(r'GICP_GLOBAL_SAMPLING deterministic=true leaf=([0-9.eE+-]+) points=([0-9]+)',text) for text in texts]
            check('actual_deterministic_target_'+log.name,bool(witnesses) and all(m and float(m.group(1))==.02 and int(m.group(2))==expected_target_points for m in witnesses))
            check('actual_metric_correspondence_'+log.name,all('GICP_METRIC_CORRESPONDENCES enabled=true neighbor_cap=8 extra_distance_m=0.025' in text for text in texts))
        unique_processes={(row['pid'],row['start_ticks']) for row in mapping}
        check('observed_every_epoch_'+log.name,len(unique_processes)>=len(texts),observed=len(unique_processes),epochs=len(texts))
        check('actual_private_load_'+log.name,all(str(private) in text for text in texts),epochs=len(texts))
        check('actual_shared_radius_log_'+log.name,all('Shared trajectory/goal-field planning radius 0.500000, physical floor 0.432884' in text for text in texts),epochs=len(texts))
        check('actual_continuous_reserve_log_'+log.name,all('Continuous trajectory cell reserve enabled; inward motion from reserve forbidden' in text for text in texts),epochs=len(texts))
        check('actual_complete_cell_coverage_log_'+log.name,all('Goal-field complete cell coverage enabled, half-diagonal reserve 0.035355 m' in text for text in texts),epochs=len(texts))
    for name in ('pb_nav2_plugins','pb_omni_pid_pursuit_controller','point_lio','small_gicp_relocalization'):
        before=(root/'before'/(name+'.diff')).read_bytes()
        actual=subprocess.check_output(['git','diff','--binary'],cwd=P/name)
        check('original_dirty_'+name,before==actual)
    baseline=root.parent/'dual-mid-C070-artifacts/final-source-sha256.json'
    intentional={'scripts/rmul2027/run_simulation.py','rmul2027_decision/tools/start_low_speed.sh','rmul2027_decision/tests/dual_mid_crossfield_integration.py','scripts/rmul2027/tests/dual_mid_live_topology.py','rmul2027_decision/tests/low_speed_integration.py','rmul2027_decision/tests/recovery_integration.py','rmul2027_decision/tests/recovery_runtime.py','rmul2027_decision/tests/timing_runtime.py'}
    for name,sha in load(baseline).items():
        if name not in intentional:check('C070_source_'+name,digest(P/name)==sha)
    original={
        W/'src/pb2025_robot_description/resource/xmacro/simulation_robot.sdf.xmacro':'22f6a6cad6e1b1ba5e0d5f9713220615580aa37ad978bc7e96379e84f7170e2d',
        W/'src/pb2025_robot_description/resource/models/mid360/model.sdf.xmacro':'e4d75370281553b12cc1e71a0dbe7c7b5fc8a8a27abb6ab8695be5b534647dfe',
        P/'scripts/rmul2027/config/nav2_test.yaml':'3dd610e809375a29f5467790fb00bcccdfb03666d569b4c0fa312088e7df3a10',
        W/'install/pb_omni_pid_pursuit_controller/lib/libpb_omni_pid_pursuit_controller.so':'bf6b4c496aa4713f4615e38d41ded64bb6fbc13c035678aa8cb02fc3abdbb611'}
    for path,sha in original.items():check('original_'+str(path),digest(path)==sha)
    for receipt in load(root/'baseline-audit.json')['checks']:
        if receipt['name'].startswith(('original_install:','runtime_library:','runtime_executable:')):
            path=receipt['name'].split(':',1)[1]
            check('baseline_preserved_'+path,digest(path)==receipt['sha256'])
    check('description_repo_clean',not subprocess.check_output(['git','status','--porcelain'],cwd=W/'src/pb2025_robot_description',text=True))
    def libraries(value,expected_log):
        if isinstance(value,dict):
            command=value.get('command',[])
            if command and command[0].endswith('/component_container_isolated'):
                observed=[row for row in actual_maps if row['pid']==value['pid'] and Path(row.get('ros_log_dir','/nonexistent')).is_relative_to(expected_log)]
                check('actual_private_epoch_PID_'+str(value['pid']),bool(observed) and all(row['libraries'].get(str(controller_private))==controller_receipt['sha256'] and row['libraries'].get(str(private))=='4b628719df9ac7ed37c2d95d67f429749aa93c9886c21a767f8b27d6872c32a7' for row in observed))
            for path,sha in value.get('libraries',{}).items():
                if isinstance(sha,str) and Path(path).is_absolute():
                    check('epoch_lib_'+path,digest(path)==sha)
            if value.get('exe_sha256') and value.get('command'):check('epoch_exe_'+value['command'][0],digest(value['command'][0])==value['exe_sha256'])
            if value.get('path') and (value.get('sha256') or value.get('library_sha256')):
                check('actual_lib_'+value['path'],digest(value['path'])==value.get('sha256',value.get('library_sha256')))
            for child in value.values():libraries(child,expected_log)
        elif isinstance(value,list):
            for child in value:libraries(child,expected_log)
    for key in ('low','crossfield','imu','imu_confirmation','timing','timing_confirmation','persistent','round1','round2','round3'):
        metadata=load(root/accepted[key]/'result.json')
        expected_log=Path(metadata['navigation_recovery_logs']).parent if metadata.get('navigation_recovery_logs') else Path(metadata['navigation_log']).parent
        for pattern in ('actual-runtime.json','runtime-epoch-*.json'):
            for path in (root/accepted[key]).glob(pattern):
                value=load(path);libraries(value,expected_log)
                if pattern=='runtime-epoch-*.json':
                    check('runtime_capture_'+str(path.relative_to(root)),value['passed'] and value['parameters']['controller_server']['FollowPath.planning_cell_coverage']['boolean'])
                    budget=value['cad_registration_budget']
                    check('runtime_actual_budget64_'+str(path.relative_to(root)),value['cad_recovery_budget_overlay'] and budget['actual']==64 and budget['metric_correspondences'] is True and budget['target_leaf_size_m']==.02 and budget['deterministic_global_sampling'] is True and budget['source_leaf_size_m']==.05 and budget['sha256']==cad_gicp_receipt['sha256'] and Path(budget['library']).resolve()==cad_gicp.resolve())
    old=load(root/'accepted-before-recovery-budget.json')
    old_meta=load(root/old['imu']/'result.json')
    old_rows=[json.loads(x) for x in (Path(old_meta['navigation_recovery_logs'])/'events.jsonl').read_text().splitlines() if x]
    original_limits=next(x['limits'] for x in old_rows if x['kind']=='created')
    check('original_recovery_limits',original_limits['imu_gap']==.1 and original_limits['static_window']==2 and original_limits['verify_timeout']==30 and original_limits['max_recoveries']==2 and original_limits['quality_samples']==4)
    recovery_summary={};extra_iterations_exercised=0
    for role in ('imu','imu_confirmation','timing','timing_confirmation','persistent'):
        metadata=load(root/accepted[role]/'result.json')
        rows=[json.loads(x) for x in (Path(metadata['navigation_recovery_logs'])/'events.jsonl').read_text().splitlines() if x]
        limits=next(x['limits'] for x in rows if x['kind']=='created')
        check('unchanged_recovery_limits_'+role,limits==original_limits)
        registrations=[x for x in rows if x['kind']=='registration']
        check('actual_registration_budget_'+role,bool(registrations) and all(x['data']['optimizer_max_iterations']==64 and 1<=x['data']['optimizer_iterations']<=64 and math.isfinite(x['data']['registration_wall_s']) and x['data']['registration_wall_s']>=0 for x in registrations))
        releases=[x for x in rows if x['kind']=='release_verified']
        if role!='persistent':check('two_verified_recoveries_'+role,{0,1,2}.issubset({x['epoch'] for x in releases}))
        for release in releases:
            matches=[x for x in registrations if x['epoch']==release['epoch'] and x['steady']<=release['steady']][-4:]
            check('four_real_quality_matches_'+role+'_'+str(release['epoch']),len(matches)==4 and all(x['accepted_for_recovery'] and x['data']['converged'] and x['data']['inliers']>0 and x['data']['error_per_inlier']<=limits['quality_error'] and x['data']['hessian_ratio']>=limits['hessian_ratio'] for x in matches) and len({x['data']['sequence'] for x in matches})==4 and release['evidence']['quality_count']>=4 and all(release['evidence']['gates'].values()))
        more=[x for x in registrations if x['epoch']>0 and x['accepted_for_recovery'] and x['data']['converged'] and x['data']['optimizer_iterations']>10]
        extra_iterations_exercised+=len(more)
        recovery_summary[role]=dict(releases=len(releases),registrations=len(registrations),successful_matches_using_more_than10=len(more),max_solver_wall_s=max(x['data']['registration_wall_s'] for x in registrations),max_iterations_used=max(x['data']['optimizer_iterations'] for x in registrations))
    check('extra_budget_exercised_in_real_recovery',extra_iterations_exercised>0,accepted_real_matches=extra_iterations_exercised)
    statistics['explicit_recovery_mode']=recovery_summary
    final_sources=load(root/'final-source-sha256.json')
    check('complete_final_source_inventory',len(final_sources)==47 and set(final_sources)==set(normal_sources)|set(recovery_sources)|{'scripts/rmul2027/audit_cad_dual_mid360.py'})
    for path,sha in final_sources.items():check('final_source_'+path,digest(P/path)==sha)
    from storage_guard import tree_bytes
    owned=[]
    for path in (W/'logs').glob('*/sensor-profile.json'):
        if Path(load(path)['prefix']).is_relative_to(root):owned.append(path.parent)
    private_root=W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072'
    total=tree_bytes(root)+tree_bytes(private_root)+sum(tree_bytes(path) for path in owned)
    check('task_budget',total<4*1024**3,bytes=total)
    for role in ('stationary','motion','topology','low','crossfield','ground','fixed','moving','lower_route','imu','imu_confirmation','timing','timing_confirmation','persistent','round1','round2','round3'):
        run_names=fixed.get('source_batches',[accepted[role]]) if role=='fixed' else [accepted[role]]
        for run_name in run_names:
            path=root/(run_name+'-guard.log')
            rows=[json.loads(line) for line in path.read_text().splitlines() if line.startswith('{')]
            starts=[x for x in rows if x.get('kind')=='start']
            ends=[x for x in rows if x.get('kind')=='finished']
            check('guard_'+role+'_'+run_name,bool(starts) and bool(ends) and ends[-1]['child_returncode']==0 and ends[-1]['artifact_bytes']<512*1024**2 and starts[-1]['policy']['start_free_gib']==50 and starts[-1]['policy']['stop_free_gib']==30)
    check('branch',subprocess.check_output(['git','branch','--show-current'],cwd=P,text=True).strip()=='sim/rmul2027-3v3')
    statistics['all_replayed_goals_including_prior_candidates']=dict(totals);statistics['accepted_configuration_goals']=dict(current_totals);statistics['nav_runs']=len(navlogs);statistics['task_bytes']=total;statistics['accepted_direct_goals']=sum((3 if trial==direct[0] else 2)+(1 if (trial/'lower-entry.log').is_file() else 0) for trial in direct)
    return dict(passed=True,scope='C072 minimum CAD dual-MID simulation qualification only',checks=checks,stats=statistics,
                failures_preserved=True,single_mid='NOT_RUN_PENDING',real_hardware='NOT_RUN')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path)
    args=parser.parse_args();value=audit(args.root)
    with (args.root/'final-audit.json').open('x') as output:output.write(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=value['passed'],checks=len(value['checks']),stats=value['stats'])))
