"""Read actual loaded libraries and RPC parameters while the harness holds an epoch."""
import hashlib,json,time,os
from pathlib import Path
from rcl_interfaces.srv import GetParameters

def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()
def audit_runtime(node,record,epoch):
    W=Path('/home/hardy/pb2025_nav_ws');B=W/'artifacts/rmul_2027_3v3/imu-recovery-C064'
    D=Path(os.environ.get('RMUL_DECISION_BUILD_ROOT',str(W/'artifacts/rmul_2027_3v3/timing-C067')))
    cad_budget=os.environ.get('RMUL_CAD_RECOVERY_PROFILE')=='budget-overlay'
    gicp_library=(W/'artifacts/rmul_2027_3v3/cad-dual-mid-C072/gicp-metric1-install/small_gicp_relocalization/lib/libsmall_gicp_relocalization.so' if cad_budget else B/'localization-install/small_gicp_relocalization/lib/libsmall_gicp_relocalization.so')
    out=dict(passed=False,epoch=epoch,processes=[],parameters={},cad_recovery_budget_overlay=cad_budget)
    path=record/f'runtime-epoch-{epoch}.json'
    try:
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():continue
            try:args=(p/'cmdline').read_bytes().split(b'\0');maps=(p/'maps').read_text()
            except OSError:continue
            if not args or not any(args[0].endswith(v) for v in (b'/decision_node',b'/component_container_isolated',b'/pointlio_mapping')):continue
            names=['libfake_vel_transform.so','libsmall_gicp_relocalization.so','libpb_omni_pid_pursuit_controller.so','librmul_pose_fresh_condition_bt_node.so','libtf2.so','libbehaviortree_cpp.so','libbehaviortree_cpp_v3.so']
            paths=sorted(set(l.split()[-1] for l in maps.splitlines() if any(l.split()[-1].endswith('/'+v) for v in names)))
            item=dict(pid=int(p.name),command=[x.decode(errors='replace') for x in args if x],
                      exe_sha256=sha(p/'exe'),libraries={x:sha(x) for x in paths})
            if args[0].endswith(b'/decision_node'):
                assert str(D/'install/rmul2027_decision/lib/rmul2027_decision/decision_node')==args[0].decode()
                assert '/decision-C062/bt4/lib/libbehaviortree_cpp.so' in maps and 'libbehaviortree_cpp_v3.so' not in maps
            if args[0].endswith(b'/component_container_isolated'):
                for lib in [B/'safety-install/fake_vel_transform/lib/libfake_vel_transform.so',gicp_library.resolve()]:
                    assert str(lib) in maps,lib
                assert 'libbehaviortree_cpp_v3.so' in maps and '/bt4/' not in maps
            if args[0].endswith(b'/pointlio_mapping'):
                assert sha(W/'install/point_lio/lib/point_lio/pointlio_mapping')==item['exe_sha256']
            out['processes'].append(item)
        assert len(out['processes'])==3,out['processes']
        for remote,names in [('point_lio',['max_imu_gap','common.imu_topic']),('small_gicp_relocalization',['input_cloud_topic']),('fake_vel_transform',['recovery_stop_timeout','decision_stop_timeout']),('rmul2027_decision',['recovery_permit_timeout']),('controller_server',['FollowPath.max_speed_xy','FollowPath.discretize_by_time','FollowPath.time_granularity','FollowPath.planning_cell_coverage'])]:
            client=node.create_client(GetParameters,remote+'/get_parameters')
            assert client.wait_for_service(timeout_sec=3),remote
            request=GetParameters.Request();request.names=names;future=client.call_async(request)
            node.wait(lambda:future.done(),3)
            out['parameters'][remote]={name:dict(type=v.type,double=v.double_value,boolean=v.bool_value,string=v.string_value,integer=v.integer_value)
                                      for name,v in zip(names,future.result().values)}
            node.destroy_client(client)
        v=out['parameters']
        assert v['point_lio']['max_imu_gap']['double']==.1
        assert v['point_lio']['common.imu_topic']['string']=='livox/imu'
        assert v['small_gicp_relocalization']['input_cloud_topic']['string']=='recovery/test_registered_scan'
        assert v['fake_vel_transform']['recovery_stop_timeout']['double']==.3
        assert v['fake_vel_transform']['decision_stop_timeout']['double']==.3
        assert v['rmul2027_decision']['recovery_permit_timeout']['double']==.3
        assert v['controller_server']['FollowPath.max_speed_xy']['double']==.5
        assert v['controller_server']['FollowPath.discretize_by_time']['boolean']
        assert v['controller_server']['FollowPath.time_granularity']['double']==.05
        inputs=node.get_subscriptions_info_by_topic('/red_standard_robot1/livox/imu')
        supervised=[x for x in inputs if x.node_name=='imu_recovery_input_reader']
        assert len(supervised)==1
        out['imu_supervisor_qos']=dict(configured_depth=node.recovery[-1]['imu_qos_depth'],
            discovered_depth=supervised[0].qos_profile.depth,discovery_limit='Remote history depth may be unknown (0) in Humble')
        assert out['imu_supervisor_qos']['configured_depth']==400
        assert node.recovery[-1]['epoch']==epoch and node.recovery[-1]['state']=='RUNNING'
        if cad_budget:
            assert out['parameters']['controller_server']['FollowPath.planning_cell_coverage']['boolean'] is True
            client=node.create_client(GetParameters,'small_gicp_relocalization/get_parameters')
            assert client.wait_for_service(timeout_sec=3)
            request=GetParameters.Request();request.names=['registration_max_iterations','global_leaf_size','registered_leaf_size','deterministic_global_sampling','metric_correspondences']
            future=client.call_async(request);node.wait(lambda:future.done(),3)
            values=future.result().values;actual=values[0].integer_value
            target_leaf=values[1].double_value;source_leaf=values[2].double_value
            assert target_leaf==.02 and source_leaf==.05
            assert values[3].bool_value is True and values[4].bool_value is True
            assert actual==int(os.environ.get('RMUL_CAD_GICP_MAX_ITERATIONS','64'))==64
            out['cad_registration_budget']=dict(actual=actual,metric_correspondences=values[4].bool_value,deterministic_global_sampling=values[3].bool_value,target_leaf_size_m=target_leaf,source_leaf_size_m=source_leaf,library=str(gicp_library.resolve()),sha256=sha(gicp_library))
            node.destroy_client(client)
        out['passed']=True
    finally:path.write_text(json.dumps(out,indent=2))
