"""Read actual loaded libraries and RPC parameters while the harness holds an epoch."""
import hashlib,json,time
from pathlib import Path
from rcl_interfaces.srv import GetParameters

def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()
def audit_runtime(node,record,epoch):
    W=Path('/home/hardy/pb2025_nav_ws');B=W/'artifacts/rmul_2027_3v3/imu-recovery-C064'
    out=dict(passed=False,epoch=epoch,processes=[],parameters={})
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
                assert str(B/'install/rmul2027_decision/lib/rmul2027_decision/decision_node')==args[0].decode()
                assert '/decision-C062/bt4/lib/libbehaviortree_cpp.so' in maps and 'libbehaviortree_cpp_v3.so' not in maps
            if args[0].endswith(b'/component_container_isolated'):
                for lib in [B/'safety-install/fake_vel_transform/lib/libfake_vel_transform.so',B/'localization-install/small_gicp_relocalization/lib/libsmall_gicp_relocalization.so']:
                    assert str(lib) in maps,lib
                assert 'libbehaviortree_cpp_v3.so' in maps and '/bt4/' not in maps
            if args[0].endswith(b'/pointlio_mapping'):
                assert sha(W/'install/point_lio/lib/point_lio/pointlio_mapping')==item['exe_sha256']
            out['processes'].append(item)
        assert len(out['processes'])==3,out['processes']
        for remote,names in [('point_lio',['max_imu_gap','common.imu_topic']),('fake_vel_transform',['recovery_stop_timeout','decision_stop_timeout']),('rmul2027_decision',['recovery_permit_timeout']),('controller_server',['FollowPath.max_speed_xy','FollowPath.discretize_by_time','FollowPath.time_granularity'])]:
            client=node.create_client(GetParameters,remote+'/get_parameters')
            assert client.wait_for_service(timeout_sec=3),remote
            request=GetParameters.Request();request.names=names;future=client.call_async(request)
            node.wait(lambda:future.done(),3)
            out['parameters'][remote]={name:dict(type=v.type,double=v.double_value,boolean=v.bool_value,string=v.string_value)
                                      for name,v in zip(names,future.result().values)}
            node.destroy_client(client)
        v=out['parameters']
        assert v['point_lio']['max_imu_gap']['double']==.1
        assert v['point_lio']['common.imu_topic']['string']=='recovery/test_imu'
        assert v['fake_vel_transform']['recovery_stop_timeout']['double']==.3
        assert v['fake_vel_transform']['decision_stop_timeout']['double']==.3
        assert v['rmul2027_decision']['recovery_permit_timeout']['double']==.3
        assert v['controller_server']['FollowPath.max_speed_xy']['double']==.5
        assert v['controller_server']['FollowPath.discretize_by_time']['boolean']
        assert v['controller_server']['FollowPath.time_granularity']['double']==.05
        assert node.recovery[-1]['epoch']==epoch and node.recovery[-1]['state']=='RUNNING'
        out['passed']=True
    finally:path.write_text(json.dumps(out,indent=2))
