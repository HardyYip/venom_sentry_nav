"""Owned CAD mount smoke/motion run. No navigation goal or runtime truth feedback."""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState,PointCloud2
from tf2_ros import Buffer,TransformListener
from rcl_interfaces.srv import GetParameters

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
parser.add_argument('--motion',action='store_true')
args=parser.parse_args()
record=args.record_dir;record.mkdir(exist_ok=False)
repo=Path(__file__).resolve().parents[3]
folder=repo/'scripts/rmul2027'
children=[];streams=[];node=None;launcher=None
result=dict(passed=False,scope='CAD mount '+('motion' if args.motion else 'stationary')+' simulation; no Action goals',checks=[],observations=[])
def save():
    (record/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
def passed(name,**data):
    value=dict(name=name,passed=True,**data);result['checks'].append(value);save()
    print(json.dumps(value),flush=True)
def launch(name,cmd):
    stream=(record/(name+'.log')).open('w');streams.append(stream)
    proc=subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    children.append(proc)
    return proc
def quat(q):
    return [q.x,q.y,q.z,q.w]
def wrap(x):
    return math.atan2(math.sin(x),math.cos(x))
def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        assert launcher.poll() is None,'launcher ended'
        rclpy.spin_once(node,timeout_sec=.02)
def tf(parent,child):
    return buffer.lookup_transform(parent,child,Time())
def pose_vector(t):
    return np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
try:
    launcher=launch('launcher',['bash',str(folder/'start_cad_dual_mid360.sh')])
    navroot=None;end=time.monotonic()+180
    while time.monotonic()<end:
        assert launcher.poll() is None,'launcher exited'
        for line in (record/'launcher.log').read_text(errors='replace').splitlines():
            if 'RMUL 2027 nav started. Logs:' in line:
                navroot=Path(line.split('Logs:',1)[1].strip())
        if navroot:break
        time.sleep(.2)
    assert navroot,'no navigation logs'
    result['navigation_log']=str(navroot/'navigation.log');save()
    ready=launch('ready',[sys.executable,str(folder/'verify_simulation.py'),'--mode','ready',
        '--record-dir',str(record/'ready'),'--startup-timeout','90','--stability-seconds','2'])
    assert ready.wait(timeout=120)==0,'initial navigation readiness failed'
    rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'])
    node=Node('cad_mount_observer',namespace='/red_standard_robot1',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    counts=collections.Counter();headers={};truth=[]
    def cloud(name,msg):
        counts[name]+=1;headers[name]=dict(frame=msg.header.frame_id,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    for name in ['livox/lidar','auxiliary/livox/lidar','auxiliary/obstacles']:
        node.create_subscription(PointCloud2,name,lambda m,n=name:cloud(n,m),qos_profile_sensor_data)
    def actual(msg):
        truth.append(dict(time=time.monotonic(),xyz=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z],
            speed=[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z]))
        if len(truth)>5000:truth.pop(0)
    node.create_subscription(Odometry,'chassis_odometry_gt',actual,qos_profile_sensor_data)
    pump(4.)
    manifest=json.loads((navroot/'sensor-profile.json').read_text())
    assert manifest['profile']=='cad-dual-mid360'
    for role,frame in [('primary','front_mid360'),('auxiliary','aux_mid360')]:
        t=tf('gimbal_yaw',frame);pose=manifest['mounts'][role]
        np.testing.assert_allclose(pose_vector(t),pose['parent_xyz_m'],atol=1e-6)
        actual_rotation=Rotation.from_quat(quat(t.transform.rotation)).as_matrix()
        np.testing.assert_allclose(actual_rotation,pose['rotation_base_sensor'],atol=1e-6)
        passed(role+'_actual_mount_matches_CAD_registration')
    for name in ['livox/lidar','auxiliary/livox/lidar','auxiliary/obstacles']:
        assert counts[name]>10 and node.get_clock().now().nanoseconds*1e-9-headers[name]['stamp']<.5,(name,counts,headers)
    passed('two_pointclouds_and_occupancy_live',counts=dict(counts),headers=headers)
    client=node.create_client(GetParameters,'sensor_scan_generation/get_parameters')
    assert client.wait_for_service(timeout_sec=3)
    request=GetParameters.Request();request.names=['require_valid_tf']
    future=client.call_async(request);end=time.monotonic()+3
    while not future.done() and time.monotonic()<end:pump(.05)
    assert future.result().values[0].bool_value
    libs=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            cmd=(proc/'cmdline').read_bytes()
            maps=(proc/'maps').read_text()
        except (OSError,ProcessLookupError):continue
        if b'component_container_isolated' in cmd:
            libs += [line.split()[-1] for line in maps.splitlines() if 'cad-dual-mid-C072/install/sensor_scan_generation/lib/libsensor_scan_generation.so' in line]
    assert libs,'private sensor_scan_generation was not loaded'
    clearance_path=Path('/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/cad-dual-mid-C072/controller-trajectory4-install/pb_omni_pid_pursuit_controller/lib/libpb_omni_pid_pursuit_controller.so')
    loaded=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if str(clearance_path) in (proc/'maps').read_text():loaded.append(int(proc.name))
        except (OSError,ProcessLookupError):continue
    assert loaded,'private projection controller not loaded'
    passed('actual_private_projection_controller_library',path=str(clearance_path),pids=loaded,sha256=hashlib.sha256(clearance_path.read_bytes()).hexdigest())

    passed('actual_private_strict_TF_library',path=libs[0],sha256=hashlib.sha256(Path(libs[0]).read_bytes()).hexdigest())
    assert truth and all(max(abs(v) for v in x['speed'])<.03 for x in truth[-10:])
    passed('stationary_chassis_at_original_threshold')
    map_client=node.create_client(GetParameters,'map_server/get_parameters')
    assert map_client.wait_for_service(timeout_sec=3)
    request=GetParameters.Request();request.names=['yaml_filename']
    future=map_client.call_async(request);end=time.monotonic()+3
    while not future.done() and time.monotonic()<end:pump(.05)
    selected=json.loads((navroot/'selected-map.json').read_text())
    assert future.done() and future.result().values[0].string_value==manifest['occupancy_boundary_guard']['yaml']
    assert selected['yaml']==manifest['occupancy_boundary_guard']['yaml']
    passed('actual_mapserver_private_boundary',yaml=selected['yaml'],cells=manifest['occupancy_boundary_guard']['cells'])
    actual_padding={}
    for which,expected in [('global',.02),('local',.02)]:
        client=node.create_client(GetParameters,which+'_costmap/'+which+'_costmap/get_parameters')
        assert client.wait_for_service(timeout_sec=3)
        request=GetParameters.Request();request.names=['footprint_padding','footprint','intensity_voxel_layer.footprint_clearing_enabled']
        future=client.call_async(request);end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:pump(.05)
        assert future.done() and future.result() is not None
        actual_padding[which]=future.result().values[0].double_value
        assert abs(actual_padding[which]-expected)<1e-9,actual_padding
        assert future.result().values[2].bool_value is True
        ring=np.asarray(json.loads(future.result().values[1].string_value))
        if which=='global':
            assert ring.shape==(32,2) and np.min(np.linalg.norm(ring,axis=1))>.4349
        else:
            np.testing.assert_allclose(ring,[[.29,.25],[.29,-.25],[-.29,-.25],[-.29,.25]],atol=1e-9)
    passed('actual_adopted_footprints_and_padding',padding=actual_padding)
    client=node.create_client(GetParameters,'controller_server/get_parameters')
    assert client.wait_for_service(timeout_sec=3)
    request=GetParameters.Request();request.names=['FollowPath.BaseObstacle.scale','FollowPath.max_speed_xy','FollowPath.time_granularity','FollowPath.pose_projection_horizon','FollowPath.planning_clearance_radius','FollowPath.planning_cell_coverage']
    future=client.call_async(request);end=time.monotonic()+3
    while not future.done() and time.monotonic()<end:pump(.05)
    assert future.done() and future.result() is not None
    actual=[v.double_value for v in future.result().values[:5]]
    assert future.result().values[5].bool_value is True
    assert actual==[.4,.5,.05,.1,.5],actual
    passed('actual_projection_horizon_and_original_scoring_speed_step',values=actual)
    assert manifest['controller_planning_clearance']['shared_trajectory_and_goal_field'] is True
    passed('actual_shared_planning_radius_preserves_hard_floor',radius=actual[4],hard_floor=manifest['controller_planning_clearance']['physical_hard_floor_m'])

    assert manifest['controller_planning_clearance']['complete_goal_field_cell_coverage'] is True
    passed('actual_complete_goal_field_cell_coverage',radius=.5,cell_half_diagonal_m=manifest['controller_planning_clearance']['goal_field_cell_half_diagonal_m'])

    for spec in manifest['scan_frames']:
        relative=tf(spec['mount'],spec['frame'])
        np.testing.assert_allclose(pose_vector(relative),spec['xyz'],atol=1e-9)
        np.testing.assert_allclose(quat(relative.transform.rotation),[0.,0.,0.,1.],atol=1e-9)
    for remote,key,expected in [('point_lio','mapping.extrinsic_T',[0.,0.,.03]),('aux_mid_obstacle_cloud','source_frame','aux_mid360_scan')]:
        client=node.create_client(GetParameters,remote+'/get_parameters')
        assert client.wait_for_service(timeout_sec=3),remote
        request=GetParameters.Request();request.names=[key]
        future=client.call_async(request);end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:pump(.05)
        assert future.done() and future.result() is not None
        value=future.result().values[0]
        if isinstance(expected,list):np.testing.assert_allclose(list(value.double_array_value),expected,atol=1e-12)
        else:assert value.string_value==expected
    passed('native_scan_TF_and_actual_LIO_extrinsic')
    # Run the topology observer inside this owned test environment rather than
    # relying on service discovery from a separate desktop exec session.
    with (record/'live-topology.log').open('w') as stream:
        observer=subprocess.run([sys.executable,str(folder/'tests/dual_mid_live_topology.py'),
            '--output',str(record/'live-topology.json'),'--aux-frame','aux_mid360_scan','--main-frame','front_mid360_scan'],stdout=stream,stderr=subprocess.STDOUT,timeout=60)
    assert observer.returncode==0,'in-run topology observer failed'
    topology=json.loads((record/'live-topology.json').read_text())
    assert topology['passed']
    passed('actual_two_costmaps_and_no_old_lidar',topology=str(record/'live-topology.json'))
    pump(.2)
    if args.motion:
        publisher=node.create_publisher(JointState,'cmd_gimbal_joint',10)
        target=0.
        def command():
            msg=JointState();msg.name=['gimbal_yaw_joint','gimbal_pitch_joint'];msg.position=[float(target),0.]
            publisher.publish(msg)
        timer=node.create_timer(.05,command)
        fixed=tf('gimbal_yaw','front_mid360')
        targets=[0.,math.pi/2,-math.pi/2,math.pi,0.,2*math.pi]
        current=0.
        for requested in targets:
            deadline=time.monotonic()+max(30.,abs(requested-current)/.2+20.)
            last=time.monotonic()
            while time.monotonic()<deadline:
                now=time.monotonic();step=.2*(now-last);last=now
                target+=max(-step,min(step,requested-target))
                pump(.04)
                t=tf('base_footprint','gimbal_yaw')
                angle=Rotation.from_quat(quat(t.transform.rotation)).as_euler('xyz')[2]
                base=tf('map','base_footprint')
                estimate=pose_vector(base)
                latest=truth[-1]
                actual_xy=np.array(latest['xyz'][:2])-np.array([-4.8,3.])
                error=float(np.linalg.norm(estimate[:2]-actual_xy))
                result['observations'].append(dict(target=target,angle=angle,estimate=estimate.tolist(),truth_xy=actual_xy.tolist(),error=error))
                assert error<=.35,('localization changed under stationary yaw',error)
                for role,frame in [('primary','front_mid360'),('auxiliary','aux_mid360')]:
                    relative=tf('gimbal_yaw',frame)
                    np.testing.assert_allclose(pose_vector(relative),manifest['mounts'][role]['parent_xyz_m'],atol=1e-6)
                if abs(target-requested)<1e-4 and abs(wrap(angle-requested))<.05:break
            else:raise AssertionError(('gimbal failed to reach target',requested,angle))
            current=requested
            passed('yaw_position_'+str(requested),actual=angle,maximum_base_error=max(x['error'] for x in result['observations']))
        passed('continuous_big_yaw_motion_without_fixed_sensor_shortcut')
    result.update(passed=True,counts=dict(counts),headers=headers)
except BaseException as exc:
    result.update(error=repr(exc),traceback=traceback.format_exc())
    print(result['traceback'],flush=True)
finally:
    if node:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    for child in reversed(children):
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=55)
            except subprocess.TimeoutExpired:
                result.setdefault('cleanup_pending',[]).append(child.pid)
                result['passed']=False
    for stream in streams:stream.close()
    save()
print(json.dumps(dict(passed=result['passed'],checks=len(result['checks']),error=result.get('error'))),flush=True)
raise SystemExit(0 if result['passed'] else 1)
