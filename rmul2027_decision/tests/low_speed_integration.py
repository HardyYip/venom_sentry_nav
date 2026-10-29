import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer,TransformListener
from source_feed import Feed

parser=argparse.ArgumentParser();parser.add_argument('--record-dir',type=Path,required=True);args=parser.parse_args()
record=args.record_dir;record.mkdir(parents=True,exist_ok=False)
ws=Path('/home/hardy/pb2025_nav_ws');repo=ws/'src/pb2025_sentry_nav'
folder=repo/'scripts/rmul2027';package=repo/'rmul2027_decision';build=ws/'artifacts/rmul_2027_3v3/decision-C062'
meta=dict(passed=False,scope='one C052 low-speed simulation, test strategy; no real-robot claim',checks=[],processes={})
children={};streams=[];node=None;lio=None;paused=False

def save():
    (record/'result.json').write_text(json.dumps(meta,indent=2))
def launch(name,cmd,env=None):
    f=(record/(name+'.log')).open('w');streams.append(f)
    p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
    children[name]=p;meta['processes'][name]=dict(pid=p.pid,command=cmd);save();return p
def passed(name,**data):
    row=dict(name=name,passed=True,**data);meta['checks'].append(row);save();print(json.dumps(row),flush=True)
def waitproc(p,timeout):
    end=time.monotonic()+timeout
    while p.poll() is None and time.monotonic()<end:time.sleep(.1)
    assert p.poll()==0,'Child failed/timed out: '+str(p.pid)

class Monitor(Feed):
    def __init__(self):
        super().__init__('/red_standard_robot1',synthetic=False)
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.truth=[];self.commands=[];self.diagnostics=[];self.action_status=[];self.trace=[]
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.extra=[self.create_subscription(Odometry,'chassis_odometry_gt',self.ontruth,qos_profile_sensor_data),
                    self.create_subscription(Twist,'cmd_vel',self.oncmd,100),
                    self.create_subscription(DiagnosticArray,'safety_status',lambda m:self.diagnostics.append(dict(wall=time.monotonic(),ros=self.sim(),message=m.status[0].message)),100),
                    self.create_subscription(GoalStatusArray,'navigate_to_pose/_action/status',self.onstatus,10)]
    def wait(self,predicate,timeout=6):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:
            self.pump(.02)
            if self.status and self.status[-1]['fault']:raise RuntimeError('Latched decision fault: '+self.status[-1]['fault'])
        assert predicate(),'Condition timeout; latest='+repr(self.status[-1:])
    def sim(self):return self.get_clock().now().nanoseconds*1e-9
    def ontruth(self,m):
        p=m.pose.pose.position;v=m.twist.twist
        self.truth.append(dict(wall=time.monotonic(),ros=self.sim(),stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
          xy=[p.x+4.8,p.y-3.0],speed=[v.linear.x,v.linear.y,v.angular.z],z=p.z))
    def oncmd(self,m):
        self.commands.append(dict(wall=time.monotonic(),ros=self.sim(),value=[m.linear.x,m.linear.y,m.angular.z]))
    def onstatus(self,m):
        for s in m.status_list:self.action_status.append(dict(wall=time.monotonic(),uuid=bytes(s.goal_info.goal_id.uuid).hex(),status=s.status))
    def moving(self):
        return self.truth and math.hypot(*self.truth[-1]['speed'][:2])>.1 and self.status and self.status[-1]['state']=='active'
    def stop_check(self,name,wall,sim):
        self.wait(lambda:self.sim()>=sim+1.05 and self.truth[-1]['ros']>=sim+1. and self.commands[-1]['ros']>=sim+1.,8)
        truth=[x for x in self.truth if x['ros']<=sim+1.][-1]
        command=[x for x in self.commands if x['ros']<=sim+1.][-1]
        assert sim+1.-truth['ros']<.1 and sim+1.-command['ros']<.1,'No fresh original one-second snapshot'
        assert max(abs(v) for v in truth['speed'])<.03,(name,truth)
        assert max(abs(v) for v in command['value'])<.03,(name,command)
        samples=[x for x in self.truth if sim+.8<=x['ros']<=sim+1.]
        assert samples and all(max(abs(v) for v in x['speed'])<.03 for x in samples),'Unstable physical stop'
        zero=next((x for x in self.commands if x['wall']>=wall and max(abs(v) for v in x['value'])<.03),None)
        stopped=next((x for x in self.truth if x['wall']>=wall and max(abs(v) for v in x['speed'])<.03),None)
        passed(name,trigger_wall=wall,trigger_sim=sim,command_delay_wall_s=zero['wall']-wall if zero else None,
          physical_delay_wall_s=stopped['wall']-wall if stopped else None,truth_speed=truth['speed'],command=command['value'],
          original_one_sim_second=True,snapshot_truth_sim=truth['ros'],snapshot_command_sim=command['ros'])
    def arrival(self,count):
        self.wait(lambda:len([e for e in self.event('action_result') if e['code']==1])>=count,190)
        event=[e for e in self.event('action_result') if e['code']==1][count-1]
        sent=next(e for e in self.event('send') if e['token']==event['token']);goal=sent['goal']
        tf=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
        pos=tf.transform.translation;truth=self.truth[-1]
        estimate=math.hypot(pos.x-goal['x'],pos.y-goal['y'])
        actual=math.hypot(truth['xy'][0]-goal['x'],truth['xy'][1]-goal['y'])
        assert estimate<=.35 and actual<=.35,dict(estimate=estimate,actual=actual)
        assert event['steady']-sent['steady']<=180
        passed('arrival_'+str(count),uuid=event['uuid'],goal=goal,estimated_error_m=estimate,truth_error_m=actual,
          elapsed_wall_s=event['steady']-sent['steady'])
        self.stop_check('arrival_stop_'+str(count),event['steady'],event['ros'])

def runtime_identity():
    expected={
      'libpb_omni_pid_pursuit_controller.so':ws/'install/pb_omni_pid_pursuit_controller/lib/libpb_omni_pid_pursuit_controller.so',
      'librmul_pose_fresh_condition_bt_node.so':build/'nav-safety-install/rmul2027_nav_safety/lib/librmul_pose_fresh_condition_bt_node.so',
      'libfake_vel_transform.so':build/'safety-install/fake_vel_transform/lib/libfake_vel_transform.so',
      'libtf2.so':ws/'artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib/libtf2.so'}
    paths={}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            cmd=(proc/'cmdline').read_bytes()
            maps=(proc/'maps').read_text()
        except (OSError,PermissionError):continue
        for name,target in expected.items():
            for line in maps.splitlines():
                fields=line.split()
                if len(fields)>=6 and fields[-1].endswith('/'+name):
                    path=Path(fields[-1]);digest=hashlib.sha256(path.read_bytes()).hexdigest()
                    if digest==hashlib.sha256(target.read_bytes()).hexdigest():
                        paths[name]=dict(pid=int(proc.name),path=str(path),sha256=digest,cmd=cmd.decode(errors='replace').replace(chr(0),' '))
        if b'bt_navigator' in cmd or b'component_container_isolated' in cmd:
            assert '/bt4/' not in maps,'BTv4 leaked into navigation'
    assert len(paths)==4,paths
    decision=children['decision'];maps=Path('/proc',str(decision.pid),'maps').read_text()
    assert str(build/'bt4/lib/libbehaviortree_cpp.so') in maps
    assert 'libbehaviortree_cpp_v3.so' not in maps
    paths['decision_bt4']=dict(pid=decision.pid,path=str(build/'bt4/lib/libbehaviortree_cpp.so'),library_sha256=hashlib.sha256((build/'bt4/lib/libbehaviortree_cpp.so').read_bytes()).hexdigest(),binary_sha256=hashlib.sha256((build/'install/rmul2027_decision/lib/rmul2027_decision/decision_node').read_bytes()).hexdigest())
    navpid=paths['libpb_omni_pid_pursuit_controller.so']['pid']
    navmaps=Path('/proc',str(navpid),'maps').read_text()
    assert 'libbehaviortree_cpp_v3.so' in navmaps and '/bt4/' not in navmaps
    assert str(ws/'artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib/libtf2.so') in navmaps
    values={}
    for remote,names in [('controller_server',['FollowPath.max_speed_xy']),('velocity_smoother',['max_velocity']),('fake_vel_transform',['decision_stop_timeout']),('bt_navigator',['default_nav_to_pose_bt_xml'])]:
        client=node.create_client(GetParameters,remote+'/get_parameters');assert client.wait_for_service(timeout_sec=2),remote
        request=GetParameters.Request();request.names=names
        future=client.call_async(request);node.wait(lambda:future.done(),3)
        vals=future.result().values
        values[remote]=[dict(type=v.type,double_value=v.double_value,double_array_value=list(v.double_array_value),string_value=v.string_value) for v in vals]
    assert values['controller_server'][0]['double_value']==.5
    assert values['velocity_smoother'][0]['double_array_value'][0]==.5
    assert values['fake_vel_transform'][0]['double_value']==.3
    assert values['bt_navigator'][0]['string_value'].endswith('/navigate_field.xml')
    paths['actual_parameters']=values
    (record/'actual-runtime.json').write_text(json.dumps(paths,indent=2))
    passed('actual_library_isolation',libraries=paths)

def inject(name,change,restore):
    node.wait(node.moving,40);old=node.event('accepted')[-1]['token']
    wall=time.monotonic();sim=node.sim();change()
    node.wait(lambda:any(e['token']==old for e in node.event('cancel')) or any(e['token']==old for e in node.event('action_result')),4)
    node.stop_check(name,wall,sim)
    node.wait(lambda:any(e['token']==old for e in node.event('action_result')),5)
    terminal=next(e for e in node.event('action_result') if e['token']==old)
    assert terminal['code'] in (1,2,3),terminal
    assert terminal['steady']>=wall,terminal
    previous=len(node.event('send'));restore()
    node.wait(lambda:len(node.event('send'))>previous,15)
    sent=node.event('send')[-1]
    assert sent['steady']>terminal['steady']
    passed(name+'_terminal_and_recovery',old_token=old,terminal=terminal['code'],new_token=sent['token'],
      classification='cancel_confirmed' if terminal['code']==3 else 'navigation_guard_aborted' if terminal['code']==2 else 'completed_during_stop')

try:
    meta['free_bytes']={x:__import__('shutil').disk_usage(x).free for x in ['/mnt/d','/mnt/e']};save()
    files=[*package.glob('src/*.cpp'),*package.glob('include/rmul2027_decision/*.hpp'),package/'config/test_policy.yaml',package/'behavior_trees/rmul_2027.xml',repo/'rmul2027_nav_safety/src/pose_fresh.cpp',repo/'fake_vel_transform/src/fake_vel_transform.cpp']
    (record/'source-sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
    wrapper=launch('launcher',['bash',str(package/'tools/start_low_speed.sh')])
    navlog=None;end=time.monotonic()+180
    while time.monotonic()<end:
        assert wrapper.poll() is None,'Launcher exited'
        for line in (record/'launcher.log').read_text(errors='replace').splitlines():
            if 'RMUL 2027 nav started. Logs:' in line:navlog=Path(line.split('Logs:',1)[1].strip())/'navigation.log'
        if navlog:break
        time.sleep(.2)
    assert navlog,'No navigation startup log';meta['navigation_log']=str(navlog);save()
    ready=launch('ready',[sys.executable,str(folder/'verify_simulation.py'),'--mode','ready','--record-dir',str(record/'ready'),'--startup-timeout','90','--stability-seconds','2'])
    waitproc(ready,120)
    rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'],signal_handler_options=rclpy.signals.SignalHandlerOptions.NO);node=Monitor();node.pump(.5)
    decision=launch('decision',['bash',str(package/'tools/start_decision.sh'),'--ros-args','--params-file',str(package/'config/test_policy.yaml'),
      '-r','__ns:=/red_standard_robot1','-r','/tf:=tf','-r','/tf_static:=tf_static',
      '-p','dwell_seconds:=4.0','-p','event_file:='+str(record/'decision-events.jsonl')])
    node.child=decision;node.wait(lambda:bool(node.status),10);node.pump(.7)
    assert not node.event('send')
    node.wait(lambda:0<node.status[-1]['ros']<1000,5)
    assert 0<node.status[-1]['ros']<1000,'Decision not using simulation clock'
    passed('waiting_no_goal',actual_ros_time=node.status[-1]['ros'])
    node.phase=1;node.arrival(1);runtime_identity()
    node.arrival(2)
    successes=[x for x in node.event('action_result') if x['code']==1]
    goals=[next(e['goal'] for e in node.event('send') if e['token']==x['token']) for x in successes[:2]]
    assert (goals[0]['x'],goals[0]['y'])!=(goals[1]['x'],goals[1]['y']),'Patrol repeated same point'
    passed('patrol_two_distinct_points',goals=goals)
    node.wait(node.moving,40);old=node.event('accepted')[-1]['token'];node.hp=50.
    node.wait(lambda:any(e.get('goal',{}).get('key')=='retreat' for e in node.event('send')),10)
    old_terminal=next(e for e in node.event('action_result') if e['token']==old)
    retreat=next(e for e in node.event('send') if e.get('goal',{}).get('key')=='retreat')
    assert old_terminal['steady']<retreat['steady']
    passed('retreat_switch_after_terminal',old_token=old,old_terminal=old_terminal['code'],new_token=retreat['token'])
    node.arrival(3);node.hp=300.
    node.wait(lambda:node.status[-1]['mode']=='patrol' and node.status[-1]['state']=='active',15)
    passed('recovery_resumes_patrol')
    inject('referee_loss_physical_stop',lambda:setattr(node,'match_on',False),lambda:setattr(node,'match_on',True))
    inject('localization_untrusted_physical_stop',lambda:setattr(node,'trust',False),lambda:setattr(node,'trust',True))
    matches=re.findall(r'\[pointlio_mapping-\d+\]: process started with pid \[(\d+)\]',navlog.read_text(errors='replace'))
    assert len(matches)==1,matches;lio=int(matches[0])
    assert b'pointlio_mapping' in Path('/proc',str(lio),'cmdline').read_bytes()
    def pause_lio():
        global paused
        os.kill(lio,signal.SIGSTOP);paused=True
    def resume_lio():
        global paused
        os.kill(lio,signal.SIGCONT);paused=False
    inject('estop_physical_stop',lambda:setattr(node,'estop',True),lambda:setattr(node,'estop',False))
    node.wait(node.moving,40);old=node.event('accepted')[-1]['token'];wall=time.monotonic();sim=node.sim();node.phase=2
    node.stop_check('match_finished_physical_stop',wall,sim)
    node.wait(lambda:any(e['token']==old for e in node.event('action_result')),5)
    count=len(node.event('send'));node.phase=1;node.pump(.7);assert len(node.event('send'))==count
    passed('finished_latched_no_residual_goal')
    (record/'primary-observations.json').write_text(json.dumps(dict(truth=node.truth,commands=node.commands,
      diagnostics=node.diagnostics,status=node.status,action_status=node.action_status,events=node.events),separators=(',',':')))
    node.child=None;decision.send_signal(signal.SIGINT);decision.wait(timeout=4)
    assert decision.returncode==0
    passed('postmatch_graceful_exit')
    node.events=[];node.status=[];node.phase=0
    terminal_decision=launch('decision_fault_terminal',['bash',str(package/'tools/start_decision.sh'),'--ros-args',
      '--params-file',str(package/'config/test_policy.yaml'),'-r','__ns:=/red_standard_robot1',
      '-r','/tf:=tf','-r','/tf_static:=tf_static','-p','dwell_seconds:=4.0',
      '-p','event_file:='+str(record/'localization-terminal-events.jsonl')])
    node.child=terminal_decision;node.wait(lambda:bool(node.status),10);node.phase=1
    node.wait(node.moving,40)
    owned=node.event('goal_response')[-1];token=owned['token'];wall=time.monotonic();sim=node.sim()
    pause_lio()
    node.wait(lambda:any(e['token']==token for e in node.event('cancel')) or
      any(e['token']==token for e in node.event('action_result')),4)
    node.stop_check('actual_localization_stream_loss',wall,sim)
    node.wait(lambda:any(e['token']==token for e in node.event('action_result')),5)
    terminal=next(e for e in node.event('action_result') if e['token']==token)
    assert terminal['code'] in (2,3),terminal
    previous=len(node.event('send'));resume_lio();node.pump(2.)
    lines=[line for line in navlog.read_text(errors='replace').splitlines() if 'IMU_INPUT_FAULT_LATCH' in line]
    assert lines,'Terminal localization fault did not exercise original IMU gap latch'
    assert len(node.event('send'))==previous and node.status[-1]['inhibited'] and not node.status[-1]['goal']
    assert node.status[-1]['reason'] in ('odometry_invalid_or_stalled','odometry_source_age','tf_stale_or_stalled')
    passed('imu_gap_latch_preserved_no_unsafe_resume',uuid=owned['uuid'],terminal=terminal['code'],
      backend_recovered=False,requires_stationary_reinitialization=True,original_limit_s=.1,raw_evidence=lines[-1])
    meta.update(passed=True,primary_closed_loop_passed=True,terminal_gap_safety_passed=True,
      backend_gap_automatic_recovery='not available by design; original safety latch retained')
except BaseException as exc:
    meta.update(error=repr(exc),traceback=traceback.format_exc());(record/'exception.txt').write_text(meta['traceback']);save();print(meta['traceback'],flush=True)
finally:
    if paused and lio:
        os.kill(lio,signal.SIGCONT);paused=False
    if node:
        node.child=None;node.estop=True
        try:node.pump(.5)
        except Exception:pass
        owned=[x['uuid'] for x in node.event('goal_response') if x['accepted']]
        if owned:
            service=node.create_client(CancelGoal,'navigate_to_pose/_action/cancel_goal')
            if service.wait_for_service(timeout_sec=1):
                req=CancelGoal.Request();req.goal_info.goal_id.uuid=list(bytes.fromhex(owned[-1]))
                future=service.call_async(req)
                end=time.monotonic()+3
                while not future.done() and time.monotonic()<end:node.pump(.02)
                meta['cleanup_cancel_uuid']=owned[-1]
        (record/'observations.json').write_text(json.dumps(dict(truth=node.truth,commands=node.commands,diagnostics=node.diagnostics,
          status=node.status,action_status=node.action_status,events=node.events),separators=(',',':')))
        try:
            node.cleanup()
            if rclpy.ok():rclpy.shutdown()
        except Exception as cleanup_error:meta['node_cleanup_error']=repr(cleanup_error)
    for name in ['decision_fault_terminal','decision','ready','launcher']:
        child=children.get(name)
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=55)
            except subprocess.TimeoutExpired:meta.setdefault('cleanup_pending',[]).append(child.pid);meta['passed']=False
    for stream in streams:stream.close()
    save()
print(json.dumps(dict(passed=meta['passed'],checks=len(meta['checks']),error=meta.get('error'),record=str(record))),flush=True)
raise SystemExit(0 if meta['passed'] else 1)
