"""C067 TEST TF/clock timing recovery on the 2027 map. GT is evaluation only."""
import argparse,hashlib,json,math,os,re,signal,subprocess,sys,time,traceback

from pathlib import Path

import rclpy

from rclpy.parameter import Parameter

from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

from std_msgs.msg import String

from sensor_msgs.msg import Imu,PointCloud2

from tf2_ros import Buffer,TransformListener

from source_feed import Feed
from timing_runtime import audit_runtime

def save():(record/'result.json').write_text(json.dumps(result,indent=2))

def check(name,**kw):
    result['checks'].append(dict(name=name,passed=True,**kw));save();print(json.dumps(result['checks'][-1]),flush=True)

def launch(name,cmd,env=None):
    f=(record/(name+'.log')).open('w');streams.append(f)
    p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
    children[name]=p;return p

class Monitor(Feed):
    def __init__(self):
        super().__init__('/red_standard_robot1',synthetic=False)
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.truth=[];self.commands=[];self.recovery=[];self.relay_enabled=True;self.imu_relay_trace=[]
        self.cloud_on=True;self.cloud_forwarded=0
        self.cloud_relay=self.create_publisher(PointCloud2,'recovery/test_registered_scan',10)
        self.cloud_sub=self.create_subscription(PointCloud2,'registered_scan',self.oncloud,10)
        self.imu_relay=self.create_publisher(Imu,'recovery/test_imu',qos_profile_sensor_data)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.extra=[self.create_subscription(Odometry,'chassis_odometry_gt',self.ontruth,qos_profile_sensor_data),
                    self.create_subscription(Twist,'cmd_vel',self.oncmd,100),
                    self.create_subscription(String,'recovery/status',lambda m:self.recovery.append(json.loads(m.data)),100)]
    def oncloud(self,m):
        if self.cloud_on:
            self.cloud_relay.publish(m);self.cloud_forwarded+=1
    def relay_imu(self,m):
        if self.relay_enabled:self.imu_relay.publish(m)
    def sim(self):return self.get_clock().now().nanoseconds*1e-9
    def ontruth(self,m):
        p=m.pose.pose.position;v=m.twist.twist
        self.truth.append(dict(t=time.monotonic(),ros=self.sim(),xy=[p.x+4.8,p.y-3.],speed=[v.linear.x,v.linear.y,v.angular.z]))
    def oncmd(self,m):self.commands.append(dict(t=time.monotonic(),ros=self.sim(),speed=[m.linear.x,m.linear.y,m.angular.z]))
    def wait(self,pred,timeout=10,allow_failed=False):
        end=time.monotonic()+timeout
        while not pred() and time.monotonic()<end:
            self.pump(.02)
            if self.status and self.status[-1]['fault']:raise RuntimeError('decision fault: '+self.status[-1]['fault'])
            if self.recovery and self.recovery[-1]['state']=='FAILED' and not pred() and not allow_failed:raise RuntimeError('recovery failed: '+self.recovery[-1]['fault'])
        assert pred(),'timeout; decision='+repr(self.status[-1:])+' recovery='+repr(self.recovery[-1:])
    def moving(self):
        return self.truth and math.hypot(*self.truth[-1]['speed'][:2])>.1 and self.status and self.status[-1]['state']=='active'
    def stop_check(self,name,t,s):
        self.wait(lambda:self.sim()>s+1.05 and self.truth[-1]['ros']>=s+1. and self.commands[-1]['ros']>=s+1.,8,allow_failed=True)
        truth=[x for x in self.truth if x['ros']<=s+1.][-1];cmd=[x for x in self.commands if x['ros']<=s+1.][-1]
        assert s+1.-truth['ros']<.1 and s+1.-cmd['ros']<.1
        assert max(abs(v) for v in truth['speed'])<.03 and max(abs(v) for v in cmd['speed'])<.03,(truth,cmd)
        window=[x for x in self.truth if s+.8<=x['ros']<=s+1.]
        assert window and all(max(abs(v) for v in x['speed'])<.03 for x in window)
        stopped=next(x for x in self.truth if x['t']>=t and max(abs(v) for v in x['speed'])<.03)
        check(name,trigger_ros=s,physical_delay_ros=stopped['ros']-s,truth_speed=truth['speed'],command=cmd['speed'])
    def arrival(self,count):
        self.wait(lambda:len([e for e in self.event('action_result') if e['code']==1])>=count,180)
        end=[e for e in self.event('action_result') if e['code']==1][count-1]
        start=next(e for e in self.event('send') if e['token']==end['token']);g=start['goal']
        tf=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time());p=tf.transform.translation
        estimate=math.hypot(p.x-g['x'],p.y-g['y']);truth=math.hypot(self.truth[-1]['xy'][0]-g['x'],self.truth[-1]['xy'][1]-g['y'])
        assert estimate<=.35 and truth<=.35,(estimate,truth)
        assert end['steady']-start['steady']<=180
        check('actual_arrival_'+str(count),uuid=end['uuid'],estimated_error_m=estimate,truth_error_m=truth,goal=g)
        self.stop_check('arrival_stop_'+str(count),end['steady'],end['ros'])

import argparse,os

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
parser.add_argument('--mode',choices=['cycle','persistent'],default='cycle')
parser.add_argument('--launcher',type=Path,help='Explicit isolated diagnostic launcher; default retains adopted recovery launcher')
parser.add_argument('--bridge-provenance',type=Path,help='Optional actual API trace mapping proof required before sending a goal')
parser.add_argument('--bridge-count',type=int,choices=(2,3),default=2)
parser.add_argument('--fault-min-distance',type=float,choices=(.2,1.2),default=.2,help='Explicit TEST injection position; 1.2m matches the C064 positive recovery route')
args=parser.parse_args();record=args.record_dir;record.mkdir(parents=True,exist_ok=False)
W=Path('/home/hardy/pb2025_nav_ws');P=W/'src/pb2025_sentry_nav';package=P/'rmul2027_decision'
B=Path(os.environ.get('RMUL_DECISION_BUILD_ROOT',str(W/'artifacts/rmul_2027_3v3/timing-C067')))
RB=W/'artifacts/rmul_2027_3v3/imu-recovery-C064'
children={};streams=[];node=None;world_paused=False
result=dict(passed=False,scope='C067 TEST controlled TF/clock timing fault recovery; no historical root-cause claim',mode=args.mode,fault_min_distance_m=args.fault_min_distance,checks=[])
def wait_active():
    node.wait(lambda:node.moving() and bool(node.recovery) and node.recovery[-1]['state']=='RUNNING',60)
    token=node.event('accepted')[-1]['token']
    node.wait(lambda:any(e['token']==token and e['accepted'] for e in node.event('goal_response')),5)
    response=next(e for e in node.event('goal_response') if e['token']==token and e['accepted'])
    (record/'goal-ownership.json').write_text(json.dumps({'goal_uuid':response['uuid'],'token':token}))
    return token
def world_control(paused):
    global world_paused
    command=['ign','service','-s','/world/default/control','--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean','--timeout','2000','--req','pause: '+str(paused).lower()]
    response=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=6)
    result.setdefault('world_control',[]).append(dict(steady=time.monotonic(),ros=node.sim(),paused=paused,returncode=response.returncode,output=response.stdout));save()
    assert response.returncode==0 and 'data:true' in ''.join(response.stdout.split()),response.stdout
    world_paused=paused
def terminal_for(token):
    node.wait(lambda:any(e['token']==token for e in node.event('action_result')),8)
    terminal=next(e for e in node.event('action_result') if e['token']==token)
    assert terminal['code'] in (2,3),terminal
    return terminal
def resumed(epoch,token,terminal,estop=False):
    if estop:
        node.wait(lambda:node.recovery[-1]['epoch']==epoch and node.recovery[-1]['state']=='VERIFY',80)
        node.estop=True;before=len(node.event('send'));node.pump(2)
        assert not node.recovery[-1]['permit'] and len(node.event('send'))==before
        assert max(abs(v) for v in node.truth[-1]['speed'])<.03
        check('estop_blocks_timing_recovery_release',epoch=epoch);node.estop=False
    node.wait(lambda:node.recovery[-1]['epoch']==epoch and node.recovery[-1]['state']=='RUNNING',90)
    ready=next(e for e in node.recovery if e['epoch']==epoch and e['state']=='RUNNING')
    node.wait(lambda:any(e['token']>token for e in node.event('send')),10)
    new=next(e for e in node.event('send') if e['token']>token)
    trace=[json.loads(l) for l in (navlogs/'events.jsonl').read_text().splitlines()]
    restart=next(e for e in trace if e['kind']=='restart_authorized' and e['epoch']==epoch)
    assert restart['terminal'] and restart['stationary'] and restart['steady']>terminal['steady']
    assert all(not e['remaining'] for e in trace if e['kind']=='owned_tree_audit')
    assert new['steady']>=ready['steady']-.03 and new['steady']>terminal['steady']
    release=ready['release']
    assert release['epoch']==epoch and release['quality_count']>=4
    assert all(release['gates'].values()) and release['steady']<=new['steady']
    audit_runtime(node,record,epoch)
    check('timing_recovery_released_after_new_real_alignment',epoch=epoch,trigger=ready['trigger'],quality_count=release['quality_count'],release=release,old_terminal=terminal,new_token=new['token'])
def full_intent_arrival(count,token):
    first=next(e for e in node.event('send') if e['token']==token)
    node.arrival(count)
    last=[e for e in node.event('action_result') if e['code']==1][count-1]
    elapsed=last['steady']-first['steady']
    assert elapsed<=180,elapsed
    check('whole_intent_including_timing_recovery',count=count,elapsed_wall_s=elapsed,first_token=token,successful_token=last['token'])
try:
    save()
    rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'],signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=Monitor()
    env=os.environ.copy();env.update(RMUL_DECISION_BUILD_ROOT=str(B),RMUL_RECOVERY_BUILD_ROOT=str(RB),RMUL_NAV_BASE_BUILD_ROOT=str(W/'artifacts/rmul_2027_3v3/decision-C062'),RMUL_RECOVERY_GICP_TOPIC='recovery/test_registered_scan')
    env.pop('RMUL_RECOVERY_IMU_TOPIC',None)
    decision=launch('decision',['bash',str(package/'tools/start_decision.sh'),'--ros-args','--params-file',str(package/'config/test_crossfield_policy.yaml'),'-r','__ns:=/red_standard_robot1','-r','/tf:=tf','-r','/tf_static:=tf_static','-p','recovery_permit_timeout:=0.3','-p','event_file:='+str(record/'decision-events.jsonl')],env)
    node.child=decision
    wrapper=launch('launcher',['bash',str(args.launcher.resolve() if args.launcher else package/'tools/start_recovery.sh')],env)
    deadline=time.monotonic()+80;navlogs=None
    while time.monotonic()<deadline and navlogs is None:
        assert wrapper.poll() is None
        for line in (record/'launcher.log').read_text(errors='replace').splitlines():
            if 'RMUL 2027 nav started. Logs:' in line:navlogs=Path(line.split('Logs:',1)[1].strip())/'imu-recovery'
        node.pump(.1)
    assert navlogs is not None
    result['navigation_recovery_logs']=str(navlogs);save()
    (record/'session.json').write_text(json.dumps({'owner':'Codex C067 timing integration','navigation_log':str(navlogs.parent/'navigation.log')}))
    node.wait(lambda:node.recovery and node.recovery[-1]['state']=='RUNNING',120)
    assert not node.event('send')
    check('epoch0_real_alignment_static_and_no_goal',forwarded=node.cloud_forwarded)
    audit_runtime(node,record,0)
    if args.bridge_provenance:
        def bridge_ready():
            try:return len(json.loads(args.bridge_provenance.read_text()).get('processes',{}))==args.bridge_count
            except (OSError,ValueError):return False
        node.wait(bridge_ready,15);check('actual_bridge_trace_mapping_ready')
    node.phase=1;token=wait_active()
    node.wait(lambda:math.hypot(*node.truth[-1]['xy'])>=args.fault_min_distance,30)
    t=time.monotonic();s0=node.sim();node.cloud_on=False
    node.stop_check('tf_timing_fault_original_one_second_physical_stop',t,s0)
    terminal=terminal_for(token)
    node.wait(lambda:node.recovery[-1]['state']!='RUNNING',5)
    assert node.recovery[-1]['trigger'].startswith('tf_'),node.recovery[-1]
    check('tf_fault_detected_while_raw_inputs_healthy',trigger=node.recovery[-1]['trigger'],terminal=terminal)
    node.wait(lambda:node.recovery[-1]['epoch']==1 and node.recovery[-1]['state']=='VERIFY',80)
    before=len(node.event('send'));node.pump(2)
    assert not node.recovery[-1]['permit'] and len(node.event('send'))==before
    check('withheld_registration_cannot_release_new_epoch',status=node.recovery[-1])
    if args.mode=='persistent':
        node.wait(lambda:node.recovery[-1]['state']=='FAILED',40)
        assert node.recovery[-1]['fault']=='new_epoch_localization_unconfirmed'
        node.cloud_on=True;before=len(node.event('send'));node.pump(3)
        assert not node.recovery[-1]['permit'] and len(node.event('send'))==before
        assert max(abs(v) for v in node.truth[-1]['speed'])<.03
        check('persistent_timing_failure_stays_latched_after_input_returns',status=node.recovery[-1])
    else:
        node.cloud_on=True;resumed(1,token,terminal,estop=True);full_intent_arrival(1,token)
        token=wait_active();t=time.monotonic();s0=node.sim()
        world_control(True);node.pump(.85);world_control(False)
        node.stop_check('native_clock_pause_original_one_second_physical_stop',t,s0)
        terminal=terminal_for(token)
        resumed(2,token,terminal);full_intent_arrival(2,token)
        token=wait_active();t=time.monotonic();s0=node.sim();node.cloud_on=False
        node.stop_check('third_timing_fault_original_one_second_physical_stop',t,s0)
        terminal=terminal_for(token)
        node.wait(lambda:node.recovery[-1]['state']=='FAILED',45)
        assert node.recovery[-1]['fault']=='recovery_budget_exhausted' and node.recovery[-1]['epoch']==2
        before=len(node.event('send'));node.pump(2)
        assert len(node.event('send'))==before and node.status[-1]['inhibited']
        check('timing_budget_no_third_restart',terminal=terminal,status=node.recovery[-1])
    node.phase=2;node.pump(.6);assert node.status[-1]['mode']=='finished'
    check('finished_remains_stopped_after_timing_failure')
    result['passed']=True;save()
except BaseException as e:
    result.update(error=repr(e),traceback=traceback.format_exc());save();print(result['traceback'],flush=True)
finally:
    if world_paused:
        try:world_control(False)
        except Exception as error:result['world_resume_cleanup_error']=repr(error)
    if node:
        node.phase=2;node.pump(.2);node.cloud_on=True;node.child=None
        (record/'stop-state.json').write_text(json.dumps({'finished':True}))
    for name in ('decision','launcher'):
        p=children.get(name)
        if p and p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=85 if name=='launcher' else 5)
            except subprocess.TimeoutExpired:result['cleanup_error']=name+' did not stop'
    if node:
        (record/'observations.json').write_text(json.dumps(dict(truth=node.truth,commands=node.commands,recovery=node.recovery,status=node.status,events=node.events),separators=(',',':')))
        node.cleanup()
    for f in streams:f.close()
    if rclpy.ok():rclpy.shutdown()
    result['child_returncodes']={k:v.poll() for k,v in children.items()};save()
assert result['passed'],result
