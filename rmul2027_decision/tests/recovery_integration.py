"""C064 single 2027 map / C052 low-speed physical acceptance.
Ground truth is subscribed for assertions ONLY and never published into navigation.
"""
import argparse,hashlib,json,math,os,re,signal,subprocess,sys,time,traceback
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from recovery_runtime import audit_runtime
from tf2_ros import Buffer,TransformListener
from source_feed import Feed
from imu_relay import ImuRelay

parser=argparse.ArgumentParser();parser.add_argument('--record-dir',type=Path,required=True);args=parser.parse_args()
record=args.record_dir;record.mkdir(parents=True,exist_ok=False)
W=Path('/home/hardy/pb2025_nav_ws');P=W/'src/pb2025_sentry_nav';package=P/'rmul2027_decision'
B=Path(os.environ.get('RMUL_DECISION_BUILD_ROOT',str(W/'artifacts/rmul_2027_3v3/imu-recovery-C064')))
children={};streams=[];node=None;paused=None
result=dict(passed=False,scope='C064 TEST 2027 CAD-flat-floor C052 low-speed; GT evaluation only',checks=[])
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
        self.truth=[];self.commands=[];self.recovery=[];self.imu_relay_trace=[]
        self.imu_bridge=ImuRelay(self.get_namespace())
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.extra=[self.create_subscription(Odometry,'chassis_odometry_gt',self.ontruth,qos_profile_sensor_data),
                    self.create_subscription(Twist,'cmd_vel',self.oncmd,100),
                    self.create_subscription(String,'recovery/status',lambda m:self.recovery.append(json.loads(m.data)),100)]
    @property
    def relay_enabled(self):return self.imu_bridge.enabled
    @relay_enabled.setter
    def relay_enabled(self,value):self.imu_bridge.enabled=value
    def cleanup(self):
        (record/'imu-relay-provenance.json').write_text(json.dumps(self.imu_bridge.snapshot(),indent=2))
        self.imu_bridge.close()
        super().cleanup()
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
def inject(epoch,allow=True,estop_during_verify=False,raw_imu=False):
    global paused
    node.wait(node.moving,45)
    log=navlogs/f'nav-epoch-{epoch-1}.log'
    pids=re.findall(r'\[pointlio_mapping-\d+\]: process started with pid \[(\d+)\]',log.read_text(errors='replace'))
    assert len(pids)==1,pids
    pid=int(pids[0]);proc=Path('/proc',str(pid))
    assert b'pointlio_mapping' in (proc/'cmdline').read_bytes()
    identity=(proc/'stat').read_text().rsplit(') ',1)[1].split()[19]
    # PID came from this supervisor's own epoch log, and start time is checked again.
    token=node.event('accepted')[-1]['token'];t=time.monotonic();s=node.sim()
    if raw_imu:
        node.relay_enabled=False;node.imu_relay_trace.append(dict(t=t,ros=s,enabled=False))
    else:
        os.kill(pid,signal.SIGSTOP);paused=(pid,identity)
    node.pump(.85)
    if raw_imu:
        node.relay_enabled=True;node.imu_relay_trace.append(dict(t=time.monotonic(),ros=node.sim(),enabled=True))
    else:
        assert (proc/'stat').read_text().rsplit(') ',1)[1].split()[19]==identity
        os.kill(pid,signal.SIGCONT);paused=None
    node.stop_check('imu_fault_stop_'+str(epoch),t,s)
    node.wait(lambda:any(e['token']==token for e in node.event('action_result')),5)
    terminal=next(e for e in node.event('action_result') if e['token']==token)
    assert terminal['code'] in (2,3),terminal
    node.wait(lambda:'IMU_INPUT_FAULT_LATCH' in log.read_text(errors='replace'),4)
    latch=[l for l in log.read_text(errors='replace').splitlines() if 'IMU_INPUT_FAULT_LATCH' in l][-1]
    check('original_gap_latch_'+str(epoch),terminal=terminal['code'],uuid=terminal['uuid'],log=latch,
          source='raw IMU messages withheld from backend and recovery monitor' if raw_imu else 'owned backend paused; IMU queue overflows')
    if not allow:
        node.wait(lambda:node.recovery and node.recovery[-1]['state']=='FAILED',45)
        assert node.recovery[-1]['fault']=='recovery_budget_exhausted',node.recovery[-1]
        assert node.recovery[-1]['epoch']==2
        before=len(node.event('send'));node.pump(2)
        assert len(node.event('send'))==before and node.status[-1]['inhibited'] and not node.status[-1]['goal']
        assert max(abs(v) for v in node.truth[-1]['speed'])<.03
        check('recovery_budget_latched_no_third_restart',status=node.recovery[-1]);return
    if estop_during_verify:
        node.wait(lambda:node.recovery and node.recovery[-1]['epoch']==epoch and node.recovery[-1]['state']=='VERIFY',80)
        node.estop=True;before=len(node.event('send'));node.pump(2.5)
        assert len(node.event('send'))==before and not node.recovery[-1]['permit']
        assert max(abs(v) for v in node.truth[-1]['speed'])<.03
        check('estop_during_reinitialization_blocks_release')
        node.estop=False
    node.wait(lambda:node.recovery and node.recovery[-1]['epoch']==epoch and node.recovery[-1]['state']=='RUNNING',95)
    node.wait(lambda:len([e for e in node.event('send') if e['token']>token])>=1,10)
    new=next(e for e in node.event('send') if e['token']>token)
    assert new['steady']>terminal['steady']
    recovered=next(e for e in node.recovery if e['epoch']==epoch and e['state']=='RUNNING')
    assert new['steady']>=recovered['steady']-.03
    release=recovered['release']
    assert release['epoch']==epoch and release['quality_count']>=4
    assert all(release['gates'].values()) and release['steady']<=new['steady']
    trace=[json.loads(l) for l in (navlogs/'events.jsonl').read_text().splitlines()]
    restart=next(e for e in trace if e['kind']=='restart_authorized' and e['epoch']==epoch)
    assert restart['terminal'] and restart['stationary'] and restart['steady']>terminal['steady']
    audits=[e for e in trace if e['kind']=='owned_tree_audit']
    assert audits and all(not e['remaining'] for e in audits)
    check('controlled_recovery_'+str(epoch),old_uuid=terminal['uuid'],old_terminal=terminal['code'],new_token=new['token'],
          recovery_wall_s=recovered['steady']-t,epoch=epoch,actual_alignment_count=release['quality_count'],release=release)
    audit_runtime(node,record,epoch)

try:
    save()
    rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'],signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=Monitor()
    env=os.environ.copy();env['RMUL_DECISION_BUILD_ROOT']=str(B);env['RMUL_RECOVERY_IMU_TOPIC']='recovery/test_imu'
    decision=launch('decision',['bash',str(package/'tools/start_decision.sh'),'--ros-args',
        '--params-file',str(package/'config/test_crossfield_policy.yaml'),'-r','__ns:=/red_standard_robot1',
        '-r','/tf:=tf','-r','/tf_static:=tf_static','-p','recovery_permit_timeout:=0.3',
        '-p','event_file:='+str(record/'decision-events.jsonl')],env)
    node.child=decision
    wrapper=launch('launcher',['bash',str(package/'tools/start_recovery.sh')],env)
    navlogs=None
    deadline=time.monotonic()+70
    while time.monotonic()<deadline and navlogs is None:
        assert wrapper.poll() is None
        for line in (record/'launcher.log').read_text(errors='replace').splitlines():
            if 'RMUL 2027 nav started. Logs:' in line:navlogs=Path(line.split('Logs:',1)[1].strip())/'imu-recovery'
        node.pump(.1)
    assert navlogs is not None
    result['navigation_recovery_logs']=str(navlogs);save()
    node.wait(lambda:node.recovery and node.recovery[-1]['state']=='RUNNING',120)
    assert not node.event('send')
    check('startup_quality_static_epoch0_no_goal')
    audit_runtime(node,record,0)
    node.phase=1
    node.wait(lambda:node.moving() and node.truth[-1]['xy'][0]>=1.2,50)
    inject(1,raw_imu=True);node.arrival(1)
    inject(2,estop_during_verify=True);node.arrival(2)
    inject(3,allow=False,raw_imu=True)
    node.phase=2;node.pump(.6)
    assert node.status[-1]['mode']=='finished'
    check('finished_stays_stopped_after_fault')
    result['passed']=True;save()
except BaseException as e:
    result.update(error=repr(e),traceback=traceback.format_exc());save();print(result['traceback'],flush=True)
finally:
    if paused:
        try:
            pid,identity=paused
            if Path('/proc',str(pid),'stat').read_text().rsplit(') ',1)[1].split()[19]==identity:os.kill(pid,signal.SIGCONT)
        except OSError:pass
    if node:
        node.phase=2;node.pump(.2);node.child=None
    for name in ('decision','launcher'):
        p=children.get(name)
        if p and p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=85 if name=='launcher' else 5)
            except subprocess.TimeoutExpired:result['cleanup_error']=name+' did not stop'
    if node:
        (record/'observations.json').write_text(json.dumps(dict(truth=node.truth,commands=node.commands,
            recovery=node.recovery,status=node.status,events=node.events,imu_relay=node.imu_relay_trace),separators=(',',':')))
        node.cleanup()
    for f in streams:f.close()
    if rclpy.ok():rclpy.shutdown()
    result['child_returncodes']={k:v.poll() for k,v in children.items()};save()
assert result['passed'],result
