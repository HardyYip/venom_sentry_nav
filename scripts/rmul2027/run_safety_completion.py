if __name__ == "__main__":
    from storage_entry import ensure_storage_guard
    ensure_storage_guard(__file__)

import argparse
import bisect
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
import threading

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from example_interfaces.msg import Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool as StatusBool
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
import yaml

parser=argparse.ArgumentParser()
parser.add_argument('--record-bag', action='store_true', help='Opt in to a light, guarded recording; default is no bag')
parser.add_argument('--record-dir',type=Path,required=True)
parser.add_argument('--scenario',choices=['watchdog','localization','lio-gap','lio-buffered'],required=True)
parser.add_argument('--params-file',type=Path)
args=parser.parse_args()
record=args.record_dir.resolve();record.mkdir(parents=True,exist_ok=False)
folder=Path(__file__).resolve().parent;workspace=folder.parents[3]
params=yaml.safe_load((args.params_file or folder/'config/nav2_test.yaml').read_text())
params['point_lio']['ros__parameters']['max_imu_gap']=.1
if args.scenario=='localization':params['loam_interface']['ros__parameters']['state_estimation_topic']='fault_lio_relay'
if args.scenario=='lio-gap':params['point_lio']['ros__parameters']['common']['imu_topic']='fault_imu_relay'
plugins=params['bt_navigator']['ros__parameters']['plugin_lib_names']
if 'pb_pose_fresh_condition_bt_node' not in plugins:plugins.append('pb_pose_fresh_condition_bt_node')
if args.scenario=='watchdog':params['fake_vel_transform']['ros__parameters']['input_cmd_vel_topic']='fault_test_cmd'
param_file=record/'params.yaml';param_file.write_text(yaml.safe_dump(params,sort_keys=False))
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(folder/'config/fastdds_local_64m.xml')
meta={'owner':'Codex C024','passed':False,'scenario':args.scenario,'processes':{},'checks':[]}
children={};streams=[];paused_pid=None;node=None;goal_handle=None;resumed=False

def save():
    (record/'session.json').write_text(json.dumps(meta,indent=2))

def check(condition,name,**values):
    meta['checks'].append({'name':name,'passed':bool(condition),**values});save()
    if not condition:raise AssertionError(name)

def launch(name,command):
    stream=(record/(name+'.log')).open('w');streams.append(stream)
    process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    children[name]=process;meta['processes'][name]={'pid':process.pid,'command':command};save();return process

class Probe(Node):
    def __init__(self):
        super().__init__('safety_completion_probe',namespace='/red_standard_robot1')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.rows=[];self.commands=[];self.status=[];self.watchdog=[];self.bridge_watchdog=[];self.truth=None
        self.errors=[];self.last_guard=0.;self.relay=True;self.imu_relay=True;self.lio_rows=[];self.imu_rows=[];self.emit=False;self.phase='preparing';self.last_pub=0.
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.subs=[self.create_subscription(Odometry,'chassis_odometry_gt',self.pose,100),
          self.create_subscription(Twist,'cmd_vel',self.command,100),
          self.create_subscription(DiagnosticArray,'safety_status',self.diagnostic,100),
          self.create_subscription(StatusBool,'cmd_vel_timed_out',self.timeout,100),
          self.create_subscription(StatusBool,'robot_base/command_timed_out',self.bridge_timeout,100)]
        self.pub=self.create_publisher(Twist,'fault_test_cmd',10)
        self.spin_pub=self.create_publisher(Float32,'cmd_spin',10)
        self.stop_pub=self.create_publisher(Bool,'safety_stop',10)
        self.action=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.relay_pub=self.create_publisher(Odometry,'fault_lio_relay',10)
        self.lio_sub=self.create_subscription(Odometry,'aft_mapped_to_init',self.lio,100)
        self.imu_relay_pub=self.create_publisher(Imu,'fault_imu_relay',400)
        self.imu_sub=self.create_subscription(Imu,'livox/imu',self.imu,qos_profile_sensor_data)
    def imu(self,msg):
        self.imu_rows.append({'wall':time.monotonic(),'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'forwarded':self.imu_relay,'phase':self.phase})
        if args.scenario=='lio-gap' and self.imu_relay:self.imu_relay_pub.publish(msg)
    def lio(self,msg):
        self.lio_rows.append({'wall':time.monotonic(),'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'phase':self.phase})
        if args.scenario=='localization' and self.relay:self.relay_pub.publish(msg)
    def pose(self,msg):
        self.truth=msg;p=msg.pose.pose.position;v=msg.twist.twist
        self.rows.append({'wall':time.monotonic(),'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'phase':self.phase,'position':[p.x,p.y,p.z],'speed':[v.linear.x,v.linear.y,v.angular.z]})
    def command(self,msg):self.commands.append({'wall':time.monotonic(),'phase':self.phase,'value':[msg.linear.x,msg.linear.y,msg.angular.z]})
    def diagnostic(self,msg):self.status.append({'wall':time.monotonic(),'phase':self.phase,'message':msg.status[0].message})
    def timeout(self,msg):self.watchdog.append({'wall':time.monotonic(),'phase':self.phase,'timed_out':msg.data})
    def bridge_timeout(self,msg):self.bridge_watchdog.append({'wall':time.monotonic(),'phase':self.phase,'timed_out':msg.data})
    def pump(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if self.emit and time.monotonic()-self.last_pub>.04:
                self.last_pub=time.monotonic();msg=Twist();msg.linear.x=.25;self.pub.publish(msg)
                spin=Float32();spin.data=.7;self.spin_pub.publish(spin)
            rclpy.spin_once(self,timeout_sec=.005)
            if time.monotonic()-self.last_guard>.05:
                self.last_guard=time.monotonic();self.check_localization()
    def check_localization(self):
        if len(self.rows)<2:return
        try:t=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
        except Exception:return
        stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
        index=bisect.bisect_left([v['stamp'] for v in self.rows],stamp)
        if not 0<index<len(self.rows):return
        a,b=self.rows[index-1],self.rows[index];span=b['stamp']-a['stamp']
        if span<=0 or span>.05:return
        ratio=(stamp-a['stamp'])/span;truth=[a['position'][i]+ratio*(b['position'][i]-a['position'][i]) for i in range(2)]
        p=t.transform.translation;error=math.hypot(p.x-truth[0]-4.8,p.y-truth[1]+3.0)
        value={'stamp':stamp,'estimate':[p.x,p.y],'truth_map':[truth[0]+4.8,truth[1]-3.0],'error_m':error,'phase':self.phase};self.errors.append(value)
        if error>.35:
            (record/'guard-trigger.json').write_text(json.dumps(value,indent=2));raise RuntimeError('Aligned localization error exceeds unchanged 0.35m guard')
    def wait(self,condition,timeout):
        end=time.monotonic()+timeout
        while not condition() and time.monotonic()<end:self.pump(.02)
        if not condition():raise TimeoutError('Condition timed out in phase '+self.phase)
    def future(self,value,timeout):
        self.wait(value.done,timeout);return value.result()
    def moving(self):
        return self.truth and math.hypot(self.truth.twist.twist.linear.x,self.truth.twist.twist.linear.y)>.08
    def speed(self):
        if not self.truth:return math.inf
        v=self.truth.twist.twist;return max(abs(v.linear.x),abs(v.linear.y),abs(v.angular.z))
    def send(self,xy):
        self.wait(lambda:self.action.server_is_ready(),10)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=float(xy[0]);goal.pose.pose.position.y=float(xy[1]);goal.pose.pose.orientation.w=1.
        handle=self.future(self.action.send_goal_async(goal),10)
        check(handle.accepted,'goal accepted',goal=xy)
        meta.setdefault('goals',[]).append({'xy':xy,'uuid':bytes(handle.goal_id.uuid).hex()});save()
        return handle

def owned_pid(navlog,executable):
    matches=re.findall(r'\['+re.escape(executable)+r'-\d+\]: process started with pid \[(\d+)\]',navlog.read_text(errors='replace'))
    if len(matches)!=1:raise RuntimeError('Expected one owned process: '+executable)
    pid=int(matches[0]);cmd=Path('/proc',str(pid),'cmdline').read_bytes()
    if executable.encode() not in cmd:raise RuntimeError('Process identity changed')
    return pid

try:
    rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static']);node=Probe()
    wrapper=launch('launcher',['bash',str(folder/'start_rmul2027.sh'),'nav','--params-file',str(param_file),'--gz-world-path',str(folder/'config/gz_world_test.yaml'),'--field-contact','cad-flat-floor','--render-mode','gpu','--tf2-library-dir',str(workspace/'artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib')])
    end=time.monotonic()+150;navlog=None
    while time.monotonic()<end:
        if wrapper.poll() is not None:raise RuntimeError('Launcher exited before readiness')
        for line in (record/'launcher.log').read_text(errors='replace').splitlines():
            if 'RMUL 2027 nav started. Logs: ' in line:navlog=Path(line.split('Logs: ',1)[1])/'navigation.log'
        if navlog:break
        node.pump(.2)
    if not navlog:raise TimeoutError('Navigation launch did not finish')
    meta['navigation_log']=str(navlog)
    meta['runtime_hashes']={str(path.relative_to(workspace)):hashlib.sha256(path.read_bytes()).hexdigest() for path in [workspace/'build/rmoss_gz_plugins/libMecanumDrive2.so',workspace/'install/rmoss_gz_base/lib/librmoss_gz_base.so',workspace/'install/point_lio/lib/point_lio/pointlio_mapping'] if path.is_file()};save()
    topics=['/clock']+['/red_standard_robot1/'+v for v in ['tf','tf_static','livox/imu','velodyne_points','fault_imu_relay','fault_lio_relay','chassis_odometry_gt','odometry','aft_mapped_to_init','cmd_vel','cmd_vel_nav2_result','fault_test_cmd','cmd_spin','safety_stop','safety_status','cmd_vel_timed_out','robot_base/command_timed_out','plan','local_plan','global_costmap/costmap','global_costmap/costmap_updates','local_costmap/costmap','local_costmap/costmap_updates','navigate_to_pose/_action/status','navigate_to_pose/_action/feedback','navigate_through_poses/_action/status','navigate_through_poses/_action/feedback']]
    topics=[topic for topic in topics if 'velodyne_points' not in topic]
    meta['record_bag']=args.record_bag
    meta['recorded_topics']=topics if args.record_bag else []
    save()
    if args.record_bag:
        launch('recorder',['ros2','bag','record','--include-hidden-topics','--max-cache-size','67108864','--max-bag-size','33554432','-o',str(record/'inputs'),*topics])
    ready=launch('ready',[sys.executable,str(folder/'run_baseline_audit.py'),'--record-dir',str(record/'ready'),'--last-case','ready','--navigation-log',str(navlog)])
    node.wait(lambda:ready.poll() is not None,180)
    check(ready.returncode==0,'readiness passed')
    observer=json.loads((record/'ready/run.json').read_text()).get('observer_pid')
    if observer:
        p=Path('/proc',str(observer),'cmdline')
        if p.exists() and str(record).encode() in p.read_bytes():os.kill(observer,signal.SIGINT)
    node.pump(.5)
    if args.scenario=='watchdog':
        node.emit=True;node.phase='before-freeze';node.wait(node.moving,8);node.pump(.4)
        paused_pid=owned_pid(navlog,'component_container_isolated');meta['paused_container_pid']=paused_pid
        check(abs(node.truth.twist.twist.angular.z)>.2,'nonzero spin before container freeze')
        node.phase='container-frozen';start=time.monotonic();os.kill(paused_pid,signal.SIGSTOP);node.emit=False;node.pump(1.5)
        transitions=[v for v in node.bridge_watchdog if v['wall']>start and v['timed_out']]
        check(bool(transitions),'independent watchdog timeout observed')
        delay=transitions[0]['wall']-start
        check(.25<delay<.8,'watchdog timeout delay',wall_seconds=delay)
        tail=[v for v in node.rows if v['wall']>start+1.1]
        check(bool(tail) and max(max(abs(x) for x in v['speed']) for v in tail)<.03,'chassis physically stopped while software guard frozen',tail_speed_max=max((max(abs(x) for x in v['speed']) for v in tail),default=math.inf))
        outputs=[v for v in node.commands if v['wall']>start+.2]
        check(not outputs,'no upstream zero command needed for watchdog stop',messages=len(outputs))
        node.phase='container-resumed-no-command';os.kill(paused_pid,signal.SIGCONT);paused_pid=None;node.pump(1.2)
        check(node.speed()<.03,'no cached motion replay after guard resumes')
        node.phase='fresh-commands';node.emit=True;node.wait(node.moving,10);node.pump(.3)
        check(True,'fresh commands restore motion')
        paused_pid=owned_pid(navlog.parent/'simulator.log','rmua19_robot_base');node.phase='bridge-frozen';start=time.monotonic();os.kill(paused_pid,signal.SIGSTOP);node.pump(1.5)
        transitions=[v for v in node.watchdog if v['wall']>start and v['timed_out']]
        check(bool(transitions) and transitions[0]['wall']-start<.8,'plugin watchdog stops when bridge freezes',wall_seconds=transitions[0]['wall']-start if transitions else None)
        check(node.speed()<.03,'physical stop while bridge frozen')
        node.emit=False;os.kill(paused_pid,signal.SIGCONT);paused_pid=None;node.pump(1.2)
        check(node.speed()<.03,'bridge stops after resume without fresh command stream')
        node.emit=True;node.wait(node.moving,10);node.pump(.3)
        pid=owned_pid(navlog.parent/'simulator.log','rmua19_robot_base');node.phase='bridge-exited';start=time.monotonic();os.kill(pid,signal.SIGTERM);node.pump(1.5)
        transitions=[v for v in node.watchdog if v['wall']>start and v['timed_out']]
        check(bool(transitions) and transitions[0]['wall']-start<.8,'plugin watchdog stops after bridge process exits',wall_seconds=transitions[0]['wall']-start if transitions else None)
        check(node.speed()<.03,'physical stop after bridge process exit')
    else:
        node.phase='active-goal';goal_handle=node.send([9.3,-.2]);result=goal_handle.get_result_async()
        node.wait(node.moving,12);node.pump(.25)
        node.phase='localization-paused';start=time.monotonic()
        if args.scenario=='lio-buffered':
            paused_pid=owned_pid(navlog,'pointlio_mapping');meta['paused_lio_pid']=paused_pid;os.kill(paused_pid,signal.SIGSTOP)
        elif args.scenario=='lio-gap':node.imu_relay=False
        else:node.relay=False
        response=node.future(result,8);check(response.status==GoalStatus.STATUS_ABORTED,'stale localization aborts current Nav2 goal',status=response.status,wall_seconds=time.monotonic()-start)
        goal_handle=None;node.pump(.8);check(node.speed()<.03,'physical stop after localization loss')
        check('PoseFresh FAILURE' in navlog.read_text(errors='replace'),'Nav2 freshness gate caused failure')
        node.phase='localization-resumed'
        if args.scenario=='lio-buffered':os.kill(paused_pid,signal.SIGCONT);paused_pid=None
        elif args.scenario=='lio-gap':node.imu_relay=True
        else:node.relay=True
        node.pump(8.0 if args.scenario=='lio-buffered' else 3.0);tail=[v for v in node.commands if v['wall']>time.monotonic()-1.0]
        check(bool(tail) and all(max(abs(x) for x in v['value'])<1e-5 for v in tail),'old goal does not restart automatically after localization returns')
        if args.scenario=='lio-gap':
            check('IMU_INPUT_FAULT_LATCH' in navlog.read_text(errors='replace'),'lost IMU history causes explicit latched localization fault')
            recent=[v for v in node.lio_rows if v['wall']>time.monotonic()-1.0]
            check(not recent,'latched LIO refuses to publish falsely fresh poses',messages=len(recent))
            node.phase='goal-while-latched';goal_handle=node.send([9.3,-.2]);response=node.future(goal_handle.get_result_async(),8);goal_handle=None
            check(response.status==GoalStatus.STATUS_ABORTED,'new goal rejected by freshness gate until localization restart',status=response.status)
            node.pump(.6);check(node.speed()<.03,'robot remains physically stopped while localization latched')
        else:
            before_recovery=node.buffer.lookup_transform('map','base_footprint',rclpy.time.Time()).transform.translation
            node.phase='new-explicit-goal'
            recovery=launch('recovery',[sys.executable,str(folder/'verify_simulation.py'),'--mode','nav','--record-dir',str(record/'recovery'),'--startup-timeout','45','--goal-timeout','180','--stability-seconds','2'])
            node.wait(lambda:recovery.poll() is not None,240)
            check(recovery.returncode==0,'new explicit safe goal succeeds after fresh localization',record_dir=str(record/'recovery'))
            node.pump(.3);check(node.speed()<.03,'recovery goal physically stopped')
            current=node.buffer.lookup_transform('map','base_footprint',rclpy.time.Time()).transform.translation
            node.phase='through-poses-smoke';client=ActionClient(node,NavigateThroughPoses,'navigate_through_poses')
            node.wait(client.server_is_ready,10);goal=NavigateThroughPoses.Goal()
            for ratio in [.5,1.0]:
                pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg();pose.pose.position.x=current.x+ratio*(before_recovery.x-current.x);pose.pose.position.y=current.y+ratio*(before_recovery.y-current.y);pose.pose.orientation.w=1.;goal.poses.append(pose)
            goal_handle=node.future(client.send_goal_async(goal),10);check(goal_handle.accepted,'guarded NavigateThroughPoses accepted')
            response=node.future(goal_handle.get_result_async(),180);goal_handle=None
            check(response.status==GoalStatus.STATUS_SUCCEEDED,'guarded NavigateThroughPoses succeeds',status=response.status);node.pump(1.2);check(node.speed()<.03,'through-poses physically stopped')
    meta['checks_passed']=True
except BaseException as error:
    meta['error']=repr(error)
finally:
    if paused_pid:
        try:os.kill(paused_pid,signal.SIGCONT)
        except ProcessLookupError:pass
    if node:
        try:
            node.emit=False;stop=Bool();stop.data=True;node.stop_pub.publish(stop);node.pump(.15)
            if goal_handle:node.future(goal_handle.cancel_goal_async(),3)
        except Exception:pass
        (record/'observations.json').write_text(json.dumps({'truth':node.rows,'commands':node.commands,'diagnostics':node.status,'watchdog':node.watchdog,'bridge_watchdog':node.bridge_watchdog,'aligned_errors':node.errors,'lio':node.lio_rows,'imu_input':node.imu_rows}))
    for name in ['recovery','ready','recorder','launcher']:
        proc=children.get(name)
        if proc and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=55)
            except subprocess.TimeoutExpired:meta.setdefault('cleanup_pending',[]).append(proc.pid);meta['passed']=False
    ownership=record/'ready/run.json'
    if ownership.exists():
        observer=json.loads(ownership.read_text()).get('observer_pid')
        if observer:
            proc=Path('/proc',str(observer));cmd=proc/'cmdline'
            if cmd.exists() and str(record).encode() in cmd.read_bytes():
                os.kill(observer,signal.SIGINT);end=time.monotonic()+4
                while proc.exists() and time.monotonic()<end:time.sleep(.05)
                if cmd.exists() and str(record).encode() in cmd.read_bytes():os.kill(observer,signal.SIGKILL)
    for stream in streams:stream.close()
    meta['checks_finished']=True;meta['cleanup_state']='closing-dds';save()
    def dds_deadline():
        meta.update(passed=False,finished=True,cleanup_state='dds_shutdown_timeout');save()
        os._exit(2)
    timer=threading.Timer(8,dds_deadline);timer.daemon=True;timer.start()
    if node:
        node.destroy_node();rclpy.shutdown()
    timer.cancel();meta['finished']=True;meta['cleanup_state']='complete'
    meta['passed']=bool(meta.get('checks_passed')) and not meta.get('cleanup_pending')
    save();print(json.dumps(meta),flush=True)
raise SystemExit(0 if meta['passed'] else 1)
