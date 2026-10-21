if __name__ == "__main__":
    from storage_entry import ensure_storage_guard
    ensure_storage_guard(__file__)

import argparse
import bisect
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from example_interfaces.msg import Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener
import yaml

p=argparse.ArgumentParser();p.add_argument('--record-dir',type=Path,required=True);args=p.parse_args()
record=args.record_dir.resolve();record.mkdir(parents=True,exist_ok=False)
folder=Path(__file__).resolve().parent;workspace=folder.parents[3]
params=yaml.safe_load((folder/'config/nav2_original_speed_early_brake.yaml').read_text())
params['fake_vel_transform']['ros__parameters']['input_cmd_vel_topic']='fault_test_cmd'
profile=folder/'config/fastdds_local_64m.xml';os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(profile)
param_file=record/'fault-params.yaml';param_file.write_text(yaml.safe_dump(params,sort_keys=False))
meta={'passed':False,'processes':{},'scope':'owned fresh simulation, no navigation goal, runtime node is stopping the chassis'}
children={};streams=[];lio_pid=None;frozen=False;node=None

def save():
    (record/'session.json').write_text(json.dumps(meta,indent=2))

def launch(name,command):
    f=(record/(name+'.log')).open('w');streams.append(f)
    proc=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    children[name]=proc;meta['processes'][name]={'pid':proc.pid,'command':command};save();return proc

class Probe(rclpy.node.Node):
    def __init__(self):
        super().__init__('runtime_fault_sim_probe',namespace='/red_standard_robot1')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.rows=[];self.commands=[];self.status=[];self.truth=None;self.emit_command=True;self.spin=.7
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.subs=[self.create_subscription(Odometry,'chassis_odometry_gt',self.pose,100),
                   self.create_subscription(Twist,'cmd_vel',self.command,100),
                   self.create_subscription(DiagnosticArray,'safety_status',self.diagnostic,100)]
        self.pub=self.create_publisher(Twist,'fault_test_cmd',10)
        self.spin_pub=self.create_publisher(Float32,'cmd_spin',10)
        self.stop_pub=self.create_publisher(Bool,'safety_stop',10)
        self.last_pub=0.;self.phase='preparing'
    def pose(self,m):
        self.truth=m;p=m.pose.pose.position;v=m.twist.twist
        self.rows.append({'wall':time.monotonic(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                          'phase':self.phase,'position':[p.x,p.y,p.z],
                          'speed':[v.linear.x,v.linear.y,v.angular.z]})
    def command(self,m):self.commands.append({'wall':time.monotonic(),'phase':self.phase,'value':[m.linear.x,m.linear.y,m.angular.z]})
    def diagnostic(self,m):self.status.append({'wall':time.monotonic(),'phase':self.phase,'message':m.status[0].message})
    def pump(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if self.emit_command and time.monotonic()-self.last_pub>.04:
                self.last_pub=time.monotonic();m=Twist();m.linear.x=.25;self.pub.publish(m)
                spin=Float32();spin.data=self.spin;self.spin_pub.publish(spin)
            rclpy.spin_once(self,timeout_sec=.003)
    def wait(self,condition,timeout):
        end=time.monotonic()+timeout
        while not condition() and time.monotonic()<end:self.pump(.03)
        assert condition(),'Expected physical condition not reached'

try:
    wrapper=launch('launcher',['bash',str(folder/'start_rmul2027.sh'),'nav','--params-file',str(param_file),
                             '--gz-world-path',str(folder/'config/gz_world_test.yaml'),'--field-contact','cad-flat-floor',
                             '--render-mode','gpu','--tf2-library-dir',str(workspace/'artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib')])
    end=time.monotonic()+140;navlog=None
    while time.monotonic()<end:
        assert wrapper.poll() is None,'Launch exited'
        for line in (record/'launcher.log').read_text(errors='replace').splitlines():
            if 'RMUL 2027 nav started. Logs: ' in line:navlog=Path(line.split('Logs: ',1)[1])/'navigation.log'
        if navlog:break
        time.sleep(.2)
    assert navlog,'No launch log';meta['navigation_log']=str(navlog)
    topics=['/clock']+['/red_standard_robot1/'+v for v in ['tf','tf_static','chassis_odometry_gt','odometry','cmd_vel','fault_test_cmd','cmd_spin','safety_stop','safety_status','registered_scan']]
    launch('recorder',['ros2','bag','record','-o',str(record/'inputs'),*topics])
    ready=launch('ready',[sys.executable,str(folder/'run_baseline_audit.py'),'--record-dir',str(record/'ready'),'--last-case','ready','--navigation-log',str(navlog)])
    assert ready.wait(timeout=180)==0,'Readiness failed'
    oid=json.loads((record/'ready/run.json').read_text()).get('observer_pid')
    if oid:
        cmd=Path('/proc',str(oid),'cmdline')
        if cmd.exists() and str(record).encode() in cmd.read_bytes():os.kill(oid,signal.SIGINT)
    matches=re.findall(r'\[pointlio_mapping-\d+\]: process started with pid \[(\d+)\]',navlog.read_text(errors='replace'))
    assert len(matches)==1,'Cannot identify owned Point-LIO process';lio_pid=int(matches[0])
    command_line=Path('/proc',str(lio_pid),'cmdline').read_bytes()
    assert b'pointlio_mapping' in command_line and b'red_standard_robot1' in command_line
    rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static']);node=Probe();node.pump(.5)
    node.wait(lambda:node.truth is not None and math.hypot(node.truth.twist.twist.linear.x,node.truth.twist.twist.linear.y)>.1 and abs(node.truth.twist.twist.angular.z)>.2,5)
    node.phase='moving';node.pump(.4);before=node.truth.pose.pose.position;meta['before_fault_position']=[before.x,before.y]
    node.phase='lio-paused';start=time.monotonic();os.kill(lio_pid,signal.SIGSTOP);frozen=True;meta['fault_started_wall']=start;save()
    node.pump(1.25)
    stopped=[v for v in node.commands if v['wall']>start+.05 and max(abs(x) for x in v['value'])<1e-8]
    assert stopped,'Runtime node never emitted zero';delay=stopped[0]['wall']-start
    assert delay<.7,'Runtime stop was late'
    physical=[v for v in node.rows if v['wall']>start+.95]
    assert physical and max(max(abs(x) for x in v['speed']) for v in physical)<.03,'Chassis did not physically stop'
    after=node.truth.pose.pose.position;meta.update(runtime_zero_delay_s=delay,paused_chassis_speed_max=max(max(abs(x) for x in v['speed']) for v in physical),
                                                 fault_displacement_m=math.hypot(after.x-before.x,after.y-before.y))
    node.phase='lio-resumed';os.kill(lio_pid,signal.SIGCONT);frozen=False
    node.wait(lambda:node.truth is not None and math.hypot(node.truth.twist.twist.linear.x,node.truth.twist.twist.linear.y)>.1,12)
    node.pump(.35);meta['resumed_motion']=True
    stop=Bool();stop.data=True;node.stop_pub.publish(stop);node.phase='software-stop';node.pump(.6)
    assert node.truth and max(abs(node.truth.twist.twist.linear.x),abs(node.truth.twist.twist.linear.y),abs(node.truth.twist.twist.angular.z))<.03
    meta.update(passed=True,software_stop_physical=True,nav_goals_sent=0)
except BaseException as error:
    meta['error']=repr(error)
finally:
    if frozen and lio_pid:
        try:os.kill(lio_pid,signal.SIGCONT)
        except ProcessLookupError:pass
    if node:
        try:
            stop=Bool();stop.data=True;node.stop_pub.publish(stop);node.pump(.2)
        except Exception:pass
        (record/'observations.json').write_text(json.dumps({'truth':node.rows,'commands':node.commands,'diagnostics':node.status}))
        node.destroy_node();rclpy.shutdown()
    for name in ['ready','recorder','launcher']:
        proc=children.get(name)
        if proc and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=55)
            except subprocess.TimeoutExpired:meta.setdefault('cleanup_pending',[]).append(proc.pid);meta['passed']=False
    for f in streams:f.close()
    save();print(json.dumps(meta),flush=True)
raise SystemExit(0 if meta['passed'] else 1)
