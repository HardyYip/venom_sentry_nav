import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from example_interfaces.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from tf2_msgs.msg import TFMessage

p=argparse.ArgumentParser()
p.add_argument('--record-dir',type=Path,required=True)
p.add_argument('--rotation-mode',choices=['additive','heading'],default='additive')
args=p.parse_args()
record=args.record_dir.resolve();record.mkdir(parents=True,exist_ok=False)
rclpy.init();node=rclpy.create_node('velocity_safety_contract_test',namespace='/c022_test')
rows=[];statuses=[];transforms=[];results=[]
subs=[node.create_subscription(Twist,'out',lambda m:rows.append((time.monotonic(),[m.linear.x,m.linear.y,m.angular.z])),100),
      node.create_subscription(DiagnosticArray,'safety_status',lambda m:statuses.append((time.monotonic(),m.status[0].message,{v.key:v.value for v in m.status[0].values})),100),
      node.create_subscription(TFMessage,'/tf',lambda m:transforms.extend(m.transforms),100)]
odom_pub=node.create_publisher(Odometry,'odom',10)
plan_pub=node.create_publisher(NavPath,'local_plan',10)
cmd_pub=node.create_publisher(Twist,'in',10)
spin_pub=node.create_publisher(Float32,'cmd_spin',10)
stop_pub=node.create_publisher(Bool,'safety_stop',10)
command=[str(Path(get_package_prefix('fake_vel_transform'))/'lib/fake_vel_transform/fake_vel_transform_node'),'--ros-args','-r','__ns:=/c022_test',
         '-p','robot_base_frame:=base','-p','fake_robot_base_frame:=base_fake',
         '-p','input_cmd_vel_topic:=in','-p','output_cmd_vel_topic:=out','-p','init_spin_speed:=0.7',
         '-p','diagnostics_period:=0.1','-p','rotation_mode:='+args.rotation_mode]
stream=(record/'node.log').open('w');proc=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
settings={'odom':True,'plan':True,'command':True,'frozen':None,'future':0.,'yaw':0.,'invalid_pose':False,'empty_plan':False}
cmd=[.6,.4,.2];last_publish=0.

def pump(seconds):
    global last_publish
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        now=time.monotonic()
        if now-last_publish>=.02:
            last_publish=now
            stamp=node.get_clock().now().to_msg()
            if settings['odom']:
                odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base'
                odom.header.stamp=settings['frozen'] or stamp
                if settings['future']:
                    ns=odom.header.stamp.sec*1000000000+odom.header.stamp.nanosec+int(settings['future']*1e9)
                    odom.header.stamp.sec,odom.header.stamp.nanosec=divmod(ns,1000000000)
                if settings['invalid_pose']:
                    odom.pose.pose.orientation.x=0.;odom.pose.pose.orientation.y=0.
                    odom.pose.pose.orientation.z=0.;odom.pose.pose.orientation.w=0.
                else:
                    odom.pose.pose.orientation.z=math.sin(settings['yaw']/2)
                    odom.pose.pose.orientation.w=math.cos(settings['yaw']/2)
                odom_pub.publish(odom)
            if settings['plan']:
                plan=NavPath();plan.header.frame_id='odom';plan.header.stamp=stamp
                if not settings['empty_plan']:
                    pt=PoseStamped();pt.header=plan.header;pt.pose.orientation.w=1.;plan.poses=[pt,pt]
                plan_pub.publish(plan)
            if settings['command']:
                value=Twist();value.linear.x,value.linear.y,value.angular.z=cmd;cmd_pub.publish(value)
        rclpy.spin_once(node,timeout_sec=.003)
        if proc.poll() is not None:raise RuntimeError('Protected node exited')

def output_close(expected,window=.12,tolerance=1e-5):
    recent=[v for t,v in rows if t>time.monotonic()-window]
    assert len(recent)>=2, 'Missing output heartbeat'
    assert all(all(math.isfinite(x) and abs(x-y)<tolerance for x,y in zip(v,expected)) for v in recent),str(recent[-8:])

def healthy(spin=.7):
    settings.update(odom=True,plan=True,command=True,frozen=None,future=0.,yaw=0.,invalid_pose=False,empty_plan=False)
    cmd[:]=[.6,.4,.2]
    value=Bool();value.data=False;stop_pub.publish(value)
    s=Float32();s.data=spin;spin_pub.publish(s)
    pump(.5)
    output_close([.6,.4,.2+(spin if args.rotation_mode=='additive' else 0.)])

def passed(name,**extra):
    item={'name':name,'passed':True,**extra};results.append(item);print(json.dumps(item),flush=True)

def fault_case(name,change):
    healthy();start=time.monotonic();index=len(rows);change();pump(.8)
    output_close([0.,0.,0.])
    stopped=next((t for t,v in rows[index:] if t>start+.05 and max(abs(x) for x in v)<1e-8),None)
    assert stopped is not None and stopped-start<.65, 'Stop exceeded timeout budget'
    reason=statuses[-1][1] if statuses else ''
    passed(name,detected_stop_delay_s=stopped-start,status=reason)
    healthy();passed(name+'-fresh-recovery')

try:
    end=time.monotonic()+12
    while time.monotonic()<end and (not cmd_pub.get_subscription_count() or not rows):
        rclpy.spin_once(node,timeout_sec=.1)
    assert rows and max(abs(x) for x in rows[-1][1])<1e-8
    assert len(node.get_publishers_info_by_topic('/c022_test/out'))==1, 'Unexpected duplicate test output publisher'
    passed('startup-blocked-without-pose-command')
    healthy();passed('omni-translation-and-spin-addition')
    for angle in [math.pi/2,-math.pi/2,math.pi,.63]:
        settings['yaw']=angle;pump(.3)
        x=.6*math.cos(angle)+.4*math.sin(angle);y=-.6*math.sin(angle)+.4*math.cos(angle)
        output_close([x,y,.9 if args.rotation_mode=='additive' else .2]);passed('coordinate-rotation',yaw=angle,expected=[x,y,.9])
    healthy(spin=0.);cmd[:]=[0.,0.,0.];pump(.15);index=len(rows);settings['command']=False;pump(.25)
    assert rows[index:] and all(max(abs(x) for x in v)<1e-8 for t,v in rows[index:])
    passed('zero-command-does-not-replay-cached-motion')
    fault_case('odometry-receive-loss',lambda:settings.update(odom=False))
    fault_case('frozen-source-stamp-with-live-messages',lambda:settings.update(frozen=node.get_clock().now().to_msg()))
    fault_case('local-plan-loss',lambda:settings.update(plan=False))
    fault_case('command-receive-loss',lambda:settings.update(command=False))
    fault_case('invalid-pose',lambda:settings.update(invalid_pose=True))
    fault_case('future-pose-stamp',lambda:settings.update(future=2.))
    healthy();index=len(transforms);settings['future']=.25;pump(.2);output_close([0.,0.,0.])
    stamp_limit=node.get_clock().now().nanoseconds*1e-9+.1
    assert all(t.header.stamp.sec+t.header.stamp.nanosec*1e-9<=stamp_limit for t in transforms[index:]), 'Future pose leaked into fake TF'
    passed('fractional-future-stamp-does-not-refresh-fake-tf')
    healthy();passed('fractional-future-stamp-recovers-with-new-command')
    fault_case('empty-local-plan',lambda:settings.update(empty_plan=True))
    fault_case('invalid-command',lambda:cmd.__setitem__(0,float('nan')))
    healthy();v=Bool();v.data=True;stop_pub.publish(v);pump(.15);output_close([0.,0.,0.]);passed('software-stop-overrides-positive-command-and-spin')
    settings['command']=False;v.data=False;stop_pub.publish(v);pump(.25);output_close([0.,0.,0.]);passed('stop-release-cannot-replay-pre-stop-command')
    settings['command']=True;pump(.3);output_close([.6,.4,.9 if args.rotation_mode=='additive' else .2]);passed('stop-release-new-command-recovers')
    healthy();cmd[:]=[0.,0.,0.];pump(.2);output_close([0.,0.,.7 if args.rotation_mode=='additive' else 0.]);passed('normal-zero-respects-rotation-mode')
    if args.rotation_mode=='heading':
        healthy();settings['yaw']=0.;target=1.2;cmd[:]=[0.,0.,0.]
        until=time.monotonic()+4.;last=time.monotonic()
        while time.monotonic()<until:
            now=time.monotonic();dt=now-last;last=now
            if rows:settings['yaw']+=rows[-1][1][2]*dt
            cmd[2]=2.*(target-settings['yaw']);pump(.02)
        assert abs(target-settings['yaw'])<.03, 'Heading loop biased by requested spin'
        passed('heading-mode-rejects-additive-spin-bias',final_heading_error=target-settings['yaw'])
    assert statuses and any('STOPPED' in v for _,v,_ in statuses)
    assert any('RECOVERING' in v for _,v,_ in statuses)
    assert any('ACTIVE' in v for _,v,_ in statuses)
    assert transforms and any(t.child_frame_id=='base_fake' for t in transforms)
    passed('diagnostics-and-fake-frame')
    value={'passed':True,'results':results,'thresholds_s':{'command':.5,'pose':.5,'sync':.5},'scope':'isolated ROS node contract, not physical braking distance'}
except BaseException as error:
    value={'passed':False,'error':repr(error),'results':results};print(json.dumps(value),flush=True)
finally:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
    stream.close();(record/'results.json').write_text(json.dumps(value,indent=2))
    (record/'observations.json').write_text(json.dumps({'outputs':rows,'statuses':statuses}))
    node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if value['passed'] else 1)
