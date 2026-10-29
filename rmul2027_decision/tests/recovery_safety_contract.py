import argparse,json,signal,subprocess,time
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_prefix
from example_interfaces.msg import Bool,Float32
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from diagnostic_msgs.msg import DiagnosticArray
p=argparse.ArgumentParser();p.add_argument('--record-dir',type=Path,required=True);args=p.parse_args()
root=args.record_dir;root.mkdir(parents=True,exist_ok=False)
rclpy.init();n=rclpy.create_node('gate_contract',namespace='/gate_test')
outputs=[];diagnostics=[];results=[]
subs=[n.create_subscription(Twist,'out',lambda m:outputs.append((time.monotonic(),[m.linear.x,m.linear.y,m.angular.z])),100),
      n.create_subscription(DiagnosticArray,'safety_status',lambda m:diagnostics.append((time.monotonic(),m.status[0].message)),100)]
pubs={name:n.create_publisher(typ,name,10) for name,typ in
      [('odom',Odometry),('local_plan',NavPath),('in',Twist),('cmd_spin',Float32),('safety_stop',Bool),('decision_stop',Bool),('recovery/stop',Bool)]}
f=(root/'node.log').open('w')
cmd=[str(Path(get_package_prefix('fake_vel_transform'))/'lib/fake_vel_transform/fake_vel_transform_node'),
     '--ros-args','-r','__ns:=/gate_test','-p','robot_base_frame:=base','-p','fake_robot_base_frame:=base_fake',
     '-p','input_cmd_vel_topic:=in','-p','output_cmd_vel_topic:=out','-p','decision_stop_timeout:=0.3','-p','recovery_stop_timeout:=0.3','-p','diagnostics_period:=0.1']
proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
state=dict(decision=True,software=False,heartbeat=False,command=True,recovery=True,recovery_hb=False)
def pump(seconds):
    end=time.monotonic()+seconds;last=0
    while time.monotonic()<end:
        if time.monotonic()-last>.02:
            last=time.monotonic();stamp=n.get_clock().now().to_msg()
            o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base';o.pose.pose.orientation.w=1.
            pubs['odom'].publish(o)
            plan=NavPath();plan.header=o.header;pt=PoseStamped();pt.pose.orientation.w=1.;plan.poses=[pt,pt];pubs['local_plan'].publish(plan)
            if state['command']:
                c=Twist();c.linear.x=.6;c.linear.y=.4;c.angular.z=.2;pubs['in'].publish(c)
            pubs['cmd_spin'].publish(Float32(data=.7))
            pubs['safety_stop'].publish(Bool(data=state['software']))
            if state['heartbeat']:pubs['decision_stop'].publish(Bool(data=state['decision']))
            if state['recovery_hb']:pubs['recovery/stop'].publish(Bool(data=state['recovery']))
        rclpy.spin_once(n,timeout_sec=.003)
        assert proc.poll() is None,'Safety node exited'
def expect(name,value):
    rows=[v for t,v in outputs if t>time.monotonic()-.1]
    assert len(rows)>=2,'No output heartbeat'
    assert all(all(abs(x-y)<1e-5 for x,y in zip(v,value)) for v in rows),(name,rows[-8:])
    results.append(dict(name=name,passed=True))
def moving():
    state.update(decision=False,software=False,heartbeat=True,command=True,recovery=False,recovery_hb=True);pump(.4);expect('healthy motion',[.6,.4,.9])
result=dict(passed=False)
try:
    pump(1);expect('missing decision heartbeat inhibits startup',[0,0,0])
    state['heartbeat']=True;pump(.2);expect('explicit decision inhibit',[0,0,0])
    moving()
    state['software']=True;pump(.2);expect('decision release cannot release external stop',[0,0,0])
    state.update(software=False,decision=True);pump(.2);expect('external release cannot release decision stop',[0,0,0])
    moving();start=time.monotonic();state['heartbeat']=False;pump(.7)
    expect('heartbeat stream loss overrides positive command and spin',[0,0,0])
    first=next(t for t,v in outputs if t>start+.05 and max(abs(x) for x in v)<1e-8)
    assert first-start<.65
    state.update(heartbeat=True,command=False);pump(.2);expect('restart cannot replay cached motion',[0,0,0])
    moving()
    state['recovery']=True;pump(.2);expect('recovery stop overrides both other releases',[0,0,0])
    state.update(recovery=False,decision=True);pump(.2);expect('recovery release cannot override decision stop',[0,0,0])
    state.update(decision=False,software=True);pump(.2);expect('recovery release cannot override external stop',[0,0,0])
    moving();start_recovery=time.monotonic();state['recovery_hb']=False;pump(.7)
    expect('lost recovery process heartbeat overrides motion',[0,0,0])
    first_recovery=next(t for t,v in outputs if t>start_recovery+.05 and max(abs(x) for x in v)<1e-8)
    assert first_recovery-start_recovery<.65
    state.update(recovery_hb=True,command=False);pump(.2);expect('recovery heartbeat cannot replay cached motion',[0,0,0])
    moving()
    result=dict(passed=True,results=results,heartbeat_stop_delay_s=first-start,scope='actual isolated safety binary; physical stop not claimed')
finally:
    proc.send_signal(signal.SIGINT);proc.wait(timeout=5);f.close()
    (root/'result.json').write_text(json.dumps(result,indent=2))
    (root/'observations.json').write_text(json.dumps(dict(outputs=outputs,diagnostics=diagnostics)))
    n.destroy_node();rclpy.shutdown()
print(json.dumps(result),flush=True)
assert result['passed']
