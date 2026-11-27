"""Drive a neutral cart with the archived Nav2 planner and controller in native Gazebo."""
from collections import deque
from pathlib import Path
import json
import math
import sys
import time

import numpy as np
import yaml
from PIL import Image
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from scipy.spatial.transform import Rotation
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

out = Path(sys.argv[1])
case = json.loads((out/'case.json').read_text())
def stamp(msg): return msg.sec + msg.nanosec*1e-9
def rot(r,p,y):
    a,b,c,d,e,f=math.cos(r), math.sin(r), math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[e*c,e*d*b-f*a,e*d*a+f*b],[f*c,f*d*b+e*a,f*d*a-e*b],[-d,c*b,c*a]])
def tf(parent,child,when):
    t=TransformStamped(); t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=when;t.transform.rotation.w=1.;return t

class Trial(Node):
    def __init__(self):
        super().__init__('cart_fixture')
        self.odom=deque(maxlen=300);self.box=None
        self.dt_last=0.;self.count={'odin':0,'mid':0};self.returns={'odin':0,'mid':0}
        layout=next(Path('/baseline/fixed-left').glob('*/cad-mid-odin-*/map-boundary-margin.yaml'))
        meta=yaml.safe_load(layout.read_text())
        image=np.flipud(np.asarray(Image.open(layout.parent/meta['image']).convert('L')))
        prob=(255-image.astype(float))/255
        self.base=np.where(prob>meta['occupied_thresh'],100,np.where(prob<meta['free_thresh'],0,-1)).astype('int8')
        self.origin=np.asarray(meta['origin'][:2])-np.asarray(case['start'])
        self.res=meta['resolution'];self.contacts=0;self.peak=0.;self.peak_cmd=0.;self.commands=[]
        self.gap=math.inf;self.measured_speed=0.;self.goal=np.array(case['goal'])
        self.last_cmd=math.inf;self.map_sent=False;self.moving_started=None
        self.samples=[];self.plan_count=0;self.box_history=[];self.handles=[];self.trace=(out/'trace.jsonl').open('w')
        self.broadcaster=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        when=self.get_clock().now().to_msg()
        transforms=[tf('map','odom',when),tf('base_link','base_footprint',when)]
        for key,pose in case['poses'].items():
            frame='odin_scan' if key=='odin' else 'front_mid360_scan'
            t=tf('base_link',frame,when)
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=pose[:3]
            q=Rotation.from_euler('xyz',pose[3:]).as_quat()
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
            transforms.append(t)
        self.static.sendTransform(transforms)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.empty_pub=self.create_publisher(OccupancyGrid,'map',qos)
        self.box_cmd=self.create_publisher(Twist,'/box/cmd',10)
        self.cloud_pub={}
        for key,topic in [('odin','odin/obstacles'),('mid','mid/geometric_obstacles')]:
            self.cloud_pub[key]=self.create_publisher(PointCloud2,topic,qos_profile_sensor_data)
            self.create_subscription(PointCloud2,'/'+key+'/scan/points',lambda msg,k=key:self.cloud(k,msg),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/cart/odom',self.pose,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/box/odom',self.box_pose,qos_profile_sensor_data)
        self.create_subscription(Contacts,'/world/navigation/model/cart/link/body/sensor/contacts/contact',self.touch,qos_profile_sensor_data)
        self.create_subscription(Twist,'/cart/cmd',self.command,10)
        self.create_timer(.15,self.publish_map)
        self.navigator=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.create_subscription(NavPath,'plan',self.new_plan,10)
    def new_plan(self,m):
        if m.poses:self.plan_count+=1
    def event(self,name,**fields):
        self.trace.write(json.dumps({'event':name,'time':self.now(),**fields})+'\n');self.trace.flush()
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def command(self,m):
        self.commands.append([self.now(),m.linear.x,m.linear.y])
        self.last_cmd=math.hypot(m.linear.x,m.linear.y);self.peak_cmd=max(self.peak_cmd,self.last_cmd)
    def box_pose(self,m):
        x=m.pose.pose.position;self.box=np.array([x.x,x.y]);self.box_history.append([stamp(m.header.stamp),x.x,x.y])
    def touch(self,m):self.contacts+=len(m.contacts)
    def pose(self,m):
        pos=m.pose.pose.position;q=m.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        x=np.array([pos.x,pos.y]);t=stamp(m.header.stamp)
        self.odom.append((t,x,yaw))
        tr=tf('odom','base_link',m.header.stamp);tr.transform.translation.x=pos.x;tr.transform.translation.y=pos.y;tr.transform.rotation=q
        self.broadcaster.sendTransform(tr)
        self.measured_speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)
        self.peak=max(self.peak,self.measured_speed)
        boxes=[self.box] if case['moving'] else [np.array(b)-case['start'] for b in case['boxes']]
        for box in boxes:
            if box is not None:
                delta=np.abs(x-box)-np.array([.58,.515])
                gap=float(np.linalg.norm(np.maximum(delta,0))) if np.any(delta>0) else float(np.max(delta))
                self.gap=min(self.gap,gap)
        if t-self.dt_last>.19:
            self.dt_last=t; self.samples.append([t,*x,self.measured_speed,self.last_cmd])
    def cloud(self,key,m):
        if not self.odom: return
        when=stamp(m.header.stamp);t,x,yaw=min(self.odom,key=lambda o:abs(o[0]-when))
        if abs(t-when)>.18:return
        dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[next(f.offset for f in m.fields if f.name==k) for k in ('x','y','z')],'itemsize':m.point_step})
        points=np.frombuffer(m.data,dtype=dt)[::4]
        xyz=np.stack([points[k] for k in ('x','y','z')],axis=1)
        xyz=xyz[np.isfinite(xyz).all(axis=1)]
        pose=case['poses'][key];body=xyz@rot(*pose[3:]).T+pose[:3]
        body=body[(body[:,2]>.13)&(body[:,2]<1.8)&(np.max(np.abs(body[:,:2]),axis=1)<10)]
        body=body[(np.abs(body[:,0])>.32)|(np.abs(body[:,1])>.28)]
        xy=body[:,:2]@rot(0,0,yaw)[:2,:2].T+x
        idx=np.floor((xy-self.origin)/self.res).astype(int)
        self.count[key]+=1;self.returns[key]+=len(xy)
        if len(xy):
            _,ids=np.unique(idx,axis=0,return_index=True);xy=xy[ids];body=body[ids]
        arr=np.ones((len(xy),4),dtype='<f4');arr[:,:2]=xy;arr[:,2]=body[:,2]
        cloud=PointCloud2();cloud.header.frame_id='map';cloud.header.stamp=m.header.stamp;cloud.height=1;cloud.width=len(arr)
        cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z','intensity'))]
        cloud.point_step=16;cloud.row_step=16*len(arr);cloud.is_dense=True;cloud.data=arr.tobytes();self.cloud_pub[key].publish(cloud)
    def publish_map(self):
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.info.height,m.info.width=self.base.shape;m.info.resolution=self.res;m.info.origin.position.x=float(self.origin[0]);m.info.origin.position.y=float(self.origin[1]);m.info.origin.orientation.w=1.
        from array import array
        if not self.map_sent:
            m.data=array('b',self.base.tobytes());self.empty_pub.publish(m);self.map_sent=True
    def spin_until(self,test,limit):
        end=time.monotonic()+limit
        while not test():
            if time.monotonic()>=end:raise TimeoutError('timeout '+str(test))
            rclpy.spin_once(self,timeout_sec=.03)
    def result(self,f,timeout=5):
        self.spin_until(f.done,timeout)
        return f.result()
    def active(self,name):
        cli=self.create_client(GetState,name+'/get_state')
        until=time.monotonic()+45
        while time.monotonic()<until:
            if cli.wait_for_service(timeout_sec=.1):
                ans=self.result(cli.call_async(GetState.Request()))
                if ans.current_state.label=='active':return
            rclpy.spin_once(self,timeout_sec=.1)
        raise RuntimeError(name+' lifecycle activation failed')
    def run(self):
        self.spin_until(lambda: self.odom and all(self.count.values()) and (not case['moving'] or self.box is not None),45)
        for name in ['planner_server','controller_server','velocity_smoother','behavior_server','bt_navigator']:self.active(name)
        ready_time=self.now();self.spin_until(lambda:self.now()-ready_time>=2.,12)
        self.event('ready',clouds=dict(self.count))
        target=PoseStamped();target.header.frame_id='map';target.pose.orientation.w=1.;target.pose.position.x=float(self.goal[0]);target.pose.position.y=float(self.goal[1])
        msg=NavigateToPose.Goal();msg.pose=target;msg.pose.header.stamp=self.get_clock().now().to_msg()
        self.spin_until(lambda:self.navigator.server_is_ready(),6)
        handle=self.result(self.navigator.send_goal_async(msg))
        if not handle.accepted:raise RuntimeError('NavigateToPose rejected')
        feedback=handle.get_result_async();start=self.now();stopped=None
        if case['moving']:self.moving_started=start;self.event('box_motion_start')
        while self.now()-start < 100:
            rclpy.spin_once(self,timeout_sec=.03)
            now=self.now()
            if self.contacts or self.gap<=.05:raise RuntimeError('contact or clearance <= 0.05 m')
            if case['moving']:
                cmd=Twist()
                if now-start<case['trajectory']['duration_s']:cmd.linear.x=-case['trajectory']['speed']
                self.box_cmd.publish(cmd)
            if feedback.done():
                status=feedback.result().status
                if status!=GoalStatus.STATUS_SUCCEEDED:raise RuntimeError('NavigateToPose status '+str(status))
                if stopped is None:stopped=now;self.event('goal_response',status=status)
                if now-stopped>=1.8 and self.measured_speed<.03 and self.last_cmd<.03:return
        raise TimeoutError('did not reach goal')
rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
trial=Trial();error=None
try:trial.run()
except Exception as exc:error=repr(exc)
finally:
    final=trial.odom[-1][1] if trial.odom else np.array([math.nan,math.nan])
    final_error=float(np.linalg.norm(final-trial.goal))
    stopped_seconds=0.
    if trial.samples:
        for sample in reversed(trial.samples):
            if sample[3]>=.03 or sample[4]>=.03:break
            stopped_seconds=trial.samples[-1][0]-sample[0]
    box_travel=float(np.linalg.norm(np.array(trial.box_history[-1][1:])-np.array(trial.box_history[0][1:]))) if trial.box_history else 0.
    box_speed=None
    if case['moving'] and trial.moving_started is not None:
        window=np.asarray([s for s in trial.box_history if trial.moving_started+.2<s[0]<trial.moving_started+case['trajectory']['duration_s']-.2])
        if len(window)>10:
            box_speed=float(np.linalg.norm(np.polyfit(window[:,0]-window[0,0],window[:,1:],1)[0]))
    passed=error is None and final_error<=.12 and stopped_seconds>=1. and trial.contacts==0 and trial.gap>.05 and all(trial.count.values()) and trial.peak>.1 and trial.peak_cmd<=2.500001 and trial.peak<=2.515 and (not case['moving'] or (box_travel>=3.4 and box_speed is not None and abs(box_speed-.5)<.05))
    report=dict(passed=bool(passed),error=error,case=case['name'],distance_to_goal=final_error,stop_seconds=stopped_seconds,odom_peak=trial.peak,cmd_peak=trial.peak_cmd,contacts=trial.contacts,gap=trial.gap,cloud_frames=trial.count,obstacle_returns=trial.returns,plans=trial.plan_count,box_travel=box_travel,box_speed=box_speed)
    (out/'commands.json').write_text(json.dumps(trial.commands))
    (out/'result.json').write_text(json.dumps(report,indent=2));(out/'trajectory.json').write_text(json.dumps(trial.samples));print(report,flush=True)
    trial.destroy_node();rclpy.shutdown()
sys.exit(0 if passed else 1)
