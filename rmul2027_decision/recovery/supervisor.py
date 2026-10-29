#!/usr/bin/env python3
"""Own exactly one navigation launch tree. Never discover a PID to stop.
Keep simulator alive while replacing the navigation graph (LIO, relocalizer,
terrain, costmaps and controllers) under independent fail-closed stop/permit.
"""
import argparse,concurrent.futures,json,math,os,signal,subprocess,time
from collections import deque
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu,PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from example_interfaces.msg import Bool
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer,TransformListener
from scipy.spatial.transform import Rotation
from core import Recovery,Limits
from stationary import Stillness

class Supervisor(Node):
    def __init__(self,command,params,record):
        super().__init__('imu_recovery_supervisor',namespace='/red_standard_robot1')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.command=command;self.params=params;self.record=record;record.mkdir(exist_ok=False)
        configured=yaml.safe_load(params.read_text())
        root=configured.get('/red_standard_robot1',configured.get('red_standard_robot1',configured))
        lio=root['point_lio']['ros__parameters']
        self.imu_topic=lio.get('common.imu_topic',lio.get('common',{}).get('imu_topic','livox/imu'))
        if not isinstance(self.imu_topic,str) or not self.imu_topic:raise ValueError('IMU input topic required')
        self.file=(record/'events.jsonl').open('w',buffering=1)
        self.child=None;self.streams=[];self.owned=[];self.closing=False
        self.pool=concurrent.futures.ThreadPoolExecutor(max_workers=2)
        self.future=None;self.scan_future=None;self.cloud=None;self.last_scan=0.
        self.still=Stillness();self.still_at=-1e20;self.imu=deque();self.imu_at=-1e20;self.imu_stamp=0.;self.imu_since=None
        self.odom_at=-1e20;self.odom_stamp=0.;self.odom_progress=-1e20
        self.command_at=-1e20;self.command_value=[math.inf]*3
        self.status_at=-1e20;self.decision={};self.goals=[];self.goals_at=-1e20
        self.seed=None;self.saved_seed=None;self.registration_seed=None;self.saved_registration_seed=None;self.pose_history=deque()
        self.seed_acked=False;self.seed_requests=set();self.seed_request_at=0.
        self.core=Recovery(time.monotonic());self.core.since+=60. # Startup allowance incl. map loading, bounded.
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.lifecycle={};self.lifecycle_pending={};self.lifecycle_query=0.
        self.lifecycle_clients={name:self.create_client(GetState,name+"/get_state") for name in ("bt_navigator","controller_server")}
        self.seed_pub=self.create_publisher(PoseWithCovarianceStamped,'initialpose',10)
        self.stop_pub=self.create_publisher(Bool,'recovery/stop',1)
        self.permit_pub=self.create_publisher(Bool,'recovery/permit',1)
        self.status_pub=self.create_publisher(String,'recovery/status',10)
        self.subs=[
            self.create_subscription(Imu,self.imu_topic,self.onimu,qos_profile_sensor_data),
            self.create_subscription(PointCloud2,'velodyne_points',lambda m:setattr(self,'cloud',m),qos_profile_sensor_data),
            self.create_subscription(Odometry,'odometry',self.onodom,qos_profile_sensor_data),
            self.create_subscription(Twist,'cmd_vel',self.oncommand,10),
            self.create_subscription(String,'decision/status',self.ondecision,10),
            self.create_subscription(String,'recovery/registration',self.onquality,10),
            self.create_subscription(GoalStatusArray,'navigate_to_pose/_action/status',self.ongoals,10)]
        self.write('created',command=command,params=str(params),limits=vars(self.core.limits),imu_topic=self.imu_topic)
        self.start_navigation(0,None)
        self.timer=self.create_timer(.02,self.tick,clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
    def ros(self):return self.get_clock().now().nanoseconds*1e-9
    def write(self,kind,**kw):
        self.file.write(json.dumps(dict(kind=kind,steady=time.monotonic(),ros=self.ros(),epoch=self.core.epoch,**kw),allow_nan=False)+'\n')
    def start_navigation(self,epoch,seed):
        config=yaml.safe_load(self.params.read_text())
        # run_simulation already wraps the file in namespace; support both.
        root=config.get('/red_standard_robot1',config.get('red_standard_robot1',config))
        assert root['fake_vel_transform']['ros__parameters']['recovery_stop_timeout']==.3
        root['small_gicp_relocalization']['ros__parameters']['recovery_epoch']=epoch
        if seed is not None:root['small_gicp_relocalization']['ros__parameters']['init_pose']=seed
        params=self.record/f'nav-epoch-{epoch}.yaml'
        params.write_text(yaml.safe_dump(config,sort_keys=False))
        command=[('params_file:='+str(params)) if s.startswith('params_file:=') else s for s in self.command]
        stream=(self.record/f'nav-epoch-{epoch}.log').open('w');self.streams.append(stream)
        self.child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.owned.append(self.child)
        self.write('navigation_started',pid=self.child.pid,command=command,seed=seed)
    def stop_navigation(self):
        if self.child is None:return True
        p=self.child
        # Snapshot and stop only this Popen's verified launch tree. Never pkill.
        helper=Path(os.environ.get('RMUL_WORKSPACE','/home/hardy/pb2025_nav_ws'))/'src/pb2025_sentry_nav/scripts/venom/stop_launches.py'
        tracked={}
        def capture(pid):
            try:
                stat=Path('/proc',str(pid),'stat').read_text().rsplit(') ',1)[1].split()
                tracked[pid]=stat[19]
                for child in Path('/proc',str(pid),'task',str(pid),'children').read_text().split():capture(int(child))
            except (OSError,IndexError):pass
        capture(p.pid)
        result=subprocess.run(['python3',str(helper),str(p.pid)],timeout=23,capture_output=True,text=True)
        p.wait(timeout=3)
        self.write('navigation_stopped',pid=p.pid,returncode=p.returncode,helper_returncode=result.returncode)
        remaining=[]
        for pid,start in tracked.items():
            try:
                stat=Path('/proc',str(pid),'stat').read_text().rsplit(') ',1)[1].split()
                if stat[0]!='Z' and stat[19]==start:remaining.append(pid)
            except OSError:pass
        self.write('owned_tree_audit',remaining=remaining)
        return result.returncode==0 and not remaining
    def restart(self):
        try:
            if not self.stop_navigation() or self.closing:return False
            self.start_navigation(self.core.epoch,self.saved_registration_seed);return True
        except Exception as e:self.write('restart_exception',error=str(e));return False
    def onimu(self,m):
        t=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        values=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,
                m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]
        valid=all(math.isfinite(v) for v in values) and -.1<=self.ros()-stamp<=.1
        valid=valid and (not self.imu_stamp or 0<stamp-self.imu_stamp<=.1)
        if not valid:self.imu_since=None;self.imu.clear()
        elif self.imu_since is None:self.imu_since=t
        self.imu_stamp=stamp;self.imu_at=t;self.imu.append((t,values,valid))
        while self.imu and t-self.imu[0][0]>2.5:self.imu.popleft()
    def onodom(self,m):
        t=time.monotonic();s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        p=m.pose.pose.position;q=m.pose.pose.orientation
        if not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]):return
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01:return
        self.odom_at=t
        if s>self.odom_stamp:self.odom_stamp=s;self.odom_progress=t
    def oncommand(self,m):self.command_at=time.monotonic();self.command_value=[m.linear.x,m.linear.y,m.angular.z]
    def ondecision(self,m):
        try:self.decision=json.loads(m.data);self.status_at=time.monotonic()
        except ValueError:self.decision={}
    def ongoals(self,m):self.goals=[s.status for s in m.status_list];self.goals_at=time.monotonic()
    def onquality(self,m):
        try:
            q=json.loads(m.data)
            if q.get('kind')=='seed_ack':
                if q['epoch']==self.core.epoch and any(abs(q['stamp']-s)<1e-6 for s in self.seed_requests):
                    self.seed_acked=True;self.pose_history.clear()
                self.write('seed_ack',data=q,matched=self.seed_acked)
                return
            accepted=self.core.registration(q,time.monotonic(),self.ros())
            self.write('registration',accepted_for_recovery=accepted,data=q)
        except (ValueError,TypeError):self.core.quality=[]
    def raw_healthy(self,t):
        return (self.imu and self.imu[-1][2] and t-self.imu_at<=.15
                and -.1<=self.ros()-self.imu_stamp<=.1 and self.imu_since is not None
                and t-self.imu_since>=2.)
    def independent_stationary(self,t):
        if not self.raw_healthy(t) or t-self.still_at>.4:return False
        samples=np.array([v for s,v,valid in self.imu if valid and t-s<=2.])
        return (len(samples)>=100 and np.max(np.linalg.norm(samples[:,:3],axis=1))<.03
                and np.max(np.abs(np.linalg.norm(samples[:,3:],axis=1)-9.81))<.2
                and np.max(np.std(samples[:,3:],axis=0))<.1
                and t-self.command_at<.3 and max(abs(v) for v in self.command_value)<.001)
    def observe_cloud(self,m,t):
        try:
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if not -.1<=self.ros()-stamp<=.3:raise ValueError('raw scan source stale/future')
            points=np.array(list(point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True)))
            if points.dtype.names:points=np.column_stack([points[n] for n in ('x','y','z')])
            ok=self.still.observe(points,t)
            return ok,t,self.still.report
        except Exception as e:self.still.reset();return False,t,dict(error=str(e))
    def pose_good(self,t):
        self.pose_report={}
        try:
            tf=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            self.pose_report.update(stamp=stamp,age=self.ros()-stamp)
            if not -.1<=self.ros()-stamp<=.5 or stamp<=self.core.restart_stamp:
                self.pose_report['blocked']='source_time';return False
            p=tf.transform.translation;q=tf.transform.rotation
            quat=[q.x,q.y,q.z,q.w];xyz=[p.x,p.y,p.z]
            if not all(math.isfinite(v) for v in xyz+quat) or abs(sum(v*v for v in quat)-1)>.01:
                self.pose_report['blocked']='invalid_pose';return False
            angles=Rotation.from_quat(quat).as_euler('xyz');current=xyz+angles.tolist()
            self.pose_report['map_to_base']=current
            if self.core.state=='RUNNING':
                self.seed=current
                gimbal=self.buffer.lookup_transform('map','gimbal_yaw',rclpy.time.Time())
                gp=gimbal.transform.translation;gq=gimbal.transform.rotation
                self.registration_seed=[gp.x,gp.y,gp.z]+Rotation.from_quat([gq.x,gq.y,gq.z,gq.w]).as_euler('xyz').tolist()
            self.pose_history.append((t,stamp,np.array(current)))
            while self.pose_history and t-self.pose_history[0][0]>2.:self.pose_history.popleft()
            if not self.pose_history or t-self.pose_history[0][0]<1.5:
                self.pose_report['blocked']='insufficient_history';return False
            a=np.array([v for _,_,v in self.pose_history])
            angles_delta=np.arctan2(np.sin(a[:,5]-a[-1,5]),np.cos(a[:,5]-a[-1,5]))
            spread=float(np.max(np.linalg.norm(a[:,:3]-a[-1,:3],axis=1)))
            yaw_spread=float(max(abs(angles_delta)))
            self.pose_report.update(position_spread=spread,yaw_spread=yaw_spread)
            if spread>.03 or yaw_spread>.02:
                self.pose_report['blocked']='pose_unstable';return False
            if self.saved_registration_seed is not None:
                # Compare the SAME stable frame accepted by the relocalizer.
                # Chassis yaw legitimately changes during the 3.1rad/s spin stop.
                gimbal=self.buffer.lookup_transform('map','gimbal_yaw',rclpy.time.Time())
                gs=gimbal.header.stamp.sec+gimbal.header.stamp.nanosec*1e-9
                if gs<=self.core.restart_stamp or not -.1<=self.ros()-gs<=.5:
                    self.pose_report['blocked']='gimbal_source_time';return False
                gp=gimbal.transform.translation;gq=gimbal.transform.rotation
                gpose=[gp.x,gp.y,gp.z]+Rotation.from_quat([gq.x,gq.y,gq.z,gq.w]).as_euler('xyz').tolist()
                if not all(math.isfinite(v) for v in gpose):
                    self.pose_report['blocked']='invalid_gimbal_pose';return False
                distance=float(np.linalg.norm(np.array(gpose[:2])-np.array(self.saved_registration_seed[:2])))
                dy=gpose[5]-self.saved_registration_seed[5];dy=abs(math.atan2(math.sin(dy),math.cos(dy)))
                self.pose_report.update(seed_frame='gimbal_yaw',map_to_gimbal=gpose,
                    seed_distance=distance,seed_yaw_delta=dy,saved_map_to_gimbal=self.saved_registration_seed)
                if distance>.35 or dy>.15:
                    self.pose_report['blocked']='seed_deviation';return False
            self.pose_report['blocked']='';return True
        except Exception as e:self.pose_report['blocked']=str(e);return False
    def request_seed(self,t):
        if self.core.epoch==0:return True
        if self.seed_acked:return True
        if self.core.state!='VERIFY' or t-self.seed_request_at<.5:return False
        try:
            tf=self.buffer.lookup_transform('odom','gimbal_yaw',rclpy.time.Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            if stamp<=self.core.restart_stamp or self.odom_stamp<=self.core.restart_stamp:return False
            if not -.1<=self.ros()-stamp<=.5:return False
            if self.count_subscribers('initialpose')<1 or self.count_publishers('initialpose')!=1:return False
            pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
            seed=self.saved_registration_seed
            pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z=seed[:3]
            q=Rotation.from_euler('xyz',seed[3:]).as_quat()
            pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w=q
            source=pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9
            self.seed_requests.add(source);self.seed_request_at=t;self.seed_pub.publish(pose)
            self.core.quality=[]
            self.write('seed_requested',frame='gimbal_yaw',stamp=source,pose=seed)
        except Exception as e:self.write('seed_request_wait',error=str(e))
        return False
    def lifecycle_check(self,t):
        if self.core.state=='VERIFY' and t-self.lifecycle_query>.5:
            self.lifecycle_query=t
            for name,client in list(self.lifecycle_clients.items()):
                pending=self.lifecycle_pending.get(name)
                if pending:
                    future,sent,epoch=pending
                    if t-sent<=.8:continue
                    client.remove_pending_request(future)
                    self.lifecycle_pending.pop(name,None)
                    self.destroy_client(client)
                    client=self.create_client(GetState,name+"/get_state")
                    self.lifecycle_clients[name]=client
                    self.write('lifecycle_query_timeout',node=name,request_epoch=epoch)
                if not client.service_is_ready():continue
                epoch=self.core.epoch;future=client.call_async(GetState.Request())
                self.lifecycle_pending[name]=(future,t,epoch)
                def done(f,name=name,epoch=epoch):
                    pending=self.lifecycle_pending.get(name)
                    if pending is None or pending[0] is not f:return
                    self.lifecycle_pending.pop(name,None)
                    try:
                        if epoch==self.core.epoch:
                            self.lifecycle[name]=(time.monotonic(),f.result().current_state.id)
                    except Exception:self.lifecycle.pop(name,None)
                future.add_done_callback(done)
        return all(name in self.lifecycle and t-self.lifecycle[name][0]<1.5 and self.lifecycle[name][1]==3
                   for name in self.lifecycle_clients)
    def tick(self):
        t=time.monotonic()
        if self.closing:return
        if self.child and self.child.poll() is not None and self.core.state!='RESTARTING':self.core.fail('owned_navigation_exited',t)
        if self.scan_future and self.scan_future.done():
            ok,observed,report=self.scan_future.result();self.scan_future=None
            if ok:self.still_at=observed
            else:self.still_at=-1e20
            if self.core.state!='RUNNING':self.write('stationarity',stationary=ok,report=report)
        if self.core.state in ('STATIC','VERIFY') and self.scan_future is None and self.cloud is not None and t-self.last_scan>=.2:
            self.last_scan=t;self.scan_future=self.pool.submit(self.observe_cloud,self.cloud,t)
        d=self.decision
        fresh=t-self.status_at<.3
        external=(fresh and not d.get('fault') and d.get('recovery_permit_timeout')==.3
                  and d.get('reason') in ('recovery_hold_or_missing_heartbeat','odometry_invalid_or_stalled',
                      'odometry_source_age','tf_stale_or_stalled','tf_missing',''))
        terminal=(fresh and d.get('inhibited') is True and d.get('goal')=='' and not d.get('fault')
                  and not any(1<=s<=3 for s in self.goals))
        healthy=(self.raw_healthy(t) and t-self.odom_at<=.5 and t-self.odom_progress<=.5
                 and -.1<=self.ros()-self.odom_stamp<=.5)
        navigation_active=self.lifecycle_check(t)
        seed_confirmed=self.request_seed(t)
        pose=self.pose_good(t) and navigation_active and seed_confirmed
        before=self.core.state
        if self.future and self.future.done():
            success=self.future.result();self.future=None
            self.core.restarted(t,self.ros(),success)
            self.pose_history.clear();self.still.reset();self.still_at=-1e20;self.lifecycle.clear();self.seed_acked=False;self.seed_requests.clear()
            for client in self.lifecycle_clients.values():self.destroy_client(client)
            self.lifecycle_clients={name:self.create_client(GetState,name+"/get_state") for name in ("bt_navigator","controller_server")}
            self.lifecycle_pending.clear()
        stationary=self.independent_stationary(t)
        effect=self.core.tick(t,healthy=healthy,terminal=terminal,
            stationary=stationary,external=external,pose=pose)
        if before=='RUNNING' and self.core.state!='RUNNING':
            self.saved_seed=self.seed;self.saved_registration_seed=self.registration_seed
            self.still.reset();self.still_at=-1e20
            if self.saved_seed is None or self.saved_registration_seed is None:self.core.fail('no_pre_fault_estimated_pose_seed',t)
        if effect=='restart':
            self.write('restart_authorized',terminal=terminal,stationary=self.independent_stationary(t),seed=self.saved_seed)
            self.pose_history.clear()
            self.future=self.pool.submit(self.restart)
        stop=Bool();stop.data=not self.core.permit;self.stop_pub.publish(stop)
        permit=Bool();permit.data=self.core.permit;self.permit_pub.publish(permit)
        if before!=self.core.state:self.write('transition',previous=before,state=self.core.state,fault=self.core.fault)
        if t-getattr(self,'last_status',0)>.1:
            self.last_status=t
            status=dict(state=self.core.state,epoch=self.core.epoch,attempts=self.core.attempts,
                permit=self.core.permit,fault=self.core.fault,trigger=self.core.trigger,
                steady=t,ros=self.ros(),quality_count=len(self.core.quality),
                gates=dict(healthy=bool(healthy),terminal=bool(terminal),stationary=bool(stationary),external=bool(external),pose=bool(pose),navigation_active=bool(navigation_active)),pose_detail=getattr(self,'pose_report',{}))
            self.status_pub.publish(String(data=json.dumps(status)))
            self.write('gate_status',status=status)
    def close(self):
        self.closing=True
        self.stop_pub.publish(Bool(data=True));self.permit_pub.publish(Bool(data=False))
        if self.future:self.future.result(timeout=65)
        self.stop_navigation()
        self.pool.shutdown(wait=True)
        self.write('shutdown',state=self.core.state,fault=self.core.fault)
        for s in self.streams:s.close()
        self.file.close()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--command',required=True);parser.add_argument('--params',type=Path,required=True)
    parser.add_argument('--record-dir',type=Path,required=True)
    args=parser.parse_args()
    rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'],signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    n=None;stop=False
    def request_stop(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    try:
        n=Supervisor(json.loads(args.command),args.params,args.record_dir)
        while rclpy.ok() and not stop:rclpy.spin_once(n,timeout_sec=.01)
    finally:
        if n:n.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
