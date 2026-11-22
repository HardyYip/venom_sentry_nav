"""C075 opt-in large-Yaw observation, using estimated TF and real clouds only."""
import math,time,json,collections
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.time import Time
from rclpy.clock import Clock,ClockType
from tf2_ros import Buffer,TransformListener
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import PointCloud2,JointState,Imu
from example_interfaces.msg import Bool
from rmoss_interfaces.msg import Gimbal
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String
from mid_odin_view_policy import ViewPolicy,StableHeading,wrap
from mid_odin_view_geometry import ground_probes_visible as odin_probes_visible,mid_probes_visible
from mid_odin_view_coverage import MeasuredCoverage,MidMeasuredCoverage
from mid_odin_view_path import path_heading
from mid_odin_view_gaze import common_yaw_solution
from mid_geometric_obstacle_cloud import SourcePairHealth


def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

class Observer(Node):
    def __init__(self):
        super().__init__('mid_odin_view',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.tf=Buffer();self.listener=TransformListener(self.tf,self);self.policy=ViewPolicy();self.active=False;self.active_id=None;self.intent=None;self.command=None;self.gimbal=None;self.goal_count=0;self.last_state=None;self.target_command=None;self.last_tick=time.monotonic();self.last_rotation=None;self.last_rotation_wall=None;self.external_stop=False;self.recovery_stop=False;self.last_ros=-1.;self.last_ros_progress=time.monotonic();self.pending=collections.deque(maxlen=20);self.cloud_errors=collections.Counter();self.coverage=MeasuredCoverage();self.mid_coverage=MidMeasuredCoverage();self.selected_sensor="odin";self.primary_imu=None;self.holding_yaw=None;self.holding_target=None;self.last_diagnostic=0.;self.steering=StableHeading();self.direction_wait_started=None;self.planned_path=None;self.request_steering=StableHeading();self.last_nonzero_request=None;self.look_history=[];self.gaze_tag=None;self.gaze_axis_target=None;self.paired_gaze=False
        self.aim_correction = 0.
        self.aim_still_since = None
        self.geometry_guard_required=self.declare_parameter('mid_geometric_guard',False).value
        self.geometry_health=SourcePairHealth()
        if self.count_publishers('cmd_gimbal_joint') or self.count_publishers('robot_base/gimbal_cmd'):raise RuntimeError('existing gimbal intent owner; refuse competing control')
        self.stop=self.create_publisher(Bool,'mid_odin_view/stop',1);self.gimbal_pub=self.create_publisher(JointState,'cmd_gimbal_joint',1);self.status=self.create_publisher(String,'mid_odin_view/status',10)
        self.stop_sub=self.create_subscription(Bool,'safety_stop',self.emergency,10)
        self.recovery_sub=self.create_subscription(Bool,'recovery/stop',self.recovery,10)
        sensor=QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.path_sub=self.create_subscription(NavPath,'plan',self.plan,2)
        self.imu_sub=self.create_subscription(Imu,'livox/imu',self.imu,sensor)
        self.subs=[self.create_subscription(Twist,'cmd_vel_nav2_result',self.cmd,10),self.create_subscription(PoseStamped,'mid_odin_view/intent',self.request,10),self.create_subscription(Gimbal,'robot_base/gimbal_state',self.state,10),self.create_subscription(GoalStatusArray,'navigate_to_pose/_action/status',self.goals,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        for kind,topic in [('raw','odin/points'),('converted','odin/obstacles'),('mid_raw','livox/lidar'),('mid_converted','terrain_map')]:self.subs.append(self.create_subscription(PointCloud2,topic,lambda m,k=kind:self.cloud(k,m),sensor))
        if self.geometry_guard_required:self.geometry_sub=self.create_subscription(PointCloud2,'mid/geometric_obstacles',self.geometry_cloud,sensor)
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME));self.stop.publish(Bool(data=True))
    def emergency(self,m):
        self.external_stop=m.data
        if m.data:self.policy.state='failed';self.stop.publish(Bool(data=True))
    def recovery(self,m):
        self.recovery_stop=m.data
        if m.data:self.look_history=[];self.gaze_tag=None;self.policy.reset();self.target_command=None;self.holding_yaw=None;self.holding_target=None;self.stop.publish(Bool(data=True))
    def imu(self,m):
        q=m.angular_velocity;value=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z)
        if math.isfinite(value):self.primary_imu=(time.monotonic(),stamp(m),value)
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def cmd(self,m):self.command=(time.monotonic(),m)
    def request(self,m):
        if not math.isfinite(m.pose.position.x) or not math.isfinite(m.pose.position.y):return
        self.intent=(time.monotonic(),m);self.look_history=[];self.gaze_tag=None;self.policy.reset();self.holding_yaw=None;self.holding_target=None;self.steering.reset();self.request_steering.reset();self.direction_wait_started=None
    def state(self,m):
        if math.isfinite(m.yaw):self.gimbal=(time.monotonic(),float(m.yaw))
    def goals(self,m):
        ids=[bytes(x.goal_info.goal_id.uuid).hex() for x in m.status_list if x.status in [1,2]];self.goal_count=len(ids)
        new=ids[0] if len(ids)==1 else None
        if self.active_id is not None and new!=self.active_id:self.policy.reset()
        if new!=self.active_id:self.look_history=[];self.gaze_tag=None;self.steering.reset();self.request_steering.reset();self.direction_wait_started=None;self.holding_yaw=None;self.holding_target=None
        self.active_id=new;self.active=new is not None
        if not self.active:self.stop.publish(Bool(data=True))
    def transform(self,target,source,s=None):
        t=self.tf.lookup_transform(target,source,Time() if s is None else Time(seconds=s));r=rotation(t.transform.rotation);v=t.transform.translation;return r,np.array([v.x,v.y,v.z]),stamp(t)
    def plan(self,m):
        points=np.array([[v.pose.position.x,v.pose.position.y] for v in m.poses])
        if points.ndim==2 and len(points) and np.isfinite(points).all():self.planned_path=(time.monotonic(),stamp(m),m.header.frame_id,points)
    def look_heading(self,wall,robot,fallback):
        if self.active and self.planned_path is not None:
            received,s,frame,points=self.planned_path
            if wall-received<=1. and -.1<=self.now()-s<=1.:
                if frame!='odom':
                    r,v,_=self.transform('odom',frame);points=(np.column_stack([points,np.zeros(len(points))])@r.T+v)[:,:2]
                heading=path_heading(points,robot[:2])
                if heading is not None:return heading,'path'
        return fallback,'request_or_intent'
    def heading(self,wall,robot):
        if self.active and self.command and wall-self.command[0]<.3:
            cmd=self.command[1]
            if math.hypot(cmd.linear.x,cmd.linear.y)>1e-4:
                r,_,s=self.transform('odom','gimbal_yaw_fake')
                if not -.1<=self.now()-s<=.3:raise ValueError('stale command-frame TF')
                direction=r@np.array([cmd.linear.x,cmd.linear.y,0.]);return math.atan2(direction[1],direction[0])
        if self.intent and wall-self.intent[0]<10:
            m=self.intent[1];point=np.array([m.pose.position.x,m.pose.position.y,m.pose.position.z]);frame=m.header.frame_id
            if frame!='odom':
                r,v,s=self.transform('odom',frame);point=r@point+v
            delta=point[:2]-robot[:2]
            if np.linalg.norm(delta)>.05:return math.atan2(delta[1],delta[0])
        return self.policy.target
    def geometry_cloud(self,m):
        fields={f.name:f for f in m.fields}
        valid=(m.header.frame_id=='front_mid360_scan' and m.height==1 and m.point_step==16 and m.row_step==16*m.width and len(m.data)==m.row_step
               and all(k in fields and fields[k].datatype==7 and fields[k].count==1 and fields[k].offset==4*i for i,k in enumerate(('x','y','z','intensity'))))
        if valid:self.geometry_health.add('converted',stamp(m),time.monotonic())
    def cloud(self,kind,m):
        if self.geometry_guard_required and kind=='mid_raw' and m.header.frame_id=='front_mid360_scan':self.geometry_health.add('raw',stamp(m),time.monotonic())
        if self.policy.target is None or self.policy.state not in ['aiming','observing','ready']:return
        self.pending.append((time.monotonic(),kind,m))
    def consume_clouds(self):
        remaining=collections.deque(maxlen=20)
        while self.pending:
            received,kind,m=self.pending.popleft();s=stamp(m);ros=self.now()
            if not -.1<=ros-s<=.25:continue
            try:
                fields={f.name:f for f in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['>f4' if m.is_bigendian else '<f4']*3,'offsets':[fields[n].offset for n in ['x','y','z']],'itemsize':m.point_step})
                a=np.frombuffer(m.data,dtype=dt,count=m.width*m.height);points=np.column_stack([a[n] for n in ['x','y','z']]);r,v,_=self.transform('odom',m.header.frame_id,s);points=points@r.T+v;_,robot,_=self.transform('odom','base_footprint',s)
                if kind.startswith('mid_'):self.mid_coverage.add(kind[4:],s,points-robot)
                else:self.coverage.add(kind,s,points-robot)
            except Exception as e:
                if time.monotonic()-received<.25:remaining.append((received,kind,m))
                else:
                    self.cloud_errors[kind+':'+type(e).__name__]+=1
                    if self.cloud_errors[kind+':'+type(e).__name__]<=3:self.get_logger().warn('cloud TF/format unavailable: '+kind+' '+str(e))
        self.pending=remaining
    def correct_stalled_aim(self, target, error, rate, gyro_fresh, wall, dt):
        # The Gazebo position loop can hold a repeatable error while the
        # measured gimbal is stationary. Keep the same slew and scan checks.
        if (self.policy.state == 'aiming' and gyro_fresh and self.primary_imu[2] <= .08
                and rate <= .08 and abs(error) > .10):
            if self.aim_still_since is None:
                self.aim_still_since = wall
            elif wall - self.aim_still_since >= .30:
                step = math.copysign(.35 * dt, error)
                self.aim_correction = float(np.clip(self.aim_correction + step, -.40, .40))
        else:
            self.aim_still_since = None
        return target + self.aim_correction

    def tick(self):
        wall=time.monotonic();ros=self.now();allowed=False;reason='waiting';error=None;fov=None
        self.consume_clouds()
        if ros>self.last_ros:self.last_ros_progress=wall;self.last_ros=ros
        try:
            if self.count_publishers('cmd_gimbal_joint')>1 or self.count_publishers('robot_base/gimbal_cmd')>0:
                self.policy.state='failed';raise ValueError('competing gimbal intent owner; observation locked')
            if self.external_stop or self.recovery_stop:raise ValueError('emergency/recovery stop inhibits gimbal observation')
            if ros<=0 or wall-self.last_ros_progress>.3:raise ValueError('source clock stalled; observation held')
            r,origin,s=self.transform('odom','odin_scan');_,robot,bs=self.transform('odom','base_footprint');healthy=(-.1<=ros-s<=.3 and -.1<=ros-bs<=.3 and self.gimbal is not None and wall-self.gimbal[0]<.4 and self.primary_imu is not None and wall-self.primary_imu[0]<=.25 and -.1<=ros-self.primary_imu[1]<=.25 and self.goal_count<=1)
            if not healthy:raise ValueError('stale TF/gimbal state or multiple goals')
            if self.geometry_guard_required and not self.geometry_health.healthy(ros,wall):raise ValueError('MID geometric output missing, stale or source-unmatched')
            desired=self.heading(wall,robot);look,look_source=self.look_heading(wall,robot,desired)
            fresh_request=self.active and self.command is not None and wall-self.command[0]<.3 and math.hypot(self.command[1].linear.x,self.command[1].linear.y)>1e-4
            if fresh_request:
                self.last_nonzero_request=wall
                request_stable=self.request_steering.update(desired,wall)
            else:
                request_stable=False
                # A brief zero command has no direction. It must not let a
                # newly flipped global path seize an established view.
                if self.last_nonzero_request is None or wall-self.last_nonzero_request>.3:self.request_steering.reset()
            if request_stable and desired is not None:
                look,look_source=desired,'stable_velocity_request'
            elif self.active and self.policy.target is not None:
                look,look_source=self.policy.target,'holding_observed_direction'
            self.look_history=[v for v in self.look_history if wall-v[0]<=5.]
            if request_stable and desired is not None:
                # Intents may persist five seconds; measurements retain the
                # original .25-second age and .4-second confirmation gates.
                self.look_history=[v for v in self.look_history if abs(wrap(v[1]-desired))>.20]
                self.look_history.append((wall,desired));self.look_history=self.look_history[-2:]
            direction_stable=self.steering.update(look,wall)
            if request_stable:direction_stable=True  # The same .30s confirmation, not a second serial wait.
            # Hold an acquisition heading while the chassis is inhibited; an
            # unexecuted controller request may oscillate during that stop.
            acquiring=self.policy.target is not None and self.policy.state in ['aiming','observing']
            sensors={'odin':(r,origin,s,odin_probes_visible,self.coverage)}
            try:
                mr,mo,ms=self.transform('odom','front_mid360_scan')
                if -.1<=ros-ms<=.3:sensors['mid']=(mr,mo,ms,mid_probes_visible,self.mid_coverage)
            except Exception:pass
            def measured_sources(heading,barrier):
                if heading is None:return []
                return [key for key,(sr,so,ss,geometry,cache) in sensors.items()
                        if geometry(sr,so,robot,heading)[0]
                        and len(cache.stamps('raw',heading,ros,barrier))>=3
                        and len(cache.stamps('converted',heading,ros,barrier))>=3]
            preconfirmed=(measured_sources(desired,self.policy.observation_ros)
                          if self.policy.state=='ready' and self.policy.observation_ros is not None else [])
            already_measured=bool(preconfirmed) and self.selected_sensor not in preconfirmed
            # An already measured second aperture does not require another
            # gimbal turn or a reset of the shared stationary observation.
            if not already_measured and not acquiring and self.policy.state!='failed' and look is not None:
                visible_sensors=[key for key,(sr,so,ss,geometry,cache) in sensors.items() if geometry(sr,so,robot,look)[0]]
                def turn_distance(key):
                    sr=sensors[key][0];axis=sr[:,0] if key=='odin' else sr[:,2]
                    return abs(wrap(look-math.atan2(axis[1],axis[0])))
                candidate=self.selected_sensor if self.selected_sensor in visible_sensors else min(visible_sensors or list(sensors),key=turn_distance)
                if candidate!=self.selected_sensor:
                    self.selected_sensor=candidate;self.policy.heading(look,wall,force=True)
                    self.holding_yaw=None;self.holding_target=None
            if self.selected_sensor not in sensors:
                raise ValueError('selected sensor TF unavailable; observation held')
            r,origin,s,probes_visible,coverage=sensors[self.selected_sensor]

            if look is not None and not acquiring and not already_measured:
                same_view,_=probes_visible(r,origin,robot,look)
                if same_view and self.policy.target is not None and self.policy.state!='failed':
                    self.policy.target=look;self.policy.raw=[];self.policy.converted=[]
                elif direction_stable:self.policy.heading(look,wall,force=self.policy.state=='ready')
            axis=r[:,0] if self.selected_sensor=="odin" else r[:,2];axis_yaw=math.atan2(axis[1],axis[0])
            tag=(self.selected_sensor,self.policy.target,self.policy.started)
            if self.policy.target is not None and self.policy.state=='aiming' and tag!=self.gaze_tag:
                self.gaze_tag=tag;self.gaze_axis_target=self.policy.target;self.paired_gaze=False
                self.aim_correction = 0.
                self.aim_still_since = None
                alternatives=[h for t,h in self.look_history if abs(wrap(h-self.policy.target))>.20]
                headings=[self.policy.target]+alternatives[-1:]
                if len(headings)>1 and len(sensors)>1:
                    try:
                        gr,pivot,gs=self.transform('odom','gimbal_yaw')
                        # The scan model is yaw-only here. Refuse this planning
                        # shortcut for an appreciably tilted yaw axis.
                        if -.1<=ros-gs<=.3 and gr[2,2]>.999:
                            keys=list(sensors)
                            solution=common_yaw_solution([(sr,so,geometry) for sr,so,ss,geometry,cache in sensors.values()],pivot,robot,headings,keys.index(self.selected_sensor))
                            if solution is not None:
                                delta,primary=solution
                                self.selected_sensor=keys[primary]
                                r,origin,s,probes_visible,coverage=sensors[self.selected_sensor]
                                axis=r[:,0] if self.selected_sensor=="odin" else r[:,2];axis_yaw=math.atan2(axis[1],axis[0])
                                self.gaze_tag=(self.selected_sensor,self.policy.target,self.policy.started)
                                self.gaze_axis_target=wrap(axis_yaw+delta);self.paired_gaze=True
                                self.policy.raw=[];self.policy.converted=[];self.holding_yaw=None;self.holding_target=None
                    except Exception:pass
            aim=self.gaze_axis_target if self.paired_gaze else self.policy.target
            error=wrap(aim-axis_yaw) if aim is not None else None
            rate=0.
            if self.last_rotation is not None and wall-self.last_rotation_wall>.001:rate=abs(wrap(axis_yaw-self.last_rotation))/(wall-self.last_rotation_wall)
            self.last_rotation=axis_yaw;self.last_rotation_wall=wall
            outer_visible,fov=probes_visible(r,origin,robot,self.policy.target) if self.policy.target is not None else (False,None)
            # Odin angular bounds include a one-degree margin inside the simulated FOV.
            # Centering must not add an unreachable second vertical window.
            visible=outer_visible
            gyro_fresh=self.primary_imu is not None and wall-self.primary_imu[0]<=.25 and -.1<=ros-self.primary_imu[1]<=.25
            centered=error is not None and abs(error)<=.10
            aligned=visible and gyro_fresh and (self.policy.state=='ready' or (centered and self.primary_imu[2]<=.08))
            if self.policy.observation_ros is not None and self.policy.target is not None:
                self.policy.raw=coverage.stamps('raw',self.policy.target,ros,self.policy.observation_ros)
                self.policy.converted=coverage.stamps('converted',self.policy.target,ros,self.policy.observation_ros)
            allowed,reason=self.policy.step(wall,ros,aligned,healthy,self.active)
            if allowed and desired is not None:
                barrier=self.policy.observation_ros if self.policy.observation_ros is not None else ros
                available=measured_sources(desired,barrier)
                permit_sensor=(self.selected_sensor if self.selected_sensor in available else available[0] if available else None)
                if not available:
                    allowed=False;reason='latest_requested_direction_not_observed'
                    # This scan completed, but it cannot authorize a different request.
                    if not acquiring and direction_stable and look_source!='holding_observed_direction' and (look_source!='path' or not centered):self.policy.heading(look,wall,force=True)
                    else:reason='latest_requested_direction_waiting_stable_steering'

            # Include steering confirmation in the same bounded preparation budget.
            if desired is not None:
                if reason=='ready':self.direction_wait_started=None
                else:
                    if self.direction_wait_started is None:self.direction_wait_started=wall
                    if wall-self.direction_wait_started>=self.policy.timeout:
                        self.policy.state='failed';allowed=False;reason='observation_timeout'

            # Move only the existing gimbal intent interface; never publish chassis velocity.
            if error is not None and self.policy.state!='failed':
                current=self.gimbal[1]
                if visible and (self.policy.state=='ready' or (centered and gyro_fresh and self.primary_imu[2]<=.08)):
                    if self.holding_yaw is None:
                        # Acquisition keeps the planned aim, not the first feedback
                        # sample inside its tolerance. Preserve command continuity
                        # when entering hold; the same slew limit applies throughout.
                        self.holding_yaw=current if self.policy.state=='ready' else current+error
                        self.holding_target=self.policy.target
                    if self.policy.state=='ready' and not self.paired_gaze and look_source in ['path','stable_velocity_request'] and not already_measured:
                        self.holding_yaw+=wrap(self.policy.target-self.holding_target)
                        self.holding_target=self.policy.target
                    target=self.holding_yaw
                else:self.holding_yaw=None;self.holding_target=None;target=current+error
                if self.target_command is None:self.target_command=current
                dt = min(.1, max(.001, wall - self.last_tick))
                target = self.correct_stalled_aim(target, error, rate, gyro_fresh, wall, dt)
                step = .8 * dt
                self.target_command += float(np.clip(target - self.target_command, -step, step))
                msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=['gimbal_yaw_joint'];msg.position=[self.target_command];self.gimbal_pub.publish(msg)
        except Exception as e:
            allowed,reason=self.policy.step(wall,ros,False,False,self.active);reason=str(e)
        self.last_tick=wall;self.stop.publish(Bool(data=not allowed))
        state=dict(sensor=self.selected_sensor,permit_sensor=permit_sensor if allowed and 'permit_sensor' in locals() else None,paired_gaze=self.paired_gaze,gaze_axis_target=self.gaze_axis_target,state=reason,policy=self.policy.state,permit=allowed,active_goal=self.active_id,error_rad=error,gimbal_feedback_yaw=self.gimbal[1] if self.gimbal else None,gimbal_target=self.target_command,look_source=look_source if 'look_source' in locals() else None,look_heading=look if 'look' in locals() else None,target_heading=self.policy.target,requested_heading=desired if 'desired' in locals() else None,aperture_yaw=axis_yaw if 'axis_yaw' in locals() else None,raw_frames=len(self.policy.raw),converted_frames=len(self.policy.converted),fov_probe_degrees=fov,gyro_norm=self.primary_imu[2] if self.primary_imu else None,cloud_errors=dict(self.cloud_errors),ros=ros)
        state['mid_geometry_required']=self.geometry_guard_required;state['mid_geometry_fresh']=self.geometry_health.healthy(ros,wall) if self.geometry_guard_required else None
        if self.intent:
            request_stamp=self.intent[1].header.stamp
            state['intent_stamp']=[request_stamp.sec,request_stamp.nanosec]
        self.status.publish(String(data=json.dumps(state,separators=(',',':'))))
        key=(reason,self.policy.state,allowed)
        if key!=self.last_state or wall-self.last_diagnostic>=1.:self.get_logger().info(json.dumps(state));self.last_state=key;self.last_diagnostic=wall

if __name__=='__main__':
    from c075_ros_lifecycle import init_context,spin_context,shutdown_context
    lifecycle=init_context();node=Observer()
    try:spin_context(node,lifecycle)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.stop.publish(Bool(data=True))
        node.destroy_node()
        shutdown_context(lifecycle)
