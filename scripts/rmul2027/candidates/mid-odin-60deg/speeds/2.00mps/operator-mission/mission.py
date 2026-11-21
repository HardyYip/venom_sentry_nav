"""Operator recall on an isolated Gazebo chassis. Ground truth is for test assertions only."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from example_interfaces.msg import Bool
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

out = Path(sys.argv[1])
rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'])
node = Node('operator_mission', namespace='/red_standard_robot1',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
tf = Buffer()
listener = TransformListener(tf, node)
client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
if node.count_publishers('decision_stop'):
    raise RuntimeError('Another decision owner exists')
permit = node.create_publisher(Bool, 'decision_stop', 1)
state = dict(stop=True, recall=False, interruptible=False, leg='startup')
timer = node.create_timer(.05, lambda: permit.publish(Bool(data=state['stop'])))
truth = {}
command = {}
events = []
trace = (out/'trajectory.jsonl').open('w')
def now():
    return node.get_clock().now().nanoseconds / 1e9
def event(name, **fields):
    item = dict(event=name, sim=now(), **fields)
    events.append(item)
    print(json.dumps(item), flush=True)
    (out/'events.json').write_text(json.dumps(events, indent=2))
def odom(m):
    p=m.pose.pose.position
    v=m.twist.twist
    truth.update(x=p.x+4.8, y=p.y-3., speed=math.hypot(v.linear.x,v.linear.y),
                 angular=abs(v.angular.z), time=time.monotonic(),
                 stamp=m.header.stamp.sec + m.header.stamp.nanosec*1e-9)
    if truth['stamp']-state.get('trace_stamp',-1) >= .04:
        trace.write(json.dumps(dict(leg=state['leg'], **truth))+'\n')
        trace.flush()
        state['trace_stamp']=truth['stamp']
def cmd(m):
    command.update(speed=math.hypot(m.linear.x,m.linear.y), angular=abs(m.angular.z),time=time.monotonic())
def recall(_, reply):
    reply.success=state['interruptible'] and not state['recall']
    if reply.success:
        state['recall']=True
        event('operator_recall_received')
    reply.message='accepted' if reply.success else 'no interruptible leg'
    return reply
subs=[node.create_subscription(Odometry,'chassis_odometry_gt',odom,qos_profile_sensor_data),
      node.create_subscription(Twist,'cmd_vel',cmd,10)]
srv=node.create_service(Trigger,'operator_recall',recall)
def until(predicate, seconds):
    end=time.monotonic()+seconds
    while not predicate():
        if time.monotonic()>end:
            raise TimeoutError(state['leg'])
        rclpy.spin_once(node,timeout_sec=.04)
def await_result(future, seconds):
    until(future.done,seconds)
    return future.result()
def still(seconds):
    start=None
    end=time.monotonic()+8
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.04)
        fresh=all(m and time.monotonic()-m['time']<.3 for m in (truth,command))
        if fresh and max(truth['speed'],truth['angular'],command['speed'],command['angular'])<.03:
            if start is None:start=truth['stamp']
            if truth['stamp']-start>=seconds:
                return
        else:start=None
    raise RuntimeError('Stop or dwell not verified: '+state['leg'])
active=None
def leg(name, target, interruptible=False):
    global active
    state['leg']=name
    goal=NavigateToPose.Goal()
    goal.pose.header.frame_id='map'
    goal.pose.header.stamp=node.get_clock().now().to_msg()
    goal.pose.pose.position.x,goal.pose.pose.position.y=target
    goal.pose.pose.orientation=tf.lookup_transform('map','gimbal_yaw_fake',rclpy.time.Time()).transform.rotation
    active=await_result(client.send_goal_async(goal),10)
    if not active.accepted:raise RuntimeError('Goal rejected: '+name)
    uuid=bytes(active.goal_id.uuid).hex()
    event('goal_accepted',leg=name,xy=target,uuid=uuid)
    result=active.get_result_async()
    state['interruptible']=interruptible
    until(lambda: result.done() or (interruptible and state['recall']),150)
    state['interruptible']=False
    canceled=interruptible and state['recall']
    if canceled:
        assert not result.done()
        response=await_result(active.cancel_goal_async(),5)
        assert any(bytes(g.goal_id.uuid).hex()==uuid for g in response.goals_canceling)
    status=await_result(result,10).status
    assert status==(GoalStatus.STATUS_CANCELED if canceled else GoalStatus.STATUS_SUCCEEDED),(name,status)
    active=None
    event('goal_terminal',leg=name,uuid=uuid,status=int(status))
    still(1 if canceled else 2)
    error=math.hypot(truth['x']-target[0],truth['y']-target[1])
    if not canceled:assert error<=.25,(name,error)
    event('stop_verified',leg=name,xy=[truth['x'],truth['y']],error_m=error,dwell_s=1 if canceled else 2)
try:
    until(lambda: truth and tf.can_transform('map','gimbal_yaw_fake',rclpy.time.Time()),45)
    assert client.wait_for_server(timeout_sec=10)
    state['stop']=False
    a=(2.1,-5.5); b=(5.,-5.5); base=(1.,-5.5)
    leg('patrol_A',a)
    leg('patrol_B',b,True)
    assert state['recall']
    leg('operator_return_base',base)
    leg('resume_B',b)
    leg('patrol_A_again',a)
    event('mission_passed')
finally:
    state['stop']=True
    permit.publish(Bool(data=True))
    if active is not None and active.accepted:
        await_result(active.cancel_goal_async(),5)
    trace.close()
    node.destroy_node()
    rclpy.shutdown()
