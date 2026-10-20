import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener

parser=argparse.ArgumentParser()
parser.add_argument('--stop',action='store_true')
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'])
node=Node('rmul_runtime_inspect',namespace='/red_standard_robot1')
buffer=Buffer()
listener=TransformListener(buffer,node)
qos=QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
mapq=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
values={}
clock_samples=[]
def update(name,msg):
    values[name]=msg
    if name=='clock':
        clock_samples.append((time.monotonic(),msg.clock.sec+msg.clock.nanosec*1e-9))
subs=[node.create_subscription(kind,topic,lambda m,key=key:update(key,m),quality) for kind,topic,key,quality in [(Clock,'/clock','clock',qos),(Odometry,'chassis_odometry_gt','gt',qos),(Twist,'cmd_vel','command',qos),(OccupancyGrid,'map','map',mapq)]]
result={}
if args.stop:
    client=node.create_client(CancelGoal,'navigate_to_pose/_action/cancel_goal')
    if client.wait_for_service(timeout_sec=5):
        future=client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        result['cancelled_goals']=len(future.result().goals_canceling) if future.done() else 'timeout'
    else:
        result['cancelled_goals']='service unavailable'
start=time.monotonic()
while time.monotonic()-start<10:
    rclpy.spin_once(node,timeout_sec=.1)
if len(clock_samples)>1:
    result['real_time_factor']=(clock_samples[-1][1]-clock_samples[0][1])/(clock_samples[-1][0]-clock_samples[0][0])
if 'gt' in values:
    gt=values['gt']
    p=gt.pose.pose.position
    v=gt.twist.twist
    result['truth_world_xy']=[p.x,p.y]
    result['truth_map_xy']=[p.x+4.8,p.y-3.0]
    result['actual_velocity']=[v.linear.x,v.linear.y,v.angular.z]
try:
    tf=buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
    p=tf.transform.translation
    result['estimated_map_xy']=[p.x,p.y]
    if 'truth_map_xy' in result:
        result['localization_error_m']=math.dist(result['truth_map_xy'],[p.x,p.y])
except Exception as error:
    result['tf_error']=str(error)
if 'command' in values:
    v=values['command']
    result['command_velocity']=[v.linear.x,v.linear.y,v.angular.z]
if 'map' in values:
    grid=values['map']
    result['map_dimensions']=[grid.info.width,grid.info.height]
    result['known_fraction']=sum(v>=0 for v in grid.data)/len(grid.data)
    cells={}
    for key in ['estimated_map_xy','truth_map_xy']:
        if key in result:
            x,y=result[key]
            col=int(math.floor((x-grid.info.origin.position.x)/grid.info.resolution))
            row=int(math.floor((y-grid.info.origin.position.y)/grid.info.resolution))
            cells[key]=grid.data[row*grid.info.width+col] if 0<=col<grid.info.width and 0<=row<grid.info.height else 'outside'
    result['map_cells_at_robot']=cells
result['wall_epoch']=time.time()
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False),flush=True)
node.destroy_node()
rclpy.shutdown()
