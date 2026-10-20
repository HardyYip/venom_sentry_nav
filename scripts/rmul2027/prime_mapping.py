import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock

rclpy.init()
node = rclpy.create_node('rmul2027_mapping_prime',namespace='/red_standard_robot1')
messages={}
for kind,topic in [(LaserScan,'obstacle_scan'),(Odometry,'chassis_odometry_gt'),(Clock,'/clock')]:
    node.create_subscription(kind,topic,lambda m,t=topic:messages.update({t:m}),qos_profile_sensor_data)
publisher=node.create_publisher(Twist,'cmd_vel_nav2_result',10)
def seconds():
    c=messages['/clock'].clock
    return c.sec+c.nanosec*1e-9
def emit(**values):
    print(json.dumps(values),flush=True)
deadline=time.monotonic()+45
while not all(k in messages for k in ['obstacle_scan','chassis_odometry_gt','/clock']) and time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.1)
assert all(k in messages for k in ['obstacle_scan','chassis_odometry_gt','/clock'])
scan=messages['obstacle_scan']
finite=[r for r in scan.ranges if math.isfinite(r)]
emit(event='scan_before',finite_ranges=len(finite),total=len(scan.ranges),
     min_range=min(finite) if finite else None,max_range=max(finite) if finite else None)
start=seconds()
p=messages['chassis_odometry_gt'].pose.pose.position
xy=(p.x,p.y)
try:
    deadline=time.monotonic()+60
    while seconds()<start+3 and time.monotonic()<deadline:
        command=Twist()
        command.linear.x=.12
        publisher.publish(command)
        rclpy.spin_once(node,timeout_sec=.05)
        p=messages['chassis_odometry_gt'].pose.pose.position
        if math.hypot(p.x-xy[0],p.y-xy[1])>.4:
            break
finally:
    for i in range(20):
        publisher.publish(Twist())
        rclpy.spin_once(node,timeout_sec=.05)
p=messages['chassis_odometry_gt'].pose.pose.position
scan=messages['obstacle_scan']
emit(event='mapping_prime',sim_duration=seconds()-start,initial_world_xy=xy,
     final_world_xyz=[p.x,p.y,p.z],distance=math.hypot(p.x-xy[0],p.y-xy[1]),
     finite_ranges=sum(math.isfinite(r) for r in scan.ranges))
node.destroy_node()
rclpy.shutdown()
