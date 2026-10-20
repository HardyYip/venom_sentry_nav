import json
import math
import time
import statistics
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node=rclpy.create_node('rmul2027_settle_check',namespace='/red_standard_robot1')
messages={}
for kind,topic in [(Clock,'/clock'),(Imu,'livox/imu'),(Odometry,'chassis_odometry_gt'),(PointCloud2,'livox/lidar')]:
    node.create_subscription(kind,topic,lambda msg,t=topic:messages.update({t:msg}),qos_profile_sensor_data)
deadline=time.monotonic()+110
stable_since=None
last_report=0
history=deque()
last_imu_stamp=None
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.05)
    if not all(t in messages for t in ['/clock','livox/imu','chassis_odometry_gt','livox/lidar']):
        continue
    clock=messages['/clock'].clock
    seconds=clock.sec+clock.nanosec*1e-9
    imu=messages['livox/imu']
    acceleration=math.sqrt(imu.linear_acceleration.x**2+imu.linear_acceleration.y**2+imu.linear_acceleration.z**2)
    angular=math.sqrt(imu.angular_velocity.x**2+imu.angular_velocity.y**2+imu.angular_velocity.z**2)
    velocity=messages['chassis_odometry_gt'].twist.twist
    stationary=max(abs(velocity.linear.x),abs(velocity.linear.y),abs(velocity.angular.z))<.01
    stamp=imu.header.stamp.sec+imu.header.stamp.nanosec*1e-9
    if stamp!=last_imu_stamp:
        history.append((stamp,acceleration,angular))
        last_imu_stamp=stamp
    while history and history[0][0]<stamp-.6:
        history.popleft()
    acceleration_mean=statistics.fmean(p[1] for p in history)
    acceleration_std=statistics.pstdev(p[1] for p in history)
    angular_mean=statistics.fmean(p[2] for p in history)
    window_ready=len(history)>=60 and history[-1][0]-history[0][0]>.45
    healthy=(seconds>1.0 and window_ready and abs(acceleration_mean-9.81)<.15
             and acceleration_std<.7 and angular_mean<.02 and angular<.03 and stationary
             and messages['livox/lidar'].width*messages['livox/lidar'].height>0)
    if healthy:
        if stable_since is None:
            stable_since=seconds
        if seconds-stable_since>=.5:
            print(json.dumps({'event':'settled','sim_time':seconds,'acceleration_norm':acceleration,
                              'angular_norm':angular,'acceleration_mean':acceleration_mean,'acceleration_std':acceleration_std,'stable_sim_seconds':seconds-stable_since}),flush=True)
            break
    else:
        stable_since=None
    if time.monotonic()-last_report>10:
        print(json.dumps({'event':'waiting_for_settle','sim_time':seconds,
                          'acceleration_norm':acceleration,'angular_norm':angular,'acceleration_mean':acceleration_mean,'acceleration_std':acceleration_std}),flush=True)
        last_report=time.monotonic()
else:
    raise RuntimeError('Robot or IMU did not settle before navigation startup')
node.destroy_node()
rclpy.shutdown()
