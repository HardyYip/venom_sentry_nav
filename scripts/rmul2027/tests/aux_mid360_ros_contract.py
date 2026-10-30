import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

record=Path(sys.argv[1]);record.mkdir(exist_ok=False)
script=Path(__file__).resolve().parents[1]/'aux_mid_obstacle_cloud.py'
rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'])
node=Node('observer',namespace='/c070_aux_contract')
received=[]
pub=node.create_publisher(PointCloud2,'auxiliary/livox/lidar',qos_profile_sensor_data)
sub=node.create_subscription(PointCloud2,'auxiliary/obstacles',lambda m:received.append(m),qos_profile_sensor_data)
log=(record/'converter.log').open('w')
process=subprocess.Popen([sys.executable,str(script),'--ros-args','-r','__ns:=/c070_aux_contract','-r','/tf:=tf','-r','/tf_static:=tf_static'],stdout=log,stderr=subprocess.STDOUT)
checks=[]
def pump(t):
 end=time.monotonic()+t
 while time.monotonic()<end:
  assert process.poll() is None
  rclpy.spin_once(node,timeout_sec=.02)
def cloud(frame='aux_mid360'):
 m=PointCloud2();m.header.frame_id=frame;m.header.stamp=node.get_clock().now().to_msg()
 m.height=1;m.width=3;m.point_step=16;m.row_step=48
 m.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z','intensity'])]
 m.data=struct.pack('<12f',1,0,.2,0,2,0,.2,0,1,0,-.1,0)
 return m
def send(m):
 for _ in range(5):pub.publish(m);pump(.08)
try:
 pump(1.5)
 send(cloud());assert not received;checks.append('missing_tf_drops_frame')
 broadcaster=StaticTransformBroadcaster(node)
 t=TransformStamped();t.header.frame_id='base_footprint';t.child_frame_id='aux_mid360';t.transform.rotation.w=1.
 broadcaster.sendTransform(t);pump(.4)
 m=cloud();send(m);assert received
 for out in received:
  assert out.header==m.header and out.width==1
  x,y,z,i=struct.unpack('<4f',out.data)
  assert abs(x-1)<1e-6 and abs(z-.2)<1e-6 and i==1.
 checks+=['source_stamp_preserved','original_xyz_preserved','zero_reflectivity_becomes_occupancy_one','nearest_per_bin','floor_filtered']
 received.clear();send(cloud('wrong_frame'));assert not received;checks.append('wrong_frame_drops')
 m=cloud();m.header.stamp.sec=m.header.stamp.nanosec=0
 send(m);assert not received;checks.append('zero_stamp_drops')
 m=cloud();m.data=b'bad'
 send(m);assert not received;checks.append('malformed_cloud_drops')
 m=cloud();m.data=struct.pack('<12f',float('nan'),0,.2,0,1,0,-1,0,.1,0,.2,0)
 send(m);assert received and all(x.width==0 for x in received);checks.append('empty_valid_observation')
 (record/'result.json').write_text(json.dumps(dict(passed=True,checks=checks),indent=2))
 print('ROS_CONTRACT',len(checks),'passed',flush=True)
finally:
 process.send_signal(signal.SIGINT);process.wait(timeout=5)
 node.destroy_node();rclpy.shutdown();log.close()
