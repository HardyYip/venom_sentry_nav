"""Exercise real source-time TF and strict failure in the privately built adapter."""
import argparse
import json
import math
from pathlib import Path
import signal
import struct
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from builtin_interfaces.msg import Time

parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--build-root',type=Path,required=True)
parser.add_argument('--profile',type=Path,required=True)
args=parser.parse_args()
args.output.mkdir(exist_ok=False)
mount=json.loads(args.profile.read_text())['mounts']['primary']
rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'])
node=Node('strict_scan_contract',namespace='/c072_tf_contract')
odoms=[];scans=[];bases=[]
node.create_subscription(Odometry,'odometry',odoms.append,10)
node.create_subscription(PointCloud2,'sensor_scan',scans.append,10)
def tf_received(msg):
    bases.extend(x for x in msg.transforms if x.child_frame_id=='contract_base')
node.create_subscription(TFMessage,'tf',tf_received,100)
pub_odom=node.create_publisher(Odometry,'lidar_odometry',10)
pub_cloud=node.create_publisher(PointCloud2,'registered_scan',10)
dynamic=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
log=(args.output/'adapter.log').open('w')
binary=args.build_root/'install/sensor_scan_generation/lib/sensor_scan_generation/sensor_scan_generation_node'
cmd=[str(binary),'--ros-args','-r','__ns:=/c072_tf_contract','-r','/tf:=tf','-r','/tf_static:=tf_static',
     '-p','require_valid_tf:=true','-p','lidar_frame:=contract_lidar','-p','base_frame:=contract_base','-p','robot_base_frame:=contract_yaw']
proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
checks=[]
def check(name,condition):
    assert condition,name
    checks.append(name)
def pump(duration):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        assert proc.poll() is None
        rclpy.spin_once(node,timeout_sec=.02)
def stamp(seconds):
    ns=int(seconds*1e9)
    return Time(sec=ns//1000000000,nanosec=ns%1000000000)
def transform(parent,child,t,xyz,q):
    msg=TransformStamped();msg.header.frame_id=parent;msg.child_frame_id=child;msg.header.stamp=stamp(t)
    msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=map(float,xyz)
    msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=map(float,q)
    return msg
def send_pair(t,theta):
    rotation=Rotation.from_euler('z',theta)
    mount_r=Rotation.from_matrix(mount['rotation_base_sensor'])
    xyz=np.array([3.,2.,.2136])+rotation.apply(mount['parent_xyz_m'])
    q=(rotation*mount_r).as_quat()
    odom=Odometry();odom.header.frame_id='contract_odom';odom.child_frame_id='contract_lidar';odom.header.stamp=stamp(t)
    odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,xyz)
    odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=map(float,q)
    cloud=PointCloud2();cloud.header.frame_id='contract_odom';cloud.header.stamp=stamp(t)
    cloud.height=cloud.width=1;cloud.point_step=cloud.row_step=16
    cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(['x','y','z','intensity'])]
    cloud.data=struct.pack('<4f',4.,2.,.5,1.)
    pub_cloud.publish(cloud);pub_odom.publish(odom)
    return xyz,rotation*mount_r
def send_tf(t,theta):
    dynamic.sendTransform(transform('contract_base','contract_yaw',t,[0,0,.2136],Rotation.from_euler('z',theta).as_quat()))
try:
    pump(1.5)
    maps=Path('/proc',str(proc.pid),'maps').read_text()
    expected=str(args.build_root/'install/sensor_scan_generation/lib/libsensor_scan_generation.so')
    check('actual_private_library_loaded',expected in maps)
    (args.output/'runtime-library.txt').write_text(expected)
    t=node.get_clock().now().nanoseconds*1e-9
    send_pair(t,0);pump(1.3)
    check('missing_tf_drops_both_outputs',not odoms and not scans and not bases)
    static.sendTransform(transform('contract_yaw','contract_lidar',t,mount['parent_xyz_m'],Rotation.from_matrix(mount['rotation_base_sensor']).as_quat()))
    send_tf(t,0);send_tf(t+.1,math.pi/2);pump(.4)
    # Publish older data after a newer transform exists: latest-TF shortcuts fail this test.
    xyz,rot=send_pair(t,0);pump(.6)
    check('historical_source_stamp_accepted',bool(odoms) and bool(scans) and bool(bases))
    check('historical_yaw_not_replaced_by_latest',abs(odoms[-1].pose.pose.orientation.z)<1e-6)
    check('fixed_base_reconstructed_at_zero',np.allclose([bases[-1].transform.translation.x,bases[-1].transform.translation.y,bases[-1].transform.translation.z],[3,2,0],atol=1e-6))
    check('scan_source_stamp_preserved',scans[-1].header.stamp==stamp(t) and odoms[-1].header.stamp==stamp(t))
    point=np.array(struct.unpack('<4f',scans[-1].data)[:3])
    check('scan_coordinates_use_matching_pose',np.allclose(point,rot.inv().apply(np.array([4,2,.5])-xyz),atol=1e-5))
    send_pair(t+.1,math.pi/2);pump(.6)
    q=odoms[-1].pose.pose.orientation
    angle=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]
    check('rotating_yaw_odometry_correct',abs(angle-math.pi/2)<1e-6)
    check('fixed_base_reconstructed_during_yaw',np.allclose([bases[-1].transform.translation.x,bases[-1].transform.translation.y,bases[-1].transform.translation.z],[3,2,0],atol=1e-6))
    count=len(odoms),len(scans),len(bases)
    send_pair(t+2,math.pi/2);pump(1.3)
    check('extrapolated_tf_drops_no_identity_pose',count==(len(odoms),len(scans),len(bases)))
    send_tf(t+2,math.pi/2);pump(.2);send_pair(t+2,math.pi/2);pump(.6)
    check('actual_tf_return_restores_output',len(odoms)>count[0] and len(scans)>count[1])
    result=dict(passed=True,checks=checks,scope='adapter ROS contract, no Gazebo or physical-stop claim')
    (args.output/'result.json').write_text(json.dumps(result,indent=2))
    print('STRICT_TF_CONTRACT',len(checks),'passed',flush=True)
finally:
    proc.send_signal(signal.SIGINT)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:proc.kill();proc.wait()
    node.destroy_node();rclpy.shutdown();log.close()
