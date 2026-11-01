"""Real ROS TF-chain contract: native scan frame and near-height obstacle survive."""
import json,math,signal,struct,subprocess,sys,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
record=Path(sys.argv[1]);record.mkdir(exist_ok=False)
rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'])
node=Node('native_scan_observer',namespace='/c072_native_scan_contract')
received=[];checks=[]
pub=node.create_publisher(PointCloud2,'auxiliary/livox/lidar',qos_profile_sensor_data)
sub=node.create_subscription(PointCloud2,'auxiliary/obstacles',lambda msg:received.append(msg),qos_profile_sensor_data)
log=(record/'converter.log').open('x')
process=subprocess.Popen([sys.executable,str(Path(__file__).resolve().parents[1]/'aux_mid_obstacle_cloud.py'),'--ros-args','-r','__ns:=/c072_native_scan_contract','-r','/tf:=tf','-r','/tf_static:=tf_static','-p','source_frame:=aux_mid360_scan'],stdout=log,stderr=subprocess.STDOUT)
def pump(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  assert process.poll() is None,'converter exited'
  rclpy.spin_once(node,timeout_sec=.02)
def cloud(frame='aux_mid360_scan'):
 msg=PointCloud2();msg.header.frame_id=frame;msg.header.stamp=node.get_clock().now().to_msg()
 msg.height=1;msg.width=3;msg.point_step=16;msg.row_step=48
 msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z','intensity'])]
 # With the native +.03 scan origin, the first point is at base height.07.
 # Omitting that origin makes its height.055 and would incorrectly discard it.
 near_z=(.07-.42-.015+math.sin(math.pi/3))/math.cos(math.pi/3)
 floor_z=(0.-.42-.015+math.sin(math.pi/3))/math.cos(math.pi/3)
 msg.data=struct.pack('<12f',1.,0.,near_z,0.,2.,0.,4.,0.,1.,0.,floor_z,0.)
 return msg,near_z
def send(msg):
 for _ in range(5):pub.publish(msg);pump(.08)
try:
 pump(1.5);msg,z=cloud();send(msg);assert not received
 checks.append('missing_scan_tf_drops_no_identity')
 broadcaster=StaticTransformBroadcaster(node)
 parent=TransformStamped();parent.header.frame_id='base_footprint';parent.child_frame_id='aux_mid360';parent.transform.translation.z=.42
 parent.transform.rotation.y=math.sin(math.pi/6);parent.transform.rotation.w=math.cos(math.pi/6)
 scan=TransformStamped();scan.header.frame_id='aux_mid360';scan.child_frame_id='aux_mid360_scan';scan.transform.translation.z=.03;scan.transform.rotation.w=1.
 broadcaster.sendTransform([parent,scan]);pump(.5)
 msg,z=cloud();send(msg);assert received
 for out in received:
  assert out.header==msg.header and out.width==1
  x,y,actual_z,intensity=struct.unpack('<4f',out.data)
  assert abs(x-1)<1e-6 and abs(y)<1e-6 and abs(actual_z-z)<2e-6 and intensity==1.
 checks+=['native_tf_chain_keeps_true_7cm_obstacle','xyz_matches_declared_scan_frame','zero_reflectivity_is_occupancy_one']
 received.clear();msg,z=cloud();msg.header.stamp.sec-=2;send(msg)
 assert received and all(out.header.stamp==msg.header.stamp for out in received)
 checks.append('historical_source_stamp_not_refreshed')
 received.clear();msg,z=cloud('aux_mid360');send(msg);assert not received
 checks.append('mount_frame_cannot_impersonate_scan_frame')
 received.clear();msg,z=cloud();msg.header.stamp.sec=msg.header.stamp.nanosec=0;send(msg);assert not received
 checks.append('zero_source_stamp_drops')
 (record/'result.json').write_text(json.dumps(dict(passed=True,checks=checks,scope='ROS TF contract, no physical navigation claim'),indent=2)+'\n')
 print('NATIVE_SCAN_ROS_CONTRACT',len(checks),'PASS',flush=True)
finally:
 if process.poll() is None:process.send_signal(signal.SIGINT);process.wait(timeout=5)
 node.destroy_node();rclpy.shutdown();log.close()
