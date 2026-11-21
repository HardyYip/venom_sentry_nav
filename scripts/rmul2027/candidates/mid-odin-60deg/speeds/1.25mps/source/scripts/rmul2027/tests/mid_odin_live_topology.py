"""Read actual C075 topics and source-time TF before any motion acceptance."""
import argparse,json,time,sys
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2,Imu
from tf2_ros import Buffer,TransformException,TransformListener
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'])
n=rclpy.create_node('mid_odin_topology_probe',namespace='/red_standard_robot1')
n.set_parameters([Parameter('use_sim_time',value=True)])
buf=Buffer(node=n);listener=TransformListener(buf,n)
data={k:[] for k in ['mid','odin','obstacles','imu']}
pending=[]
def cb(key,msg):
 t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
 entry={'stamp':t,'frame':msg.header.frame_id,'width':getattr(msg,'width',None),'wall':time.monotonic()}
 if key!='imu':
  entry['tf_ok']=False
  if t>0:
   pending.append((rclpy.time.Time.from_msg(msg.header.stamp),entry))
 data[key].append(entry);data[key]=data[key][-100:]

def poll_transforms():
 wall=time.monotonic()
 remaining=[]
 for stamp,entry in pending:
  age=wall-entry['wall']
  if age>.25:
   entry.setdefault('error','source-time transform did not arrive within .25 s')
   continue
  try:
   tf=buf.lookup_transform('base_footprint',entry['frame'],stamp)
  except TransformException as ex:
   entry['error']=str(ex)
   remaining.append((stamp,entry))
   continue
  entry.pop('error',None)
  entry['tf_ok']=True
  entry['tf_wait_wall_s']=age
  entry['xyz']=[getattr(tf.transform.translation,c) for c in ['x','y','z']]
 pending[:]=remaining
subs=[]
for key,topic,typ in [('mid','livox/lidar',PointCloud2),('odin','odin/points',PointCloud2),('obstacles','odin/obstacles',PointCloud2),('imu','livox/imu',Imu)]:
 subs.append(n.create_subscription(typ,topic,lambda m,k=key:cb(k,m),qos_profile_sensor_data))
end=time.monotonic()+35
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.02)
 poll_transforms()
 if all(len(v)>=8 for v in data.values()) and all(sum(x.get('tf_ok',False) for x in data[k])>=3 for k in ['mid','odin','obstacles']):break
checks={}
for k in data:
 checks[k+'_arrives']=len(data[k])>=3
 checks[k+'_fresh_source_time']=len({x['stamp'] for x in data[k] if x['stamp']>0})>=3
for k,frame in [('mid','front_mid360_scan'),('odin','odin_scan'),('obstacles','odin_scan')]:
 checks[k+'_frame']=bool(data[k]) and all(x['frame']==frame for x in data[k])
 checks[k+'_source_stamp_tf']=sum(x.get('tf_ok',False) for x in data[k])>=3
 checks[k+'_nonempty']=any((x.get('width') or 0)>0 for x in data[k])
checks['no_old_aux_raw_publisher']=n.count_publishers('auxiliary/livox/lidar')==0
checks['two_current_raw_publishers']=n.count_publishers('livox/lidar')==1 and n.count_publishers('odin/points')==1
result={'passed':all(checks.values()),'checks':checks,'observations':data,'scope':'live topics/TF only; no motion qualification'}
a.output.write_text(json.dumps(result,indent=2));print(json.dumps({'passed':result['passed'],'checks':checks}))
n.destroy_node();rclpy.shutdown();raise SystemExit(0 if result['passed'] else 1)
