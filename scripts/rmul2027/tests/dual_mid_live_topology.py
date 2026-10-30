import argparse
import collections
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2

parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
rclpy.init()
node=Node('dual_mid_topology_observer',namespace='/red_standard_robot1',parameter_overrides=[Parameter('use_sim_time',value=True)])
counts=collections.Counter();first={};last={};frames={};bad=[]
def cloud(name,msg):
 counts[name]+=1
 t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
 first.setdefault(name,t);last[name]=t;frames[name]=msg.header.frame_id
 if name=='auxiliary/obstacles' and msg.width:
  points=np.frombuffer(msg.data,dtype='<f4').reshape(-1,4)
  if not np.isfinite(points).all() or not np.all(points[:,3]==1):bad.append('invalid_output')
subs=[node.create_subscription(PointCloud2,name,lambda m,n=name:cloud(n,m),qos_profile_sensor_data)
      for name in ['livox/lidar','auxiliary/livox/lidar','auxiliary/obstacles']]
def pump(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
try:
 params={}
 for name in ['local_costmap','global_costmap']:
  c=node.create_client(GetParameters,name+'/'+name+'/get_parameters')
  assert c.wait_for_service(timeout_sec=50),name
  req=GetParameters.Request();req.names=['plugins','aux_mid_obstacle_layer.aux_mid_cloud.topic','aux_mid_obstacle_layer.aux_mid_cloud.sensor_frame','aux_mid_obstacle_layer.aux_mid_cloud.observation_persistence']
  future=c.call_async(req);end=time.monotonic()+5
  while not future.done() and time.monotonic()<end:pump(.05)
  vals=future.result().values
  assert 'aux_mid_obstacle_layer' in vals[0].string_array_value
  assert 'rplidar_obstacle_layer' not in vals[0].string_array_value
  assert vals[1].string_value.endswith('/auxiliary/obstacles') and vals[2].string_value=='aux_mid360' and vals[3].double_value==2.
  params[name]=dict(plugins=list(vals[0].string_array_value),topic=vals[1].string_value,frame=vals[2].string_value,persistence=vals[3].double_value)
 pump(8.)
 assert all(counts[k]>10 and last[k]>first[k] for k in ['livox/lidar','auxiliary/livox/lidar','auxiliary/obstacles']),dict(counts)
 assert frames['auxiliary/livox/lidar']==frames['auxiliary/obstacles']=='aux_mid360',frames
 assert not bad,bad
 old={n:node.count_publishers(n) for n in ['rplidar_a2/scan','rplidar_obstacles']}
 assert all(v==0 for v in old.values()),old
 args.output.write_text(json.dumps(dict(passed=True,counts=dict(counts),first_stamp=first,last_stamp=last,frames=frames,maps=params,old_publishers=old),indent=2))
 print('LIVE_TOPOLOGY_PASSED',json.dumps(dict(counts)),flush=True)
finally:
 node.destroy_node();rclpy.shutdown()
