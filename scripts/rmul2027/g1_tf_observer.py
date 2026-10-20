import argparse
import collections
import datetime
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path as NavPath
from action_msgs.msg import GoalStatusArray
from tf2_ros import Buffer

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
parser.add_argument('--duration',type=float,default=300.)
parser.add_argument('--interval',type=float,default=2.)
parser.add_argument('--namespace',default='red_standard_robot1')
args=parser.parse_args()
args.record_dir.mkdir(parents=True,exist_ok=True)
rclpy.init()
node=Node('g1_tf_observer',namespace='/'+args.namespace.strip('/'))
node.set_parameters([Parameter('use_sim_time',value=True)])
buffer=Buffer(node=node)
counts=collections.Counter()
edges={}
footprints={}
plan=None
statuses=[]
sim_time=None
anomalies=[]
started=time.monotonic()
report_count=0
max_footprint_age={}
streams={name:(args.record_dir/name).open('w') for name in ['events.jsonl','samples.jsonl']}
meta={'started_at':datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'options':{**vars(args),'record_dir':str(args.record_dir)},'read_only':True,'sends_goals':False,'changes_parameters_on_other_nodes':False,'per_message_publisher_gid_available':False}
(args.record_dir/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')

def event(kind,**values):
    value={'event':kind,'wall_epoch':time.time(),'sim_time':sim_time,**values}
    streams['events.jsonl'].write(json.dumps(value)+'\n')
    streams['events.jsonl'].flush()
    if kind not in ['tf_exception']:print(json.dumps(value),flush=True)

def clock_callback(msg):
    global sim_time
    value=msg.clock.sec+msg.clock.nanosec*1e-9
    if sim_time is not None and value<sim_time-.001:event('clock_jump',previous=sim_time,new=value)
    sim_time=value
    counts['clock']+=1

def tf_callback(msg,is_static):
    gid='per_message_gid_unavailable_in_installed_rclpy'
    counts['tf_static' if is_static else 'tf']+=1
    for transform in msg.transforms:
        stamp=transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
        key=transform.header.frame_id+'->'+transform.child_frame_id
        if key not in edges:edges[key]={'parent':transform.header.frame_id,'child':transform.child_frame_id,'static':is_static,'count':0,'first_stamp':stamp,'max_stamp':stamp,'gids':[],'future_events':0,'backward_events':0}
        edge=edges[key]
        if not is_static and sim_time is not None and sim_time>1 and stamp>sim_time+2:
            edge['future_events']+=1
            if edge['future_events']<=3:
                anomalies.append({'edge':key,'stamp':stamp,'clock':sim_time,'kind':'future_tf'})
                event('future_tf',edge=key,stamp=stamp,publisher_gid=gid)
        if not is_static and edge['count'] and stamp<edge['latest_stamp']-.5:
            edge['backward_events']+=1
            if edge['backward_events']<=3:event('tf_stamp_backwards',edge=key,previous=edge['latest_stamp'],new=stamp,publisher_gid=gid)
        edge.update(latest_stamp=stamp,max_stamp=max(edge['max_stamp'],stamp),last_received_wall=time.time(),xyz=[transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z])
        edge['count']+=1
        if gid not in edge['gids']:edge['gids'].append(gid)
        try:
            if is_static:buffer.set_transform_static(transform,gid)
            else:buffer.set_transform(transform,gid)
        except Exception as error:
            counts['tf_exceptions']+=1
            if counts['tf_exceptions']<=5:event('tf_exception',edge=key,error=str(error))

def footprint_callback(key,msg):
    points=msg.polygon.points
    counts[key]+=1
    if points:footprints[key]={'frame':msg.header.frame_id,'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'center_xy':[sum(p.x for p in points)/len(points),sum(p.y for p in points)/len(points)]}

def plan_callback(msg):
    global plan
    counts['plan']+=1
    plan={'frame':msg.header.frame_id,'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'points':len(msg.poses)}
    if msg.poses:plan.update(first_xy=[msg.poses[0].pose.position.x,msg.poses[0].pose.position.y],last_xy=[msg.poses[-1].pose.position.x,msg.poses[-1].pose.position.y])

def status_callback(msg):
    global statuses
    statuses=[int(value.status) for value in msg.status_list]

sensor=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
static=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscriptions=[node.create_subscription(Clock,'/clock',clock_callback,sensor),node.create_subscription(TFMessage,'tf',lambda msg:tf_callback(msg,False),100),node.create_subscription(TFMessage,'tf_static',lambda msg:tf_callback(msg,True),static),node.create_subscription(NavPath,'plan',plan_callback,10),node.create_subscription(GoalStatusArray,'navigate_to_pose/_action/status',status_callback,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
def make_footprint_callback(key):
    def callback(msg):footprint_callback(key,msg)
    return callback

for key,topic in [('global_footprint','global_costmap/published_footprint'),('local_footprint','local_costmap/published_footprint')]:
    subscriptions.append(node.create_subscription(PolygonStamped,topic,make_footprint_callback(key),10))

def sample():
    global report_count
    lookups={}
    for parent,child in [('map','base_footprint'),('map','gimbal_yaw_fake'),('odom','gimbal_yaw_fake')]:
        try:
            value=buffer.lookup_transform(parent,child,rclpy.time.Time())
            stamp=value.header.stamp.sec+value.header.stamp.nanosec*1e-9
            lookups[parent+'->'+child]={'stamp':stamp,'age':None if sim_time is None else sim_time-stamp,'xy':[value.transform.translation.x,value.transform.translation.y]}
        except Exception as error:lookups[parent+'->'+child]={'error':str(error)}
    fps={}
    for key,value in footprints.items():
        age=None if sim_time is None else sim_time-value['stamp']
        fps[key]={**value,'age':age,'count':counts[key]}
        if age is not None:max_footprint_age[key]=max(max_footprint_age.get(key,age),age)
    edge_state={key:{**value,'age':None if value['static'] or sim_time is None else sim_time-value['latest_stamp']} for key,value in edges.items()}
    value={'wall_epoch':time.time(),'elapsed_wall':time.monotonic()-started,'sim_time':sim_time,'counts':dict(counts),'edges':edge_state,'lookups':lookups,'footprints':fps,'plan':plan,'statuses':statuses}
    if plan and 'first_xy' in plan:
        current=lookups.get('map->gimbal_yaw_fake',{})
        if plan['frame']=='map' and 'xy' in current:value['plan_start_robot_distance_m']=math.dist(plan['first_xy'],current['xy'])
    streams['samples.jsonl'].write(json.dumps(value)+'\n');streams['samples.jsonl'].flush()
    (args.record_dir/'latest.json').write_text(json.dumps(value,indent=2)+'\n')
    if report_count%5==0:print(json.dumps({'event':'sample','sim_time':sim_time,'edges':len(edges),'lookup':lookups.get('map->gimbal_yaw_fake'),'footprints':fps,'plan_count':counts['plan']}),flush=True)
    report_count+=1

next_report=time.monotonic()+args.interval
completed=False
event('observer_started')
try:
    while rclpy.ok() and time.monotonic()-started<args.duration:
        rclpy.spin_once(node,timeout_sec=.05)
        if time.monotonic()>=next_report:
            sample();next_report=time.monotonic()+args.interval
    completed=True
except KeyboardInterrupt:
    event('observer_interrupted')
except Exception as error:
    event('observer_failed',error=str(error))
    raise
finally:
    sample()
    (args.record_dir/'tree-final.yaml').write_text(buffer.all_frames_as_yaml())
    endpoints={}
    for topic in ['tf','tf_static','global_costmap/published_footprint','local_costmap/published_footprint']:
        absolute='/'+args.namespace.strip('/')+'/'+topic
        endpoints[absolute]=[{'node':v.node_namespace+'/'+v.node_name,'gid':bytes(v.endpoint_gid).hex(),'qos':str(v.qos_profile)} for v in node.get_publishers_info_by_topic(absolute)]
    (args.record_dir/'publishers.json').write_text(json.dumps(endpoints,indent=2)+'\n')
    summary={'capture_complete':completed,'clock_seen':bool(counts['clock']),'counts':dict(counts),'max_footprint_age':max_footprint_age,'tf_timestamp_anomalies':anomalies,'root_cause_fixed':False,'elapsed_wall_s':time.monotonic()-started}
    (args.record_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    event('observer_finished',summary=summary)
    for stream in streams.values():stream.close()
    node.destroy_node();rclpy.shutdown()
