if __name__ == '__main__':
 from storage_entry import ensure_storage_guard
 ensure_storage_guard(__file__)

import argparse
from pathlib import Path
import bisect,collections,datetime,hashlib,json,math,os,select,signal,subprocess,sys,time,traceback
from contact_log import ContactLog
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2,LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Twist
from example_interfaces.msg import Bool
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer,TransformListener
FOLDER=Path(__file__).resolve().parent
WS=FOLDER.parents[3]
parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
parser.add_argument('--spin-mode',action=argparse.BooleanOptionalAction,default=False)
parser.add_argument('--test-decision-permit',action='store_true',help='TEST ONLY direct Action fixture owns decision_stop heartbeat; refuses an existing decision publisher')
parser.add_argument('--launcher',type=Path,help='Optional isolated sensor-profile launcher')
parser.add_argument('--params-file',type=Path,default=FOLDER/'config/nav2_test.yaml')
parser.add_argument('--fixed-acceptance',type=Path,required=True)
options=parser.parse_args()
ROOT=options.record_dir.expanduser().resolve()
ROOT.mkdir(parents=True,exist_ok=False)
PARAMS=options.params_file.expanduser().resolve()
(ROOT/'params.yaml').write_bytes(PARAMS.read_bytes())
flags=subprocess.check_output(['pkg-config','--cflags','--libs','ignition-transport11','ignition-msgs8'],text=True).split()
subprocess.run(['g++','-std=c++17','-O2',str(FOLDER/'actors/scripted_obstacle_actor.cpp'),'-o',str(ROOT/'moving_actor'),*flags],check=True)
SCENE={'label':'headon','start':(4.5,-.2),'goal':(9.3,-.2),'opponent_start':(8.0,-.2),'opponent_end':(4.5,-.2),'speed':0.5}
STATE={'owner':'Codex C032','navigation_changes':False,'record_bag':False,'spin_mode':options.spin_mode,'test_decision_permit':options.test_decision_permit,'cases':[],'started_at':datetime.datetime.now().astimezone().isoformat()}
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def stop_state(p):
 return {'mode':'spin-mode' if options.spin_mode else 'strict-stop',
  'classification':('translation_stopped_still_spinning' if p.angular>.03 or abs(p.cmd[2])>.03 else 'safety_stop') if p.speed<.03 and math.hypot(*p.cmd[:2])<.03 else 'translation_moving',
  'translation_speed_mps':p.speed,'translation_command_mps':math.hypot(*p.cmd[:2]),
  'angular_speed_rps':p.angular,'angular_command_rps':abs(p.cmd[2]),
  'still_spinning':bool(p.angular>.03 or abs(p.cmd[2])>.03),
  'threshold':.03,'source':'chassis_odometry_gt twist and cmd_vel at final check'}
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def matrix(t):
 q=t.rotation;x,y,z,w=q.x,q.y,q.z,q.w
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]),np.array([t.translation.x,t.translation.y,t.translation.z])
def polygon(x,y,a,hx=.29,hy=.25):
 c,s=math.cos(a),math.sin(a)
 return np.array([[x+c*u-s*v,y+s*u+c*v] for u,v in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]])
def clearance(p,q):
 separated=False;penetration=float('inf')
 for points in [p,q]:
  for i in range(2):
   e=points[i+1]-points[i];axis=np.array([-e[1],e[0]])/np.linalg.norm(e);a=p@axis;b=q@axis
   separated|=max(b.min()-a.max(),a.min()-b.max())>0
   penetration=min(penetration,min(a.max(),b.max())-max(a.min(),b.min()))
 if not separated:return -max(penetration,0.)
 answer=float('inf')
 for first,second in [(p,q),(q,p)]:
  for point in first:
   for a,b in zip(second,np.roll(second,-1,axis=0)):
    delta=b-a;t=np.clip(np.dot(point-a,delta)/np.dot(delta,delta),0,1)
    answer=min(answer,float(np.linalg.norm(point-(a+t*delta))))
 return answer

def owned(case):
 result=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit() or int(p.name)==os.getpid():continue
  try:
   if ('RMUL_MOVING_CASE_ID='+str(ROOT)+':'+case).encode()+b'\0' in (p/'environ').read_bytes():
    stat=(p/'stat').read_text();tail=stat[stat.rfind(')')+2:].split();result.append((int(p.name),tail[19]))
  except (OSError,PermissionError,IndexError):pass
 return result

def cleanup(case):
 initial=owned(case)
 for sig,delay in [(signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,1)]:
  rows=owned(case)
  if not rows:break
  for pid,start in rows:
   try:
    text=Path('/proc',str(pid),'stat').read_text();now=text[text.rfind(')')+2:].split()[19]
    if now==start:os.kill(pid,sig)
   except OSError:pass
  end=time.monotonic()+delay
  while time.monotonic()<end and owned(case):time.sleep(.1)
 return {'initial':initial,'remaining':owned(case)}

class Probe(Node):
 def __init__(self,record):
  super().__init__('c032_moving_observer',namespace='/red_standard_robot1',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.record=record;self.phase='preposition';self.targets=[];self.tf=Buffer();self.listener=TransformListener(self.tf,self)
  self.stream=(record/'statistics.jsonl').open('w');self.ring=collections.deque();self.pending=collections.deque(maxlen=100)
  self.latest={};self.last_cloud={};self.errors=collections.Counter();self.grids={};self.truth=None;self.truths=collections.deque(maxlen=3000)
  self.last_sample=0.;self.last_plan=0.;self.last_grid={};self.cmd=[0.,0.,0.];self.minimum=float('inf');self.fault=None;self.fault_wall=None;self.detect_events=[];self.last_detect={};self.maxloc=0.;self.lasttruth=None;self.speed=0.;self.angular=0.;self.counts=collections.Counter();self.contacts_path=record/'contacts.jsonl';self.contact_monitor=ContactLog(self.contacts_path)
  self.opponents=collections.deque(maxlen=6000);self.opp_last=None;self.opp_speed=0.;self.min_opponent_gap=float('inf');self.min_opponent_gap_sim=None;self.opponent_move_started=None;self.opponent_past_endpoint=False;self.stopped_near_opponent=False;self.opponent_pose_handle=None;self.opponent_pose_leftover='';self.opponent_pose_offset=0;self.opponent_read_retries=0
  self.motion_rows=[]
  sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT);latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.stop_pub=self.create_publisher(Bool,'safety_stop',10);self.pending_lasers=collections.deque(maxlen=60)
  self.test_decision_permit=None
  if options.test_decision_permit:
   if self.count_publishers('decision_stop')!=0:raise RuntimeError('Existing decision_stop owner; direct Action test refuses to compete')
   self.test_decision_permit=self.create_publisher(Bool,'decision_stop',1)
   self.test_permit_timer=self.create_timer(.05,lambda:self.test_decision_permit.publish(Bool(data=self.fault is not None)))

  self.subs=[self.create_subscription(LaserScan,'rplidar_a2/scan',lambda m:self.pending_lasers.append((time.monotonic(),m)),sensor)]
  for name in ['livox/lidar','velodyne_points','cloud_registered','registered_scan','terrain_map','terrain_map_ext','rplidar_obstacles','auxiliary/livox/lidar','auxiliary/obstacles']:
   self.subs.append(self.create_subscription(PointCloud2,name,lambda m,k=name:self.cloud(k,m),sensor))
  self.subs += [self.create_subscription(Odometry,'chassis_odometry_gt',self.pose,sensor),self.create_subscription(Twist,'cmd_vel',self.command,10),self.create_subscription(Twist,'cmd_vel_controller',lambda m:self.record_command('cmd_vel_controller',m),10),self.create_subscription(NavPath,'plan',self.plan,10),self.create_subscription(NavPath,'local_plan',lambda m:self.path_record('local_plan',m),10)]
  for name in ['global_costmap','local_costmap']:
   self.subs += [self.create_subscription(OccupancyGrid,name+'/costmap',lambda m,k=name:self.grid(k,m),latched),self.create_subscription(OccupancyGridUpdate,name+'/costmap_updates',lambda m,k=name:self.update(k,m),10)]
 def now(self):return self.get_clock().now().nanoseconds*1e-9
 def emit(self,kind,**v):
  row={'wall':time.time(),'sim':self.now(),'phase':self.phase,'kind':kind,**v};self.stream.write(json.dumps(row,separators=(',',':'))+'\n')
  self.ring.append(row)
  cutoff=time.time()-2.
  if self.fault_wall is not None:cutoff=self.fault_wall-2.
  while self.ring and self.ring[0]['wall']<cutoff:self.ring.popleft()
  return row
 def fail(self,reason,**v):
  if self.fault is None:
   self.fault_wall=time.time();self.fault={'reason':reason,'wall':self.fault_wall,'sim':self.now(),**v};write(self.record/'first-fault.json',self.fault);self.emit('fault',**self.fault)
 def pose(self,m):
  p=m.pose.pose.position;s=stamp(m);a=yaw(m.pose.pose.orientation);self.truth=[p.x+4.8,p.y-3.,a];self.truths.append((s,*self.truth[:2]));self.speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y);self.angular=abs(m.twist.twist.angular.z)
 def command(self,m):self.cmd=[m.linear.x,m.linear.y,m.angular.z];self.record_command('cmd_vel',m)
 def record_command(self,k,m):
  self.counts[k]+=1
  if self.counts[k]%2==0:self.emit(k,value=[m.linear.x,m.linear.y,m.angular.z])
 def plan(self,m):self.path_record('plan',m)
 def path_record(self,k,m):
  if k=='local_plan' and self.counts[k]%5!=0:self.counts[k]+=1;return
  self.counts[k]+=1;self.emit(k,stamp=stamp(m),frame=m.header.frame_id,xy=[[p.pose.position.x,p.pose.position.y] for p in m.poses])
 def process_lasers(self):
  remaining=collections.deque(maxlen=60)
  while self.pending_lasers:
   received,m=self.pending_lasers.popleft();s=stamp(m)
   try:
    a=np.asarray(m.ranges);mask=np.isfinite(a)&(a>=m.range_min)&(a<=m.range_max);angles=m.angle_min+np.arange(len(a))*m.angle_increment
    p=np.column_stack([a[mask]*np.cos(angles[mask]),a[mask]*np.sin(angles[mask]),np.zeros(mask.sum())]);p=self.transform(p,m.header.frame_id,'map',s)
    rois=[]
    for x,y in self.targets_at(s):
     q=p[(abs(p[:,0]-x)<.39)&(abs(p[:,1]-y)<.35)];rois.append({'points':len(q),'xyz':q[:256].round(5).tolist()})
    self.emit('laser',topic='rplidar_a2/scan',stamp=s,source_age_s=self.now()-s,finite_ranges=len(p),roi=rois)
   except Exception as e:
    if time.monotonic()-received<.7:remaining.append((received,m))
    else:self.emit('alignment_missing',topic='rplidar_a2/scan',stamp=s,error=str(e))
  self.pending_lasers=remaining
 def cloud(self,k,m):
  self.counts[k]+=1;s=stamp(m)
  if s-self.last_cloud.get(k,-1)<.095:return
  self.last_cloud[k]=s;self.pending.append((time.monotonic(),k,m))
 def transform(self,p,source,target,s):
  if source==target:return p
  t=self.tf.lookup_transform(target,source,Time(seconds=s)).transform;r,o=matrix(t);return p@r.T+o
 def targets_at(self,s):
  if not self.opponents:return []
  history=list(self.opponents);i=bisect.bisect_right(history,(s,float('inf'),float('inf')))
  if i<=0:return [(history[0][1],history[0][2])]
  if i>=len(history):return [(history[-1][1],history[-1][2])]
  a,b=history[i-1],history[i];f=(s-a[0])/(b[0]-a[0]) if b[0]>a[0] else 0.
  return [(a[1]+f*(b[1]-a[1]),a[2]+f*(b[2]-a[2]))]
 def process_clouds(self):
  remaining=collections.deque(maxlen=100)
  while self.pending:
   received,k,m=self.pending.popleft();s=stamp(m)
   try:
    fields={f.name:f for f in m.fields};names=[f for f in ['x','y','z','intensity'] if f in fields];types={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    dt=np.dtype({'names':names,'formats':[('>' if m.is_bigendian else '<')+types[fields[f].datatype] for f in names],'offsets':[fields[f].offset for f in names],'itemsize':m.point_step})
    a=np.frombuffer(m.data,dtype=dt,count=m.width*m.height);p=np.column_stack([a[f] for f in ['x','y','z']]);method='stamp_tf'
    if m.header.frame_id=='camera_init':
     p=self.transform(p,'front_mid360','base_footprint',s);p=self.transform(p,'odom','map',s);method='loam_interface_verified_initial_extrinsic_then_stamp_tf'
    else:p=self.transform(p,m.header.frame_id,'map',s)
    rois=[];payload=[]
    for x,y in self.targets_at(s):
     mask=(abs(p[:,0]-x)<.39)&(abs(p[:,1]-y)<.35)&np.isfinite(p).all(axis=1);q=p[mask];v={'points':len(q)};extra=np.zeros(len(q))
     if len(q):
      v['z_quantile']=np.quantile(q[:,2],[0,.2,.5,.8,1]).tolist()
      if 'intensity' in names:
       extra=np.array(a['intensity'][mask]);v['intensity_quantile']=np.quantile(extra,[0,.2,.5,.8,1]).tolist();v['intensity_ge_0_1']=int(np.count_nonzero(extra>=.1))
     rois.append(v)
     if len(q):
      indices=np.linspace(0,len(q)-1,min(len(q),256),dtype=int);payload.append(np.column_stack([q[indices],extra[indices]]).round(5).tolist())
     else:payload.append([])
    row=self.emit('cloud',topic=k,stamp=s,frame=m.header.frame_id,transform=method,source_age_s=self.now()-s,total=len(p),roi=rois)
    self.latest[k]=row;row['roi_points_for_failure_window']=payload
   except Exception as e:
    if time.monotonic()-received<.65:remaining.append((received,k,m))
    else:
     self.errors[k+':'+type(e).__name__]+=1
     if self.errors[k+':'+type(e).__name__]<=3:self.emit('alignment_missing',topic=k,stamp=s,error=str(e))
  self.pending=remaining
 def grid(self,k,m):
  self.grids[k]={'frame':m.header.frame_id,'stamp':stamp(m),'origin':[m.info.origin.position.x,m.info.origin.position.y],'r':m.info.resolution,'data':np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)}
 def update(self,k,m):
  if k not in self.grids:return
  g=self.grids[k]
  if m.x+m.width>g['data'].shape[1] or m.y+m.height>g['data'].shape[0]:self.errors[k+':update_bounds']+=1;return
  g['data'][m.y:m.y+m.height,m.x:m.x+m.width]=np.array(m.data,dtype=np.int16).reshape(m.height,m.width);g['stamp']=stamp(m)
 def grid_roi(self,k):
  if k not in self.grids:return None
  g=self.grids[k];s=g['stamp'];rois=[];payload=[]
  for x,y in self.targets_at(s if s>0 else self.now()):
   try:
    p=self.transform(np.array([[x,y,0.]]),'map',g['frame'],s);x2,y2=p[0,:2];i=int(math.floor((x2-g['origin'][0])/g['r']));j=int(math.floor((y2-g['origin'][1])/g['r']))
    if not(0<=i<g['data'].shape[1] and 0<=j<g['data'].shape[0]):rois.append({'outside':True});payload.append(None);continue
    d=g['data'][max(0,j-7):j+8,max(0,i-8):i+9];unique,counts=np.unique(d,return_counts=True);rois.append({'center':int(g['data'][j,i]),'max':int(d.max()),'lethal_cells':int(np.count_nonzero(d>=99)),'values':{str(int(a)):int(b) for a,b in zip(unique,counts)}});payload.append(d.tolist())
   except Exception as e:rois.append({'alignment_missing':str(e)});payload.append(None)
  row=self.emit('costmap',topic=k,stamp=s,frame=g['frame'],source_age_s=self.now()-s,roi=rois);row['roi_arrays_for_failure_window']=payload
  for i,v in enumerate(rois):
   if 'max' not in v:continue
   detected=v['lethal_cells']>0;key=(k,i)
   if self.last_detect.get(key)!=detected:
    event={'sim':self.now(),'topic':k,'target':i+1,'detected':detected,'stamp':s};self.detect_events.append(event);self.emit('detection_transition',**event)
   self.last_detect[key]=detected
  return rois
 def read_opponent(self):
  path=self.record/'gazebo-poses.jsonl'
  if not path.exists():return
  if self.opponent_pose_handle is None:self.opponent_pose_handle=open(path,'rb',buffering=0)
  available=path.stat().st_size-self.opponent_pose_offset
  if available<=0:return
  try:
   self.opponent_pose_handle.seek(self.opponent_pose_offset)
   raw=self.opponent_pose_handle.read(min(available,1048576))
  except OSError as error:
   if error.errno not in [11,61]:raise
   self.opponent_read_retries+=1
   self.emit('opponent_log_read_retry',errno=error.errno,offset=self.opponent_pose_offset)
   return
  self.opponent_pose_offset+=len(raw);chunk=raw.decode('utf-8')
  if not chunk:return
  lines=(self.opponent_pose_leftover+chunk).split('\n');self.opponent_pose_leftover=lines.pop()
  for line in lines:
   try:row=json.loads(line)
   except json.JSONDecodeError:continue
   if row.get('name')!='c020_opponent_cart':continue
   s=float(row['sim']);mx=row['xyz'][0]+4.8;my=row['xyz'][1]-3.0
   self.opponents.append((s,mx,my))
   if self.opp_last is not None:
    dt=s-self.opp_last[0]
    if dt>=.02:self.opp_speed=math.dist((mx,my),self.opp_last[1:])/dt
   self.opp_last=(s,mx,my)
 def sample(self):
  if time.monotonic()-self.last_sample<.05:return
  self.last_sample=time.monotonic();self.read_opponent();self.process_clouds();self.process_lasers()
  if not self.truth:return
  row={'truth_map':self.truth,'truth_speed':self.speed,'cmd_vel':self.cmd}
  target=self.opp_last
  if self.phase=='goal' and self.opponent_move_started is not None and self.now()-self.opponent_move_started<8:
   if target is None or self.now()-target[0]>.35:self.fail('opponent_truth_stale',last_stamp=target[0] if target else None)
  if target is not None:
   gap=clearance(polygon(*self.truth),polygon(target[1],target[2],0))
   row['opponent_map']=[target[1],target[2],target[0]];row['opponent_speed']=self.opp_speed;row['gap_m']=gap
   if self.phase=='goal':
    if gap<self.min_opponent_gap:self.min_opponent_gap=gap;self.min_opponent_gap_sim=self.now()
    if self.opponent_move_started is not None and self.opp_speed<.03 and target[1]<SCENE['opponent_end'][0]+.15:self.opponent_past_endpoint=True
    if gap<=.05 and (math.hypot(*self.cmd[:2])>.03 or self.speed>.03 or self.opp_speed>.03):self.fail('dangerous_approach',gap_m=gap,truth_map=self.truth,opponent=[target[1],target[2]],cmd_vel=self.cmd)
    if gap<2.0 and self.speed<.03:self.stopped_near_opponent=True
  try:
   t=self.tf.lookup_transform('map','base_footprint',Time());s=stamp(t);p=t.transform.translation;row['estimated_map']=[p.x,p.y];row['tf_stamp']=s;row['pose_age_s']=self.now()-s
   history=list(self.truths);i=bisect.bisect_right(history,(s,float('inf'),float('inf')))
   if 0<i<len(history):
    a,b=history[i-1],history[i];f=(s-a[0])/(b[0]-a[0]);truth=[a[j]+f*(b[j]-a[j]) for j in [1,2]];err=math.dist([p.x,p.y],truth);row['aligned_error_m']=err;self.maxloc=max(self.maxloc,err)
    if self.phase=='goal' and err>.35:self.fail('aligned_localization_error',error_m=err,stamp=s,estimate=[p.x,p.y],truth_before=a,truth_after=b)
  except Exception as e:row['pose_alignment_missing']=str(e)
  self.emit('motion',**row)
  self.motion_rows.append({'phase':self.phase,'sim':self.now(),**row})
  if time.monotonic()-getattr(self,'grid_wall',0)>.19:
   self.grid_wall=time.monotonic()
   for k in self.grids:self.grid_roi(k)
  if self.phase=='goal':
   try:
    contacts=self.contact_monitor.poll()
    if contacts:self.fail('physical_contact',truth_map=self.truth,contact=contacts[0])
   except Exception as error:self.fail('contact_monitor_unavailable',error=repr(error))
 def finish(self):
  if self.fault is not None:
   with (self.record/'failure-window.jsonl').open('w') as f:
    for row in self.ring:f.write(json.dumps(row,separators=(',',':'))+'\n')
  write(self.record/'probe-summary.json',{'counts':dict(self.counts),'alignment_errors':dict(self.errors),'detection_transitions':self.detect_events,'minimum_clearance_m':self.minimum if math.isfinite(self.minimum) else None,'minimum_opponent_gap_m':self.min_opponent_gap if math.isfinite(self.min_opponent_gap) else None,'minimum_opponent_gap_sim':self.min_opponent_gap_sim,'opponent_measured_speed_mps':self.opp_speed,'opponent_past_endpoint':self.opponent_past_endpoint,'stopped_near_opponent':self.stopped_near_opponent,'max_aligned_error_m':self.maxloc,'fault':self.fault,'contact_monitor':self.contact_monitor.summary(),'point_window_cap_per_target_per_frame':256,'camera_init_contract':'matches loam_interface: fixed base_footprint->front_mid360 transform into odom, followed by stamped map->odom'})
  self.stream.close()
  if self.opponent_pose_handle:self.opponent_pose_handle.close()
  self.destroy_node()

def run_case(label,repeat):
 start=SCENE['start'];goal=SCENE['goal'];opp=SCENE['opponent_start'];opp_end=SCENE['opponent_end']
 name=f'{label}-{repeat:02d}';record=ROOT/name;record.mkdir(exist_ok=False)
 meta={'case':name,'start':start,'goal':goal,'status':'running','started_at':datetime.datetime.now().astimezone().isoformat(),'params_sha256':hashlib.sha256(PARAMS.read_bytes()).hexdigest(),'box_size':[.58,.50,.50],'z_center':.25,'record_bag':False,'spin_mode':options.spin_mode,'scenario':SCENE,'opponent_initial_distance_m':math.dist(start,opp),'opponent_direction':'toward_robot_-x','opponent_nominal_speed_mps':SCENE['speed']}
 write(record/'session.json',meta)
 children={};streams=[];probe=None;actor=None;navlog=None;rclpy_started=False
 env=os.environ.copy();env['RMUL_MOVING_CASE_ID']=str(ROOT)+':'+name
 def launch(k,cmd):
  stream=(record/(k+'.log')).open('w');streams.append(stream);p=subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,env=env,start_new_session=True);children[k]=p;return p
 def pump(seconds,guard=True):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   if probe is not None:
    rclpy.spin_once(probe,timeout_sec=.005);probe.sample()
    if guard and probe.fault is not None:raise RuntimeError(probe.fault['reason'])
   else:time.sleep(.02)
 def wait(p,seconds):
  end=time.monotonic()+seconds
  while p.poll() is None:
   pump(.02)
   if time.monotonic()>end:raise TimeoutError('Owned subprocess timed out')
  return p.returncode
 def actor_cmd(cmd):
  actor.stdin.write(cmd+'\n');actor.stdin.flush();deadline=time.monotonic()+12
  while time.monotonic()<deadline:
   if select.select([actor.stdout],[],[],0)[0]:
    row=json.loads(actor.stdout.readline());(record/'actor-events.jsonl').open('a').write(json.dumps({'command':cmd,**row})+'\n');return row
   pump(.02,False)
  raise TimeoutError('Moving actor did not reply: '+cmd)
 def nav(k,xy):return launch(k,[sys.executable,str(FOLDER/'verify_simulation.py'),'--mode','nav','--goal-x',str(xy[0]),'--goal-y',str(xy[1]),'--record-dir',str(record/k),'--goal-timeout','180','--startup-timeout','45','--stability-seconds','2']+(['--spin-mode'] if options.spin_mode else []))
 def cancel_goal():
  if probe is not None:
   probe.stop_pub.publish(Bool(data=True));probe.emit('test_safety_stop',reason='owned trial failure');pump(.08,False)
  p=record/'goal/goal-ownership.json'
  if probe is None or not p.exists():return
  owner=json.loads(p.read_text());client=probe.create_client(CancelGoal,'navigate_to_pose/_action/cancel_goal')
  if not client.wait_for_service(timeout_sec=1):return
  req=CancelGoal.Request();req.goal_info.goal_id.uuid=list(bytes.fromhex(owner['goal_uuid']));future=client.call_async(req);end=time.monotonic()+2
  while not future.done() and time.monotonic()<end:pump(.02,False)
  meta['cancelled_owned_goal']=owner['goal_uuid']
 try:
  STATE['current_case']=name;STATE['phase']='starting';write(ROOT/'status.json',STATE)
  wrapper=launch('launcher',['bash',str(options.launcher.resolve() if options.launcher else FOLDER/'start_rmul2027.sh'),'nav','--render-mode','gpu','--field-contact','cad-flat-floor','--gz-world-path',str(FOLDER/'config/gz_world_test.yaml'),'--params-file',str(PARAMS),'--tf2-library-dir',str(WS/'artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib')]);deadline=time.monotonic()+180
  while time.monotonic()<deadline:
   if wrapper.poll() is not None:raise RuntimeError('Launcher stopped before ready')
   for line in (record/'launcher.log').read_text(errors='replace').splitlines():
    if 'RMUL 2027 nav started. Logs:' in line:navlog=Path(line.split('Logs:',1)[1].strip())/'navigation.log'
   if navlog:break
   time.sleep(.2)
  if navlog is None:raise TimeoutError('Startup timeout')
  meta['navigation_log']=str(navlog);meta['resource_hashes']=json.loads((navlog.parent/'resource-hashes.json').read_text());meta['runtime_options']=json.loads((navlog.parent/'run-options.json').read_text());write(record/'session.json',meta)
  ready=launch('ready',[sys.executable,str(FOLDER/'verify_simulation.py'),'--mode','ready','--record-dir',str(record/'ready'),'--startup-timeout','90','--stability-seconds','2'])
  if wait(ready,120)!=0:raise RuntimeError('Readiness failed')
  rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static']);rclpy_started=True;probe=Probe(record);pump(2.)
  STATE['phase']='preposition';write(ROOT/'status.json',STATE)
  if wait(nav('preposition',start),210)!=0:raise RuntimeError('Preposition failed')
  pump(.5)
  wx,wy=opp[0]-4.8,opp[1]+3.0
  link=f"<link name='cart'><pose>0 0 .25 0 0 0</pose><collision name='cart_collision'><geometry><box><size>.58 .50 .50</size></box></geometry></collision><visual name='box_visual'><geometry><box><size>.58 .50 .50</size></box></geometry><material><ambient>1 .5 .05 1</ambient><diffuse>1 .5 .05 1</diffuse></material></visual><sensor name='cart_contact' type='contact'><always_on>true</always_on><update_rate>50</update_rate><topic>/c020/cart_contacts</topic><contact><collision>cart_collision</collision><topic>/c020/cart_contacts</topic></contact></sensor></link>"
  sdf="<sdf version='1.7'><model name='c020_opponent_cart'><static>true</static><pose>%s %s 0 0 0 0</pose>"%(wx,wy)+link+"</model></sdf>";(record/'obstacles.sdf').write_text(sdf)
  stream=(record/'actor-stderr.log').open('w');streams.append(stream);actor=subprocess.Popen([str(ROOT/'moving_actor'),str(record/'obstacles.sdf'),str(record)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stream,text=True,bufsize=1,env=env,start_new_session=True);children['actor']=actor
  deadline=time.monotonic()+20
  while not select.select([actor.stdout],[],[],0)[0]:
   pump(.05)
   if time.monotonic()>deadline:raise TimeoutError('Actor startup')
  created=json.loads(actor.stdout.readline());meta['created']=created
  if created.get('event')!='created':raise RuntimeError('Obstacle create failed')
  meta['opponent_pose_echo']=actor_cmd('POSE %s %s 0'%(opp[0],opp[1]))
  probe.targets=[opp];probe.phase='calibration';meta['positive_controls']=[];previous_probe_contacts=0
  for cmd in ['PROBE','PROBE']:
   result=actor_cmd(cmd);meta['positive_controls'].append(result)
   if not result.get('created') or not result.get('removed') or result.get('contact_messages',0)<=previous_probe_contacts:raise RuntimeError('Contact positive control failed')
   previous_probe_contacts=result['contact_messages']
  pump(.3)
  motion_check=actor_cmd('MOTIONPROBE');meta['motion_collision_positive_control']=motion_check
  if not motion_check.get('ok') or motion_check.get('contact_messages',0)<=0:raise RuntimeError('Moving collision consistency positive control failed')
  pump(.3)
  health=actor_cmd('HEALTH');meta['contacts_before_goal']=health
  if health.get('contacts',0):raise RuntimeError('Unexpected setup contact')
  probe.phase='fixed_observation';meta['observation_begin_sim']=probe.now();pump(5.);meta['observation_end_sim']=probe.now()
  meta['opponent_spawn_sim']=probe.now()
  STATE['phase']='goal';write(ROOT/'status.json',STATE);probe.phase='goal';meta['goal_started_wall']=time.time();write(record/'session.json',meta)
  result=nav('goal',goal);deadline=time.monotonic()+60
  ownership=record/'goal/goal-ownership.json'
  while not ownership.exists():
   if result.poll() is not None:raise RuntimeError('Goal client ended before accepting goal')
   if time.monotonic()>deadline:raise TimeoutError('Owned goal acceptance trigger unavailable')
   pump(.02)
  meta['goal_ownership_at_move']=json.loads(ownership.read_text())
  probe.opponent_move_started=probe.now();meta['opponent_move_started_sim']=probe.opponent_move_started
  meta['robot_actual_at_move_trigger']=list(probe.truth);meta['opponent_actual_at_move_trigger']=probe.opp_last
  meta['initial_distance_at_trigger_m']=math.dist(probe.truth[:2],probe.opp_last[1:]) if probe.opp_last else None
  move=actor_cmd('MOVE %s %s %s %s %s'%(opp[0],opp[1],opp_end[0],opp_end[1],SCENE['speed']));meta['opponent_move_command']=move
  if not move.get('ok'):raise RuntimeError('Opponent move command rejected')
  write(record/'session.json',meta)
  code=wait(result,210);probe.phase='after_goal';pump(2.);health=actor_cmd('HEALTH');meta['contact_health']=health;meta['goal_exit_code']=code
  status=actor_cmd('MOVESTATUS');meta['opponent_move_status']=status
  if code!=0:raise RuntimeError('Navigation acceptance failed')
  if health.get('contacts',0):raise RuntimeError('Contact monitor recorded collision')
  if status.get('active') or status.get('progress',0)<.999:raise RuntimeError('Opponent did not finish preset trajectory')
  if not probe.opp_last or math.dist(probe.opp_last[1:],opp_end)>.06:raise RuntimeError('Opponent actual endpoint mismatch')
  meta['stop_state']=stop_state(probe);meta['opponent_min_gap_m']=probe.min_opponent_gap if math.isfinite(probe.min_opponent_gap) else None
  meta['robot_stopped_near_opponent']=probe.stopped_near_opponent
  write(record/'session.json',meta)
  if probe.speed>=.03 or math.hypot(*probe.cmd[:2])>=.03:raise RuntimeError('Translation not physically stopped')
  duration=math.dist(opp,opp_end)/SCENE['speed'];t0=meta['opponent_move_started_sim']
  actual=[r for r in probe.opponents if t0+.2<r[0]<t0+duration-.2]
  if len(actual)<20:raise RuntimeError('Insufficient measured opponent motion samples')
  times=np.array([r[0] for r in actual]);xs=np.array([r[1] for r in actual]);ys=np.array([r[2] for r in actual]);vx=float(np.polyfit(times-times[0],xs,1)[0]);vy=float(np.polyfit(times-times[0],ys,1)[0])
  measured=math.hypot(vx,vy);meta['opponent_motion_regression_speed_mps']=measured;meta['opponent_motion_samples']=len(actual);meta['opponent_measured_velocity_mps']=[vx,vy]
  meta['robot_goal_speed_peak_mps']=max((r.get('truth_speed',0) for r in probe.motion_rows if r['phase']=='goal'),default=0)
  if abs(measured-SCENE['speed'])>.05:raise RuntimeError('Measured opponent speed differs from nominal by more than 0.05m/s')
  meta.update(status='passed',passed=True)
 except BaseException as e:
  meta.update(status='failed',passed=False,error=repr(e));(record/'exception.txt').write_text(traceback.format_exc())
  if probe:
   probe.fail('test_failure',error=repr(e));probe.phase='failure_stop'
   meta['stop_state_at_failure']=stop_state(probe)
   meta['opponent_min_gap_m']=probe.min_opponent_gap if math.isfinite(probe.min_opponent_gap) else None
   try:
    meta['opponent_stop_on_failure']=actor_cmd('STOPMOVE');meta['opponent_protective_stop']=True
    cancel_goal();pump(2.2,False);meta['speed_after_cancel']=probe.speed;meta['angular_after_cancel']=probe.angular;meta['cmd_after_cancel']=probe.cmd
   except Exception as e2:meta['cancel_error']=repr(e2)
  if actor and actor.poll() is None:
   try:
    meta['opponent_stop_on_failure']=actor_cmd('STOPMOVE')
    meta['opponent_protective_stop']=True
    meta['contact_health']=actor_cmd('HEALTH')
   except Exception as e2:meta['actor_health_error']=repr(e2)
 finally:
  if probe:
   meta['minimum_clearance_m']=probe.minimum if math.isfinite(probe.minimum) else None;meta['max_aligned_error_m']=probe.maxloc
   meta['opponent_min_gap_m']=probe.min_opponent_gap if math.isfinite(probe.min_opponent_gap) else None
   meta['opponent_measured_speed_mps']=probe.opp_speed;meta['opponent_log_read_retries']=probe.opponent_read_retries
   try:probe.finish()
   except Exception as e:meta['probe_finish_error']=repr(e)
  if rclpy_started:
   try:rclpy.shutdown()
   except Exception:pass
  for k in ['goal','preposition','ready']:
   p=children.get(k)
   if p and p.poll() is None:
    p.send_signal(signal.SIGINT)
    try:p.wait(timeout=8)
    except subprocess.TimeoutExpired:pass
  if actor and actor.poll() is None:
   try:actor.stdin.write('QUIT\n');actor.stdin.flush();actor.wait(timeout=12)
   except Exception as e:meta['actor_cleanup_error']=repr(e)
  p=children.get('launcher')
  if p and p.poll() is None:
   p.send_signal(signal.SIGINT)
   try:p.wait(timeout=45)
   except subprocess.TimeoutExpired:pass
  meta['process_cleanup']=cleanup(name)
  if navlog and navlog.exists():
   text=navlog.read_text(errors='replace');meta['backup_runs']=text.count('Running backup');meta['progress_failures']=text.count('Failed to make progress');meta['navigation_log_sha256']=hashlib.sha256(text.encode()).hexdigest()
   (record/'navigation-excerpts.txt').write_text('\n'.join(x for x in text.splitlines() if any(k in x.lower() for k in ['goal','progress','backup','collision','posefresh','fault','error'])))
  meta['ended_at']=datetime.datetime.now().astimezone().isoformat();write(record/'session.json',meta)
  for f in streams:f.close()
 return meta

def interrupted(sig,frame):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupted)
p0=json.loads(options.fixed_acceptance.read_text())
assert p0['passed'] and p0['cases']==12,'P0 acceptance is required before P1'
assert hashlib.sha256(PARAMS.read_bytes()).hexdigest()==p0['params_sha256'],'P1 must use the accepted P0 parameters'
for repeat in range(1,4):
 result=run_case(SCENE['label'],repeat)
 STATE['cases'].append(result);write(ROOT/'status.json',STATE)
 if result['status']!='passed' or result['process_cleanup']['remaining']:
  STATE['phase']='failed';write(ROOT/'status.json',STATE);raise SystemExit(1)
STATE['phase']='completed';write(ROOT/'status.json',STATE)
write(ROOT/'moving-acceptance.json',{'passed':True,'cases':3,'params_sha256':hashlib.sha256(PARAMS.read_bytes()).hexdigest(),'results':STATE['cases'],'scope':'one predefined head-on proxy, nominal translation 0.5m/s each'})
print('MOVING_ACCEPTANCE_DONE',flush=True)
