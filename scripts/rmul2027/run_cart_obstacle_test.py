if __name__ == "__main__":
    from storage_entry import ensure_storage_guard
    ensure_storage_guard(__file__)

import argparse
import bisect
import collections
import datetime
import hashlib
import json
import math
import os
import select
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

parser = argparse.ArgumentParser()
parser.add_argument('--record-bag', action='store_true', help='Opt in to a light, guarded recording; default is no bag')
parser.add_argument('--record-dir', required=True, type=Path)
parser.add_argument('--spin-mode',action='store_true',help='Allow translation-only stop for successful spinning arrival; safety stops remain all-zero')
parser.add_argument('--scenario', required=True, choices=['control', 'parked', 'blocked'])
parser.add_argument('--params-file', required=True, type=Path)
parser.add_argument('--actor', required=True, type=Path)
parser.add_argument('--obstacle-model', type=Path)
parser.add_argument('--obstacle-size', nargs=3, type=float, default=[.58,.50,.50])
parser.add_argument('--record-pipeline', action='store_true')
parser.add_argument('--dds-profile', type=Path)
parser.add_argument('--fixed-observation-seconds',type=float)
args = parser.parse_args()
if args.dds_profile:
    args.dds_profile=args.dds_profile.resolve()
    if not args.dds_profile.is_file():raise SystemExit('Missing DDS profile: '+str(args.dds_profile))
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(args.dds_profile)

folder = Path(__file__).resolve().parent
workspace = folder.parents[3]
record = args.record_dir.resolve()
record.mkdir(parents=True, exist_ok=False)
meta = {'scenario': args.scenario, 'started_at': datetime.datetime.now().astimezone().isoformat(),
        'params_file': str(args.params_file.resolve()), 'params_sha256': hashlib.sha256(args.params_file.read_bytes()).hexdigest(),
        'cart_size_m': args.obstacle_size, 'obstacle_behavior': 'stationary geometry proxy, no opponent controller',
        'start_map_xy': [1.0, -5.5], 'goal_map_xy': [5.0, -5.5], 'cart_map_x': 3.0,
        'status': 'starting', 'processes': {}}
if args.dds_profile:
    meta['dds_profile']=str(args.dds_profile)
    meta['dds_profile_sha256']=hashlib.sha256(args.dds_profile.read_bytes()).hexdigest()
    shutil.copy2(args.dds_profile,record/'dds-profile.xml')
children, streams = {}, []
actor = None
monitor = None


def save():
    (record / 'session.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2))


def launch(name, command):
    stream = (record / (name + '-console.log')).open('w')
    streams.append(stream)
    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    children[name] = process
    meta['processes'][name] = {'pid': process.pid, 'command': command}
    save()
    return process


def event(name, **values):
    row = {'event': name, 'wall': time.time(), **values}
    with (record / 'events.jsonl').open('a') as stream:
        stream.write(json.dumps(row) + '\n')
    print(json.dumps(row), flush=True)


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def polygon(x, y, angle, hx=.29, hy=.25):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[x+c*a-s*b, y+s*a+c*b] for a, b in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]])


def clearance(first, second):
    separated, penetration = False, float('inf')
    for points in [first, second]:
        for i in range(2):
            edge = points[i+1] - points[i]
            axis = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
            a, b = first @ axis, second @ axis
            gap = max(b.min()-a.max(), a.min()-b.max())
            separated |= gap > 0
            penetration = min(penetration, min(a.max(),b.max())-max(a.min(),b.min()))
    if not separated:
        return -max(penetration, 0.0)
    answer = float('inf')
    for p, q in [(first, second), (second, first)]:
        for point in p:
            for a, b in zip(q, np.roll(q, -1, axis=0)):
                delta = b-a
                t = np.clip(np.dot(point-a, delta)/np.dot(delta, delta), 0, 1)
                answer = min(answer, float(np.linalg.norm(point-(a+t*delta))))
    return answer


class Monitor(Node):
    def __init__(self):
        super().__init__('codex_cart_observer', namespace='/red_standard_robot1')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.sim, self.truth, self.cart = 0.0, None, None
        self.last_sample, self.last_cloud = -1.0, -1.0
        self.grids, self.plan, self.truth_history = {}, [], collections.deque(maxlen=30000)
        self.cloud_hits, self.max_cloud_hits = 0, 0
        self.last_cloud_by_source={};self.cloud_hits_by_source={};self.max_hits_by_source={}
        self.min_clearance, self.max_deviation = float('inf'), 0.0
        self.detected, self.phase = False, 'preparing'
        self.first_detection, self.yield_samples = None, 0
        self.path_changes, self.commands = [], []
        self.data = (record / 'samples.jsonl').open('w')
        sensor = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = [self.create_subscription(Clock, '/clock', self.clock, sensor),
                     self.create_subscription(Odometry, 'chassis_odometry_gt', self.pose, sensor),
                     self.create_subscription(Twist, 'cmd_vel', self.command, 10),
                     self.create_subscription(NavPath, 'plan', self.path, 10),
                     self.create_subscription(PointCloud2,'terrain_map',lambda msg:self.cloud(msg,'terrain_map'),sensor),
                     self.create_subscription(PointCloud2,'rplidar_obstacles',lambda msg:self.cloud(msg,'rplidar_obstacles'),sensor)]
        for key in ['local_costmap', 'global_costmap']:
            self.subs.append(self.create_subscription(OccupancyGrid, key+'/costmap', lambda msg,k=key:self.grid(k,msg), latched))
            self.subs.append(self.create_subscription(OccupancyGridUpdate, key+'/costmap_updates', lambda msg,k=key:self.update(k,msg), 10))
        self.static = None
        self.subs.append(self.create_subscription(OccupancyGrid, 'map', lambda msg:setattr(self,'static',msg), latched))

    def clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def pose(self, msg):
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.truth = [p.x+4.8, p.y-3.0, yaw(msg.pose.pose.orientation)]
        self.truth_history.append((stamp, p.x+4.8, p.y-3.0))

    def command(self, msg):
        self.commands.append({'sim':self.sim,'value':[msg.linear.x,msg.linear.y,msg.angular.z]})
        if self.phase == 'goal' and self.cart and self.truth and self.truth[0] < 4.0 and math.dist(self.truth[:2], self.cart) < 1.6 and math.hypot(msg.linear.x,msg.linear.y) < .15:
            self.yield_samples += 1

    def path(self, msg):
        self.plan = [[p.pose.position.x,p.pose.position.y] for p in msg.poses]
        if self.phase == 'goal':
            self.path_changes.append({'sim':self.sim,'points':self.plan})

    def grid(self, key, msg):
        self.grids[key] = {'frame':msg.header.frame_id,'origin':[msg.info.origin.position.x,msg.info.origin.position.y],
                           'resolution':msg.info.resolution,'data':np.array(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width),'stamp':self.sim}

    def update(self, key, msg):
        if key not in self.grids:return
        grid = self.grids[key]
        if msg.x+msg.width > grid['data'].shape[1] or msg.y+msg.height > grid['data'].shape[0]:return
        grid['data'][msg.y:msg.y+msg.height,msg.x:msg.x+msg.width] = np.array(msg.data,dtype=np.int16).reshape(msg.height,msg.width)
        grid['stamp'] = self.sim

    def transform_xy(self, x, y, target, source):
        if target == source:return x,y
        t = self.buffer.lookup_transform(target,source,rclpy.time.Time()).transform
        a = yaw(t.rotation);c,s=math.cos(a),math.sin(a)
        return c*x-s*y+t.translation.x,s*x+c*y+t.translation.y

    def occupancy(self, key, point):
        if key not in self.grids:return None
        grid = self.grids[key]
        try:x,y=self.transform_xy(*point,grid['frame'],'map')
        except Exception:return None
        col=int(math.floor((x-grid['origin'][0])/grid['resolution']));row=int(math.floor((y-grid['origin'][1])/grid['resolution']))
        values=[]
        for iy in range(row-3,row+4):
            for ix in range(col-3,col+4):
                if 0<=iy<grid['data'].shape[0] and 0<=ix<grid['data'].shape[1]:values.append(int(grid['data'][iy,ix]))
        return max(values) if values else None

    def cloud(self, msg, source):
        if not self.cart or self.sim-self.last_cloud_by_source.get(source,-1)<.2:return
        self.last_cloud_by_source[source]=self.sim
        fields={f.name:f for f in msg.fields}
        if not all(k in fields for k in ['x','y','intensity']):return
        count=msg.width*msg.height
        if not count:return
        try:
            cx,cy=self.transform_xy(*self.cart,msg.header.frame_id,'map')
            arrays={k:np.ndarray((count,),dtype='<f4',buffer=msg.data,offset=fields[k].offset,strides=(msg.point_step,)) for k in ['x','y','intensity']}
            mask=(np.abs(arrays['x']-cx)<.40)&(np.abs(arrays['y']-cy)<.36)&(arrays['intensity']>.1)
            self.cloud_hits_by_source[source]=(self.sim,int(np.count_nonzero(mask)))
            self.cloud_hits=max((hits for at,hits in self.cloud_hits_by_source.values() if self.sim-at<.5),default=0)
            self.max_cloud_hits=max(self.max_cloud_hits,self.cloud_hits)
            self.max_hits_by_source[source]=max(self.max_hits_by_source.get(source,0),int(np.count_nonzero(mask)))
        except Exception:return

    def sample(self):
        if self.sim-self.last_sample < .05 or not self.truth:return
        self.last_sample=self.sim
        value={'sim':self.sim,'truth_map':self.truth,'cart_map':self.cart,'phase':self.phase,'cloud_cart_hits':self.cloud_hits,'cloud_hits_by_source':self.cloud_hits_by_source}
        if self.phase == 'goal':self.max_deviation=max(self.max_deviation,abs(self.truth[1]+5.5))
        if self.cart:
            gap=clearance(polygon(*self.truth),polygon(*self.cart,0,hx=args.obstacle_size[0]/2,hy=args.obstacle_size[1]/2))
            self.min_clearance=min(self.min_clearance,gap);value['footprint_clearance_m']=gap
            value['costs']={k:self.occupancy(k,self.cart) for k in self.grids}
            if len(value['costs'])==2 and all(v is not None and v>=99 for v in value['costs'].values()) and self.cloud_hits>0:
                self.detected=True
                if self.first_detection is None:self.first_detection=self.sim
            if self.phase=='goal' and gap<0:
                (record/'guard-trigger.json').write_text(json.dumps(value,indent=2))
                raise RuntimeError('Footprints overlap: '+str(gap))
        try:
            t=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
            stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
            if self.truth_history and self.truth_history[0][0]<=stamp<=self.truth_history[-1][0]:
                history=list(self.truth_history);i=min(len(history)-1,max(1,bisect.bisect_right(history,(stamp,float('inf'),float('inf')))));a,b=history[i-1],history[i];f=(stamp-a[0])/max(b[0]-a[0],1e-9)
                p=t.transform.translation;error=math.hypot(p.x-(a[1]+f*(b[1]-a[1])),p.y-(a[2]+f*(b[2]-a[2])))
                value['aligned_localization_error_m']=error
                if self.phase=='goal' and error>.35:
                    value.update(estimate_map=[p.x,p.y],estimate_stamp=stamp,truth_before=a,truth_after=b,weight=f)
                    (record/'guard-trigger.json').write_text(json.dumps(value,indent=2))
                    raise RuntimeError('Aligned localization error exceeded .35m: '+str(error))
        except RuntimeError:raise
        except Exception:pass
        self.data.write(json.dumps(value)+'\n');self.data.flush()

    def snapshot(self, name):
        value={k:{**{a:b for a,b in v.items() if a!='data'},'data':v['data'].tolist()} for k,v in self.grids.items()}
        (record/(name+'-costmaps.json')).write_text(json.dumps(value))


def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(monitor,timeout_sec=.02)
        monitor.sample()


def wait_child(process, timeout=220):
    start=time.monotonic()
    while process.poll() is None:
        pump(.03)
        if time.monotonic()-start>timeout:raise TimeoutError('Owned navigation test deadline')
    return process.returncode


def send_actor(command):
    actor.stdin.write(command+'\n');actor.stdin.flush()
    if not select.select([actor.stdout],[],[],12)[0]:raise TimeoutError('Cart actor command timed out')
    line=actor.stdout.readline();value=json.loads(line)
    with (record/'actor-events.jsonl').open('a') as stream:stream.write(line)
    if value.get('ok') is False:raise RuntimeError('Cart pose request failed')
    return value


def navigation(name, x, y, scenario='simple'):
    return launch(name,[sys.executable,str(folder/'verify_simulation.py'),'--mode','nav','--goal-x',str(x),'--goal-y',str(y),
                        '--scenario',scenario,'--startup-timeout','45','--goal-timeout','180','--stability-seconds','2','--record-dir',str(record/name)]+(['--spin-mode'] if args.spin_mode else []))


def interrupt(signum,frame):raise KeyboardInterrupt


signal.signal(signal.SIGTERM,interrupt)
try:
    command=['bash',str(folder/'start_rmul2027.sh'),'nav','--params-file',str(args.params_file.resolve()),
             '--gz-world-path',str(folder/'config/gz_world_test.yaml'),'--field-contact','cad-flat-floor','--render-mode','gpu',
             '--tf2-library-dir',str(workspace/'artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib')]
    wrapper=launch('launcher',command)
    end=time.monotonic()+180;navlog=None
    while time.monotonic()<end:
        if wrapper.poll() is not None:raise RuntimeError('Simulation startup failed')
        for line in (record/'launcher-console.log').read_text().splitlines():
            if 'RMUL 2027 nav started. Logs: ' in line:navlog=Path(line.split('Logs: ',1)[1].strip())/'navigation.log'
        if navlog:break
        time.sleep(.2)
    if not navlog:raise TimeoutError('Simulation startup')
    meta['navigation_log']=str(navlog);save()
    for name in ['world.sdf','robot-red_standard_robot1.sdf','nav2_params.yaml','gz_world.yaml','resource-hashes.json','tf2-runtime.json','rendering-runtime.json']:
        source=navlog.parent/name
        if source.exists():shutil.copy2(source,record/name)
    topics=['/clock','/red_standard_robot1/tf','/red_standard_robot1/tf_static','/red_standard_robot1/safety_status','/red_standard_robot1/chassis_odometry_gt',
            '/red_standard_robot1/velodyne_points','/red_standard_robot1/livox/imu','/red_standard_robot1/registered_scan',
            '/red_standard_robot1/terrain_map','/red_standard_robot1/terrain_map_ext','/red_standard_robot1/plan',
            '/red_standard_robot1/local_plan','/red_standard_robot1/cmd_vel','/red_standard_robot1/map',
            '/red_standard_robot1/local_costmap/costmap','/red_standard_robot1/local_costmap/costmap_updates',
            '/red_standard_robot1/global_costmap/costmap','/red_standard_robot1/global_costmap/costmap_updates',
            '/red_standard_robot1/navigate_to_pose/_action/status','/red_standard_robot1/navigate_to_pose/_action/feedback']
    if args.record_pipeline:
        topics += ['/red_standard_robot1/'+name for name in ['livox/lidar','cloud_registered','aft_mapped_to_init','lidar_odometry','odometry','cmd_vel_controller','cmd_vel_nav2_result']]
    topics=[topic for topic in topics if not any(word in topic for word in ('velodyne_points','cloud_registered','livox/lidar','registered_scan','terrain_map'))]
    meta['record_bag']=args.record_bag
    meta['recorded_topics']=topics if args.record_bag else [];save()
    if args.record_bag:
        launch('recorder',['ros2','bag','record','--include-hidden-topics','--max-cache-size','67108864','--max-bag-size','33554432','-o',str(record/'inputs'),*topics])
    check=launch('ready',[sys.executable,str(folder/'run_baseline_audit.py'),'--record-dir',str(record/'ready'),'--last-case','ready','--navigation-log',str(navlog)])
    if check.wait(timeout=180)!=0:raise RuntimeError('Ready verification failed')
    ready_meta=json.loads((record/'ready/run.json').read_text());oid=ready_meta.get('observer_pid')
    if oid and Path('/proc',str(oid),'cmdline').exists():os.kill(oid,signal.SIGINT)
    rclpy.init(args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'])
    monitor=Monitor();pump(2)
    if wait_child(navigation('preposition',1,-5.5))!=0:raise RuntimeError('Preposition goal failed')
    pump(1);monitor.snapshot('before-cart')
    actor=subprocess.Popen([str(args.actor.resolve()),str((args.obstacle_model or folder/'assets/test_cart.sdf').resolve()),str(record)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(record/'actor-stderr.log').open('w'),text=True,bufsize=1)
    meta['actor_pid']=actor.pid;save()
    if not select.select([actor.stdout],[],[],15)[0]:raise TimeoutError('Cart create timeout')
    created=json.loads(actor.stdout.readline());event('cart_actor',response=created)
    if created.get('event')!='created':raise RuntimeError('Cart creation failed')
    send_actor('POSE 20 20 0')
    probe=send_actor('PROBE');event('contact_positive_control',response=probe)
    if probe.get('contact_messages',0)<1 or not probe.get('removed'):raise RuntimeError('Contact instrumentation positive control failed')
    if probe.get('other_contacts',0):raise RuntimeError('Unexpected contact during instrumentation setup')
    if monitor.static is None:raise RuntimeError('Missing static prior')
    prior=monitor.static;ix=int((3.0-prior.info.origin.position.x)/prior.info.resolution);iy=int((-5.5-prior.info.origin.position.y)/prior.info.resolution)
    prior_value=int(prior.data[iy*prior.info.width+ix]);meta['cart_location_static_prior']=prior_value
    if prior_value!=0:raise RuntimeError('Obstacle location is not absent from static map')
    if args.scenario!='control':
        cy=-5.5
        send_actor('POSE 3.0 '+str(cy)+' 0');monitor.cart=[3.0,cy];event('cart_inserted',map_xy=monitor.cart,static_prior=prior_value)
        if args.fixed_observation_seconds is None:
            end=monitor.sim+12;wall_end=time.monotonic()+35
            while not monitor.detected and monitor.sim<end and time.monotonic()<wall_end:pump(.1)
            monitor.snapshot('cart-detected')
            if not monitor.detected:raise RuntimeError('Cart did not enter sensed pointcloud and costmap')
        else:
            if args.fixed_observation_seconds<0:raise ValueError('Observation duration must not be negative')
            end=monitor.sim+args.fixed_observation_seconds;wall_end=time.monotonic()+35
            while monitor.sim<end and time.monotonic()<wall_end:pump(.1)
            meta['fixed_observation_seconds']=args.fixed_observation_seconds
            meta['detected_before_goal']=monitor.detected
            monitor.snapshot('after-fixed-observation')
    monitor.phase='goal';meta['status']='testing-goal';save()
    code=wait_child(navigation('goal',5,-5.5,'blocked_route' if args.scenario=='blocked' else 'simple'))
    monitor.phase='after-goal';monitor.snapshot('after-goal')
    health=send_actor('HEALTH');meta['contact_health']=health
    meta.update(navigation_exit_code=code,obstacle_detected=monitor.detected,max_cloud_cart_hits=monitor.max_cloud_hits,
                min_footprint_clearance_m=monitor.min_clearance if math.isfinite(monitor.min_clearance) else None,
                max_route_y_deviation_m=monitor.max_deviation,yield_samples=monitor.yield_samples)
    if code!=0:raise RuntimeError('Navigation goal failed its original acceptance checks')
    meta['max_hits_by_source']=monitor.max_hits_by_source
    if args.scenario!='control' and not monitor.detected:raise RuntimeError('Obstacle never verified by actual sensor and both costmaps')
    if health.get('contacts',0)>0:raise RuntimeError('Contact sensor recorded navigation collision')
    if args.scenario!='control':
        old=list(monitor.cart);send_actor('POSE 20 20 0');monitor.cart=None
        until=monitor.sim+8;wall_end=time.monotonic()+30;cleared=False
        while monitor.sim<until and time.monotonic()<wall_end:
            pump(.1);costs={k:monitor.occupancy(k,old) for k in ['local_costmap','global_costmap']}
            if all(v is not None and v<99 for v in costs.values()):cleared=True;break
        meta.update(obstacle_cleared=cleared,cleared_location_map_xy=old,cleared_costs=costs);monitor.snapshot('after-removal')
        if not cleared:raise RuntimeError('Removed cart remains lethal in costmap')
    if args.scenario=='blocked':
        monitor.phase='goal';meta['status']='testing-cleared-route';save()
        code=wait_child(navigation('after-clear-goal',5,-5.5))
        monitor.phase='after-goal';monitor.snapshot('after-clear-goal')
        meta['after_clear_navigation_exit_code']=code
        meta['after_clear_contact_health']=send_actor('HEALTH')
        if code!=0:raise RuntimeError('New goal after obstacle removal failed')
        if meta['after_clear_contact_health'].get('contacts',0)>0:raise RuntimeError('Contact after barrier removal')
    meta.update(status='passed',passed=True)
except BaseException as error:
    meta.update(status='failed',passed=False,error=repr(error));event('FAIL',error=repr(error))
    if monitor:
        monitor.phase='stopping'
        goal_process=children.get('goal');ownership=record/'goal/goal-ownership.json'
        if goal_process and goal_process.poll() is None and ownership.exists():
            try:
                owner=json.loads(ownership.read_text());client=monitor.create_client(CancelGoal,'navigate_to_pose/_action/cancel_goal')
                if client.wait_for_service(timeout_sec=2):
                    request=CancelGoal.Request();request.goal_info.goal_id.uuid=list(bytes.fromhex(owner['goal_uuid']))
                    future=client.call_async(request);rclpy.spin_until_future_complete(monitor,future,timeout_sec=2)
                    response=future.result() if future.done() else None
                    meta['owned_goal_cancel']={'goal_uuid':owner['goal_uuid'],'return_code':response.return_code if response else None,
                                               'canceling':[bytes(v.goal_id.uuid).hex() for v in response.goals_canceling] if response else []}
                    event('owned_goal_cancel',**meta['owned_goal_cancel'])
                until=time.monotonic()+8
                while goal_process.poll() is None and time.monotonic()<until:pump(.05)
            except Exception as cancel_error:meta['owned_goal_cancel_error']=repr(cancel_error)
        meta.update(obstacle_detected=monitor.detected,max_cloud_cart_hits=monitor.max_cloud_hits,
                    min_footprint_clearance_m=monitor.min_clearance if math.isfinite(monitor.min_clearance) else None)
        try:monitor.snapshot('failure')
        except Exception:pass
    try:
        p=subprocess.run(['ign','model','-m','red_standard_robot1','-p'],capture_output=True,text=True,timeout=8)
        (record/'failure-model-pose.txt').write_text(p.stdout+p.stderr)
    except Exception:pass
finally:
    save()
    for name in ['after-clear-goal','goal','preposition','ready']:
        process=children.get(name)
        if process and process.poll() is None:
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:meta.setdefault('cleanup_pending',[]).append(name)
    if actor and actor.poll() is None:
        try:send_actor('HEALTH');actor.stdin.write('QUIT\n');actor.stdin.flush();actor.wait(timeout=12)
        except Exception as error:meta['actor_cleanup_error']=str(error)
    if monitor:
        monitor.data.close();(record/'plans.json').write_text(json.dumps(monitor.path_changes));(record/'commands.json').write_text(json.dumps(monitor.commands));monitor.destroy_node();rclpy.shutdown()
    for name,timeout in [('recorder',25),('launcher',45)]:
        process=children.get(name)
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:meta.setdefault('cleanup_pending',[]).append(name)
    if (record/'ready/run.json').exists():
        oid=json.loads((record/'ready/run.json').read_text()).get('observer_pid');path=Path('/proc')/str(oid)/'cmdline'
        if path.exists() and str(record/'ready/observer').encode() in path.read_bytes():os.kill(oid,signal.SIGINT)
    meta['finished_at']=datetime.datetime.now().astimezone().isoformat();meta['bag_metadata_exists']=(record/'inputs/metadata.yaml').exists();save()
    for stream in streams:stream.close()
    event('finished',summary=meta)
sys.exit(0 if meta.get('passed') and not meta.get('cleanup_pending') else 1)
