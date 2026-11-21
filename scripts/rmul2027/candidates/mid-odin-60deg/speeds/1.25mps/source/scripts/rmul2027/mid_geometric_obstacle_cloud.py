"""C075 candidate MID geometric obstacle side branch; source cloud and localization untouched."""
import collections,json,math,os,time
import numpy as np
from aux_mid_obstacle_cloud import xyz_from_cloud,rotation_matrix

def geometric_points(points,rotation,translation,*,self_radius,min_height=.06,max_height=2.,min_range=.2,max_range=5.,voxel_size=.025):
    p=np.asarray(points,dtype=float).reshape((-1,3));r=np.asarray(rotation,dtype=float);t=np.asarray(translation,dtype=float)
    if r.shape!=(3,3) or t.shape!=(3,) or not np.isfinite(r).all() or not np.isfinite(t).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6:
        raise ValueError('invalid source-time rigid transform')
    limits=[self_radius,min_height,max_height,min_range,max_range,voxel_size]
    if not all(math.isfinite(v)for v in limits) or not(0<self_radius<.5 and 0<min_height<max_height and 0<=min_range<max_range and 0<voxel_size<=.05):
        raise ValueError('invalid geometric filter bounds')
    total=len(p);p=p[np.isfinite(p).all(axis=1)]
    d=np.linalg.norm(p,axis=1);keep=(d>=min_range)&(d<=max_range);p=p[keep]
    base=p@r.T+t;horizontal=np.linalg.norm(base[:,:2],axis=1)
    keep=(base[:,2]>=min_height)&(base[:,2]<=max_height)&(horizontal>self_radius)
    p=p[keep];base=base[keep];horizontal=horizontal[keep]
    # Keep an actual nearest return in each small horizontal cell.
    # Do not bin on tilted sensor azimuth: it can merge different ground bearings.
    cells=np.floor(base[:,:2]/voxel_size).astype(np.int64)
    order=np.argsort(horizontal,kind='stable')
    _,first=np.unique(cells[order],axis=0,return_index=True)
    selected=order[first];p=p[selected]
    return p,dict(input_points=total,height_and_self_filtered_points=len(base),output_points=len(p))

class SourcePairHealth:
    """A fresh converted stamp must be an exact member of received raw source stamps."""
    def __init__(self):
        self.raw=collections.deque(maxlen=16);self.converted=collections.deque(maxlen=16)
    def add(self,kind,stamp,received):
        if kind not in ('raw','converted') or not math.isfinite(stamp) or stamp<=0 or not math.isfinite(received):return False
        values=getattr(self,kind)
        if values and stamp<=values[-1][0]:return False
        values.append((stamp,received));return True
    def healthy(self,ros,wall,*,max_wall_age=.25):
        if not math.isfinite(ros) or not math.isfinite(wall):return False
        raw={s for s,w in self.raw if -.1<=ros-s<=.25 and 0<=wall-w<=max_wall_age}
        return any(s in raw and 0<=wall-w<=max_wall_age and -.1<=ros-s<=.25 for s,w in self.converted)

def main():
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2,PointField
    from std_msgs.msg import Header
    from tf2_ros import Buffer,TransformListener
    from source_time_obstacle_queue import SourceTimeObstacleQueue
    class MidGeometry(Node):
        def __init__(self):
            super().__init__('mid_geometric_obstacle_cloud')
            self.source=self.declare_parameter('source_frame','front_mid360_scan').value
            self.target=self.declare_parameter('filter_frame','base_footprint').value
            self.limits={k:self.declare_parameter(k,v).value for k,v in dict(self_radius=.434978334,min_height=.06,max_height=2.,min_range=.2,max_range=5.,voxel_size=.025).items()}
            geometric_points([],np.eye(3),np.zeros(3),**self.limits)
            self.tf=Buffer(cache_time=Duration(seconds=10),node=self);self.listener=TransformListener(self.tf,self)
            self.pub=self.create_publisher(PointCloud2,'mid/geometric_obstacles',qos_profile_sensor_data)
            self.source_header_pub=(self.create_publisher(Header,'mid/geometric_raw_header',qos_profile_sensor_data)
                                    if os.environ.get('RMUL_MID_GEOMETRY_SOURCE_HEADER')=='1' else None)
            self.counts=collections.Counter()
            self.queue=SourceTimeObstacleQueue(self.source,self.target,self.get_clock(),
                                               self.tf,self.convert,self.reject,self.counts)
            self.sub=self.create_subscription(PointCloud2,'livox/lidar',self.queue.receive,qos_profile_sensor_data)
            self.create_timer(.01,self.queue.drain,clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.create_timer(5.,lambda:self.get_logger().info(json.dumps(dict(self.counts))))
        def reject(self,reason):
            self.counts['dropped']+=1
            self.get_logger().warning('MID geometry source dropped: '+str(reason),throttle_duration_sec=5.)
        def convert(self,m,tf,waited):
            try:
                xyz=xyz_from_cloud(m)
                if not len(xyz):raise ValueError('empty raw MID cloud')
                p,c=geometric_points(xyz,rotation_matrix(tf.rotation),[tf.translation.x,tf.translation.y,tf.translation.z],**self.limits)
                cloud=PointCloud2();cloud.header=m.header;cloud.height=1;cloud.width=len(p)
                cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)for i,n in enumerate(('x','y','z','intensity'))]
                cloud.is_bigendian=False;cloud.point_step=16;cloud.row_step=16*len(p);cloud.is_dense=True
                cloud.data=np.column_stack((p,np.ones(len(p)))).astype('<f4').tobytes()
                if self.source_header_pub is not None:self.source_header_pub.publish(m.header)
                self.pub.publish(cloud);self.counts['published']+=1;self.counts.update(c)
                if waited:self.counts['published_after_source_tf']+=1
            except Exception as e:
                self.reject(e)
    # Finish the single-threaded callback before invalidating its ROS context.
    # Default rclpy signal handling can close the context inside take_message.
    import signal
    from rclpy.signals import SignalHandlerOptions
    stopping=False
    def request_stop(signum,frame):
        nonlocal stopping
        stopping=True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous={sig:signal.signal(sig,request_stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    n=None
    try:
        n=MidGeometry()
        while not stopping and rclpy.ok():rclpy.spin_once(n,timeout_sec=.1)
    finally:
        if n is not None:n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)
if __name__=='__main__':main()
