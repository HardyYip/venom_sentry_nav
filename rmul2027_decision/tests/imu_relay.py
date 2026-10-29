"""TEST-only transparent IMU relay, isolated from the observation/feed callbacks."""
import threading,time
from collections import deque
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Imu

class ImuRelay(Node):
    def __init__(self,namespace):
        super().__init__('test_imu_relay',namespace=namespace)
        self.lock=threading.Lock();self._enabled=True
        self.stats=dict(raw_messages=0,forwarded_messages=0,max_publish_wall_s=0.,max_raw_stamp_gap_s=0.)
        self.gaps=deque(maxlen=16);self.last_stamp=None;self.last_at=None
        qos=QoSProfile(depth=400,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output=self.create_publisher(Imu,'recovery/test_imu',qos)
        self.input=self.create_subscription(Imu,'livox/imu',self.forward,qos)
        self.runner=SingleThreadedExecutor();self.runner.add_node(self)
        self.thread=threading.Thread(target=self.runner.spin,name='test-imu-relay');self.thread.start()
    @property
    def enabled(self):
        with self.lock:return self._enabled
    @enabled.setter
    def enabled(self,value):
        with self.lock:self._enabled=bool(value)
    def forward(self,message):
        t=time.monotonic();stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        with self.lock:
            self.stats['raw_messages']+=1
            if self.last_stamp is not None:
                delta=stamp-self.last_stamp
                self.stats['max_raw_stamp_gap_s']=max(self.stats['max_raw_stamp_gap_s'],delta)
                if delta>.09 or t-self.last_at>.1:
                    self.gaps.append(dict(steady=t,stamp=stamp,source_gap_s=delta,receipt_gap_s=t-self.last_at))
            self.last_stamp=stamp;self.last_at=t
            if self._enabled:
                started=time.monotonic();self.output.publish(message)
                self.stats['max_publish_wall_s']=max(self.stats['max_publish_wall_s'],time.monotonic()-started)
                self.stats['forwarded_messages']+=1
    def snapshot(self):
        with self.lock:return dict(**self.stats,gaps=list(self.gaps))
    def close(self):
        self.runner.remove_node(self);self.runner.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        if self.thread.is_alive():raise RuntimeError('owned test relay did not stop')
        self.destroy_node()
