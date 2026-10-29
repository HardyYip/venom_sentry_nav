"""Dedicated executor for the IMU and its ROS source clock; no navigation work."""
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Imu
from imu import ImuWindow

class InputReader(Node):
    def __init__(self,topic):
        super().__init__('imu_recovery_input_reader',namespace='/red_standard_robot1')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.window=ImuWindow()
        self.subscription=self.create_subscription(Imu,topic,self.receive,
            QoSProfile(depth=400,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.runner=SingleThreadedExecutor();self.runner.add_node(self)
        self.thread=threading.Thread(target=self.runner.spin,name='recovery-input-reader')
        self.thread.start()
    def ros(self):return self.get_clock().now().nanoseconds*1e-9
    def receive(self,m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        values=(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,
                m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z)
        self.window.observe(stamp,self.ros(),time.monotonic(),values)
    def close(self):
        self.runner.remove_node(self);self.runner.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        if self.thread.is_alive():raise RuntimeError('owned input reader did not stop')
        self.destroy_node()
