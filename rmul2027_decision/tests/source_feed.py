import json
import time
import threading
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.msg import Bool
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rmul2027_decision.msg import MatchState

class Feed(Node):
    def __init__(self, namespace, synthetic=True):
        super().__init__('decision_test_feed',namespace=namespace)
        self.synthetic=synthetic
        self.phase=0;self.hp=400.;self.ammo=100.;self.heat=0.
        self.estop=False;self.trust=True;self.sequence=0
        self.match_on=True;self.estop_on=True;self.trust_on=True;self.odom_on=True;self.tf_on=True
        self.freeze_stamp=False;self.replay=False;self.valid=True;self.source_offset=0.;self.fixed_stamp=None
        self.events=[];self.status=[];self.stops=[];self.last_pub=0
        self.match_pub=self.create_publisher(MatchState,'decision/match',10)
        self.estop_pub=self.create_publisher(Bool,'decision/estop',1)
        self.trust_pub=self.create_publisher(Bool,'decision/localization_trusted',1)
        self.subs=[self.create_subscription(String,'decision/events',lambda m:self.events.append(json.loads(m.data)),100),
                   self.create_subscription(String,'decision/status',lambda m:self.status.append(json.loads(m.data)),100),
                   self.create_subscription(Bool,'decision_stop',lambda m:self.stops.append((time.monotonic(),m.data)),100)]
        if synthetic:
            self.odom_pub=self.create_publisher(Odometry,'odometry',10)
            self.tf_pub=self.create_publisher(TFMessage,'tf',10)
        self.runner=MultiThreadedExecutor(num_threads=6);self.runner.add_node(self)
        self.child=None
        self.feed_timer=self.create_timer(.03,self.publish_inputs,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.spin_thread=threading.Thread(target=self.runner.spin,daemon=True);self.spin_thread.start()
    def publish_inputs(self):
        stamp=self.get_clock().now().to_msg()
        if self.source_offset:
            ns=stamp.sec*10**9+stamp.nanosec+int(self.source_offset*1e9);stamp.sec,stamp.nanosec=divmod(ns,10**9)
        if self.fixed_stamp is None:self.fixed_stamp=stamp
        if self.match_on:
            if not self.replay:self.sequence+=1
            m=MatchState();m.stamp=stamp;m.sequence=self.sequence;m.phase=self.phase
            m.hp=self.hp;m.ammo=self.ammo;m.heat=self.heat;m.valid=self.valid;m.source='TEST fixture, not 2027 protocol'
            self.match_pub.publish(m)
        if self.estop_on:self.estop_pub.publish(Bool(data=self.estop))
        if self.trust_on:self.trust_pub.publish(Bool(data=self.trust))
        if self.synthetic:
            locstamp=self.fixed_stamp if self.freeze_stamp else stamp
            if self.odom_on:
                o=Odometry();o.header.stamp=locstamp;o.header.frame_id='odom';o.child_frame_id='base_footprint';o.pose.pose.orientation.w=1.
                self.odom_pub.publish(o)
            if self.tf_on:
                tf=TransformStamped();tf.header.stamp=locstamp;tf.header.frame_id='map';tf.child_frame_id='base_footprint'
                tf.transform.rotation.w=1.;self.tf_pub.publish(TFMessage(transforms=[tf]))
    def pump(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            time.sleep(.005)
            if self.child is not None and self.child.poll() is not None:raise RuntimeError('Decision child exited')
    def wait(self,predicate,timeout=6):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:self.pump(.02)
        assert predicate(),'Condition timeout; latest='+repr(self.status[-1:] )
    def event(self,kind):
        return [x for x in self.events if x['kind']==kind]
    def cleanup(self):
        self.feed_timer.cancel();self.runner.remove_node(self);self.runner.shutdown(timeout_sec=3);self.spin_thread.join(timeout=3);self.destroy_node()
