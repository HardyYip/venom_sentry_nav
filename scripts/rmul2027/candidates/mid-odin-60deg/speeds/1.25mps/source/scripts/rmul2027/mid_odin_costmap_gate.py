"""Source-health heartbeat for the isolated C072-style navigation profile."""
import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from example_interfaces.msg import Bool
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from c075_ros_lifecycle import init_context, shutdown_context, spin_context
from mid_geometric_obstacle_cloud import SourcePairHealth


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def valid_cloud(msg, frame, raw):
    if msg.header.frame_id != frame or not msg.height or not msg.point_step:
        return False
    if msg.row_step < msg.width * msg.point_step or len(msg.data) != msg.row_step * msg.height:
        return False
    fields = {field.name: field for field in msg.fields}
    if raw:
        return msg.width > 0 and all(name in fields and fields[name].count == 1 and
                                     fields[name].datatype in (PointField.FLOAT32, PointField.FLOAT64)
                                     for name in ('x', 'y', 'z'))
    return msg.height == 1 and msg.point_step == 16 and msg.row_step == 16 * msg.width and all(
        name in fields and fields[name].count == 1 and fields[name].datatype == PointField.FLOAT32
        and fields[name].offset == 4 * index
        for index, name in enumerate(('x', 'y', 'z', 'intensity')))


class CostmapGate(Node):
    def __init__(self):
        super().__init__('mid_odin_view', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.mid = SourcePairHealth()
        self.odin = SourcePairHealth()
        source_header = os.environ.get('RMUL_MID_GEOMETRY_SOURCE_HEADER') == '1'
        self.primary_imu = None
        self.last_ros = 0.
        self.clock_advanced = None
        self.active_goal = None
        self.goal_count = 0
        self.require_initial_view = self.declare_parameter('require_initial_view', False).value
        self.view_sample = None
        self.observed_goal = None
        self.software_stop = False
        self.latched_goal = None
        self.recovery_stop = False
        self.last_status = None
        self.stop = self.create_publisher(Bool, 'mid_odin_view/stop', 1)
        self.status = self.create_publisher(String, 'mid_odin_view/status', 10)
        self.create_subscription(Bool, 'safety_stop', self.emergency, 10)
        self.create_subscription(Bool, 'recovery/stop', self.recovery, 10)
        self.create_subscription(Imu, 'livox/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self.goals,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.require_initial_view:
            self.create_subscription(String, 'mid_odin_initial_view/status', self.initial_observation, 10)
        for sensor, frame, raw, converted in (
                ('mid', 'front_mid360_scan', 'livox/lidar', 'mid/geometric_obstacles'),
                ('odin', 'odin_scan', 'odin/points', 'odin/obstacles')):
            for kind, topic in (('raw', raw), ('converted', converted)):
                if source_header and sensor == 'mid' and kind == 'raw':
                    continue
                self.create_subscription(PointCloud2, topic,
                                         lambda msg, s=sensor, k=kind, f=frame: self.cloud(s, k, f, msg),
                                         qos_profile_sensor_data)
        if source_header:
            self.create_subscription(Header, 'mid/geometric_raw_header', self.mid_header,
                                     qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.stop.publish(Bool(data=True))

    def emergency(self, msg):
        self.software_stop = msg.data
        if msg.data:
            self.latched_goal = self.active_goal
            self.stop.publish(Bool(data=True))

    def recovery(self, msg):
        self.recovery_stop = msg.data
        if msg.data:
            self.observed_goal = self.view_sample = None
            self.stop.publish(Bool(data=True))

    def goals(self, message):
        active = [bytes(item.goal_info.goal_id.uuid).hex() for item in message.status_list
                  if item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)]
        self.goal_count = len(active)
        current = active[0] if self.goal_count == 1 else None
        if current != self.active_goal:
            self.observed_goal = self.view_sample = None
        self.active_goal = current
        if not self.active_goal:
            self.stop.publish(Bool(data=True))

    def imu(self, message):
        velocity = message.angular_velocity
        if all(math.isfinite(v) for v in (velocity.x, velocity.y, velocity.z)):
            self.primary_imu = (stamp(message.header), time.monotonic())

    def cloud(self, sensor, kind, frame, msg):
        if valid_cloud(msg, frame, kind == 'raw'):
            getattr(self, sensor).add(kind, stamp(msg.header), time.monotonic())

    def mid_header(self, header):
        if header.frame_id == 'front_mid360_scan':
            self.mid.add('raw', stamp(header), time.monotonic())

    def initial_observation(self, msg):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(status, dict) or not self.active_goal or status.get('active_goal') != self.active_goal:
            return
        current = status.get('ros')
        if status.get('permit') is True and status.get('state') == 'ready' and isinstance(current, (int, float)) and math.isfinite(current):
            self.view_sample = (current, time.monotonic())
        else:
            self.view_sample = None

    def tf_fresh(self, frame, ros):
        try:
            transform = self.tf.lookup_transform('odom', frame, Time())
        except TransformException:
            return False
        return -.1 <= ros - stamp(transform.header) <= .3

    def reason(self, ros, wall):
        if self.software_stop or self.recovery_stop or (self.active_goal and self.active_goal == self.latched_goal):
            return 'external_stop'
        if not self.active_goal:
            return 'no_single_active_goal'
        if self.clock_advanced is None or wall - self.clock_advanced > .3:
            return 'source_clock_stalled'
        if (self.primary_imu is None or not -.1 <= ros - self.primary_imu[0] <= .25
                or not 0 <= wall - self.primary_imu[1] <= .25):
            return 'primary_imu_stale'
        if not self.tf_fresh('base_footprint', ros) or not self.tf_fresh('odin_scan', ros):
            return 'obstacle_transforms_stale'
        if not self.mid.healthy(ros, wall):
            return 'mid_obstacle_source_unmatched'
        if not self.odin.healthy(ros, wall, max_wall_age=.3):
            return 'odin_obstacle_source_unmatched'
        if self.require_initial_view and self.observed_goal != self.active_goal:
            if (self.view_sample is None or not -.1 <= ros - self.view_sample[0] <= .25
                    or not 0 <= wall - self.view_sample[1] <= .25):
                return 'initial_direction_unobserved'
        return 'ready'

    def tick(self):
        wall = time.monotonic()
        ros = self.get_clock().now().nanoseconds * 1e-9
        if ros < self.last_ros:
            self.mid = SourcePairHealth()
            self.odin = SourcePairHealth()
            self.primary_imu = None
            self.clock_advanced = None
            self.observed_goal = self.view_sample = None
        elif ros > self.last_ros:
            self.clock_advanced = wall
        self.last_ros = ros
        reason = self.reason(ros, wall)
        if reason == 'ready' and self.require_initial_view:
            self.observed_goal = self.active_goal
        self.stop.publish(Bool(data=reason != 'ready'))
        state = dict(mode='costmap_navigation', state=reason, permit=reason == 'ready',
                     active_goal=self.active_goal, observed_goal=self.observed_goal, ros=ros)
        self.status.publish(String(data=json.dumps(state)))
        if reason != self.last_status:
            self.get_logger().info(json.dumps(state))
            self.last_status = reason


def main():
    lifecycle = init_context()
    node = CostmapGate()
    try:
        spin_context(node, lifecycle)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop.publish(Bool(data=True))
        node.destroy_node()
        shutdown_context(lifecycle)


if __name__ == '__main__':
    main()
