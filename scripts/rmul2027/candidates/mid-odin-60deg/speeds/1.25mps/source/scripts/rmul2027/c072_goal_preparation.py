"""Bounded goal preparation for the C072 low-speed simulation profile."""
import json
import math
import os
import threading
import time

import rclpy
from example_interfaces.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from c075_ros_lifecycle import init_context, shutdown_context


def look_target(poses):
    # Repeated NavFn segments near the start do not establish a heading.
    start = poses[0].pose.position
    for pose in poses[1:]:
        point = pose.pose.position
        if math.hypot(point.x - start.x, point.y - start.y) >= .85:
            return pose
    return poses[-1]


class Preparation(Node):
    def __init__(self):
        super().__init__('c072_goal_preparation', parameter_overrides=[Parameter('use_sim_time', value=True)])
        group = ReentrantCallbackGroup()
        self.intent = self.create_publisher(PoseStamped, 'mid_odin_view/intent', 10)
        self.status = self.create_publisher(String, 'recovery/goal_preparation_status', 10)
        self.create_subscription(String, 'mid_odin_initial_view/status', self.view, 10, callback_group=group)
        self.create_subscription(Bool, 'safety_stop', self.emergency, 10, callback_group=group)
        self.create_subscription(Bool, 'recovery/stop', self.recovery, 10, callback_group=group)
        self.navigation = ActionClient(self, NavigateToPose, 'navigate_to_pose_internal', callback_group=group)
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose', callback_group=group)
        self.server = ActionServer(self, NavigateToPose, 'navigate_to_pose',
                                   execute_callback=self.execute, goal_callback=self.prepare,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.slot = threading.Lock()
        self.condition = threading.Condition()
        self.last_view = None
        self.software_stop = False
        self.recovery_stop = os.environ.get("RMUL_IMU_RECOVERY") == "1"
        self.execution_active = False
        self.recovery_generation = 0
        self.create_timer(.05, self.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME),
                          callback_group=group)

    def publish_status(self):
        self.status.publish(String(data=json.dumps({
            'busy': self.slot.locked(), 'execution_active': self.execution_active,
            'emergency_stop': self.software_stop
        })))

    def emergency(self, msg):
        self.software_stop = msg.data
        with self.condition:
            if msg.data:
                self.last_view = None
            self.condition.notify_all()

    def recovery(self, msg):
        with self.condition:
            if msg.data and not self.recovery_stop:
                self.recovery_generation += 1
            self.recovery_stop = msg.data
            if msg.data:
                self.last_view = None
            self.condition.notify_all()

    def view(self, msg):
        try:
            parsed = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(parsed, dict):
            return
        with self.condition:
            self.last_view = parsed
            self.condition.notify_all()

    def inhibited(self):
        return self.software_stop or self.recovery_stop

    @staticmethod
    def result_until(future, deadline):
        complete = threading.Event()
        future.add_done_callback(lambda _: complete.set())
        return future.result() if complete.wait(max(0., deadline - time.monotonic())) else None

    def preview(self, request, deadline):
        query = ComputePathToPose.Goal()
        query.goal = request.pose
        query.planner_id = 'GridBased'
        sent = self.planner.send_goal_async(query)
        handle = self.result_until(sent, min(deadline, time.monotonic() + 2.))
        if handle is None or not handle.accepted:
            if handle is None:
                def cancel_late(future):
                    late = future.result()
                    if late and late.accepted:
                        late.cancel_goal_async()
                sent.add_done_callback(cancel_late)
            return None
        result = self.result_until(handle.get_result_async(), min(deadline, time.monotonic() + 2.))
        if result is None:
            handle.cancel_goal_async()
            return None
        if result.status != 4 or len(result.result.path.poses) < 2:
            return None
        poses = result.result.path.poses
        return look_target(poses)

    def prepare(self, request):
        if not self.slot.acquire(blocking=False):
            return GoalResponse.REJECT
        admit = False
        deadline = time.monotonic() + 13.
        try:
            while rclpy.ok() and not self.software_stop and time.monotonic() < deadline:
                with self.condition:
                    if (self.recovery_stop or not self.navigation.server_is_ready() or
                            not self.planner.server_is_ready()):
                        self.condition.wait(timeout=min(.1, max(0., deadline - time.monotonic())))
                        continue
                    generation = self.recovery_generation
                look = self.preview(request, deadline)
                with self.condition:
                    if generation != self.recovery_generation:
                        continue
                    if look is None:
                        self.get_logger().warning('Goal preparation could not produce a clearance-checked path')
                        return GoalResponse.REJECT
                    look.header.stamp = self.get_clock().now().to_msg()
                    tag = [look.header.stamp.sec, look.header.stamp.nanosec]
                    view_deadline = min(deadline, time.monotonic() + 8.)
                    self.last_view = None
                    self.intent.publish(look)
                    while (not self.inhibited() and generation == self.recovery_generation
                           and time.monotonic() < view_deadline and rclpy.ok()):
                        view = self.last_view
                        if (view and view.get('intent_stamp') == tag and view.get('active_goal') is None
                                and view.get('state') == 'ready'):
                            self.execution_active = True
                            admit = True
                            return GoalResponse.ACCEPT
                        self.condition.wait(timeout=min(.1, max(0., view_deadline - time.monotonic())))
                    if generation == self.recovery_generation:
                        break
            self.get_logger().warning('Goal preparation ended without matching directional observation')
            return GoalResponse.REJECT
        finally:
            if not admit:
                self.slot.release()

    async def execute(self, outer):
        try:
            if outer.is_cancel_requested or self.inhibited():
                if outer.is_cancel_requested:
                    outer.canceled()
                else:
                    outer.abort()
                return NavigateToPose.Result()
            inner = await self.navigation.send_goal_async(
                outer.request, goal_uuid=outer.goal_id,
                feedback_callback=lambda feed: outer.publish_feedback(feed.feedback) if outer.is_active else None)
            if not inner.accepted:
                outer.abort()
                return NavigateToPose.Result()
            result = inner.get_result_async()
            canceled = False
            while rclpy.ok() and not result.done():
                if not canceled and (outer.is_cancel_requested or self.inhibited()):
                    inner.cancel_goal_async()
                    canceled = True
                await self.executor.create_task(self.poll)
            if not result.done():
                if outer.is_active:
                    outer.abort()
                return NavigateToPose.Result()
            terminal = result.result()
            if outer.is_cancel_requested:
                outer.canceled()
            elif terminal.status == 4 and not canceled:
                outer.succeed()
            else:
                outer.abort()
            return terminal.result
        finally:
            self.execution_active = False
            self.slot.release()

    @staticmethod
    def poll():
        time.sleep(.025)


def main():
    lifecycle = init_context()
    node = Preparation()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        while rclpy.ok() and (lifecycle is None or not lifecycle.stopping):
            executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down ROS before joining an executor awaiting a navigation result.
        shutdown_context(lifecycle)
        executor.shutdown(timeout_sec=2.)
        node.destroy_node()


if __name__ == '__main__':
    main()
