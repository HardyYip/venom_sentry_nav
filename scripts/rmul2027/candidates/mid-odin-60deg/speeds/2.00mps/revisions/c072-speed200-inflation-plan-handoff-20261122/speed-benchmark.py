"""Measure straight-line speed and stopped arrival in the isolated RMUL simulator."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from example_interfaces.msg import Bool
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def run(output):
    rclpy.init(args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
    node = Node('speed_benchmark', namespace='/red_standard_robot1',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf = Buffer()
    listener = TransformListener(tf, node)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    permit = node.create_publisher(Bool, 'decision_stop', 1)
    truth = {}
    commands = {}
    active = None
    stopping = True
    leg = ''
    rows = []
    results = []
    next_sample = 0.

    def now():
        return node.get_clock().now().nanoseconds / 1e9

    def odometry(message):
        nonlocal next_sample
        pos = message.pose.pose.position
        twist = message.twist.twist
        stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
        truth.update(x=pos.x + 4.8, y=pos.y - 3., speed=math.hypot(twist.linear.x, twist.linear.y),
                     angular=abs(twist.angular.z), sim=stamp, received=time.monotonic())
        if stamp >= next_sample:
            next_sample = stamp + .05
            record = dict(leg=leg, **truth)
            record.update({key: value['speed'] for key, value in commands.items()})
            rows.append(record)

    def velocity(message, key):
        commands[key] = dict(speed=math.hypot(message.linear.x, message.linear.y),
                             angular=abs(message.angular.z), received=time.monotonic())

    def until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(leg or 'startup')
            rclpy.spin_once(node, timeout_sec=.05)

    def response(future, timeout):
        until(future.done, timeout)
        return future.result()

    def stopped_since():
        values = [truth, commands.get('output')]
        if not all(v and time.monotonic() - v['received'] < .3 for v in values):
            return None
        if max(max(v['speed'], v['angular']) for v in values) >= .03:
            return None
        return truth['sim']

    node.create_subscription(Odometry, 'chassis_odometry_gt', odometry, qos_profile_sensor_data)
    node.create_subscription(Twist, 'cmd_vel', lambda msg: velocity(msg, 'output'), 10)
    node.create_subscription(Twist, 'cmd_vel_controller', lambda msg: velocity(msg, 'controller'), 10)
    node.create_timer(.05, lambda: permit.publish(Bool(data=stopping)))
    try:
        until(lambda: truth and tf.can_transform('map', 'gimbal_yaw_fake', rclpy.time.Time()), 35)
        if node.count_publishers('decision_stop') != 1:
            raise RuntimeError('Unexpected decision-stop publisher')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('NavigateToPose unavailable')
        stopping = False
        for leg, target in (
            ('position', (0., -5.5)), ('outbound', (5.2, -5.5)), ('return', (0., -5.5))
        ):
            start = now()
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = target
            goal.pose.pose.orientation = tf.lookup_transform(
                'map', 'gimbal_yaw_fake', rclpy.time.Time()).transform.rotation
            active = response(client.send_goal_async(goal), 10)
            if not active.accepted:
                raise RuntimeError(f'{leg} rejected')
            status = response(active.get_result_async(), 100).status
            active = None
            if status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'{leg} returned status {status}')
            arrived = now()
            first_stop = None

            def stable():
                nonlocal first_stop
                current = stopped_since()
                if current is None or first_stop is None:
                    first_stop = current
                return current is not None and current - first_stop >= 1.

            until(stable, 12)
            error = math.hypot(truth['x'] - target[0], truth['y'] - target[1])
            if error > .25:
                raise RuntimeError(f'{leg} ended {error:.3f} m from the true target')
            samples = [row for row in rows if row['leg'] == leg]
            result = dict(leg=leg, target=target, sim_seconds=arrived - start, goal_error_m=error,
                          stop_verified=True,
                          truth_peak_mps=max(row['speed'] for row in samples),
                          command_peak_mps=max(row.get('controller', 0.) for row in samples),
                          output_peak_mps=max(row.get('output', 0.) for row in samples))
            results.append(result)
            print(json.dumps(result), flush=True)
    finally:
        stopping = True
        permit.publish(Bool(data=True))
        try:
            if active is not None and active.accepted:
                response(active.cancel_goal_async(), 5)
        finally:
            (output / 'samples.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
            (output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run(Path(sys.argv[1]))
