"""Replay the RViz action, starting in the map’s inner inflation near the field wall."""
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
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def run(output, spawn):
    rclpy.init(args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
    node = Node('rviz_inflation_replay', namespace='/red_standard_robot1',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf = Buffer()
    listener = TransformListener(tf, node)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    permit = node.create_publisher(Bool, 'decision_stop', 1)
    empty_plan = node.create_publisher(NavPath, 'local_plan', 1)
    truth = {}
    commands = {}
    active = None
    stopping = True
    leg = ''
    rows = []
    results = []
    next_sample = 0.
    grid = None
    first_cost = None

    def costmap(message):
        nonlocal grid
        if message.header.frame_id == 'map':
            grid = message

    def cell_at(x, y):
        if grid is None:
            return None
        meta = grid.metadata
        origin = meta.origin.position
        i = math.floor((x - origin.x) / meta.resolution)
        j = math.floor((y - origin.y) / meta.resolution)
        if 0 <= i < meta.size_x and 0 <= j < meta.size_y:
            return int(grid.data[j * meta.size_x + i])
        return None

    def raw_clearance(x, y):
        if grid is None:
            return None
        meta = grid.metadata
        origin = meta.origin.position
        res = meta.resolution
        i = math.floor((x - origin.x) / res)
        j = math.floor((y - origin.y) / res)
        cells = math.ceil(.65 / res)
        nearest = .65
        for cy in range(j - cells, j + cells + 1):
            for cx in range(i - cells, i + cells + 1):
                outside = not (0 <= cx < meta.size_x and 0 <= cy < meta.size_y)
                if outside or grid.data[cy * meta.size_x + cx] >= 254:
                    dx = max(0., abs(x - origin.x - (cx + .5) * res) - res / 2)
                    dy = max(0., abs(y - origin.y - (cy + .5) * res) - res / 2)
                    nearest = min(nearest, math.hypot(dx, dy))
        return nearest

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
            record = dict(leg=leg, center_cost=cell_at(truth['x'],truth['y']),
                          raw_clearance_m=raw_clearance(truth['x'],truth['y']), **truth)
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
    node.create_subscription(Costmap, 'global_costmap/costmap_raw', costmap,
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_timer(.05, lambda: permit.publish(Bool(data=stopping)))
    try:
        until(lambda: grid is not None and truth and tf.can_transform('map', 'gimbal_yaw_fake', rclpy.time.Time()), 35)
        if node.count_publishers('decision_stop') != 1:
            raise RuntimeError('Unexpected decision-stop publisher')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('NavigateToPose unavailable')
        first_cost = cell_at(truth['x'], truth['y'])
        assert math.dist((truth['x'],truth['y']), spawn) <= .04, truth
        assert first_cost is not None and 180 <= first_cost <= 253, first_cost
        stopping = False
        for leg, target in (
            ('escape_and_navigate', (5.198,-4.553)), ('reject_deep_goal', (3.801,-1.942)),
            ('second_valid_goal', (5.0,-.2))
        ):
            start = now()
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = target
            goal.pose.pose.orientation = tf.lookup_transform(
                'map', 'gimbal_yaw_fake', rclpy.time.Time()).transform.rotation
            active = response(client.send_goal_async(goal), 10)
            if leg == 'reject_deep_goal':
                if active.accepted:
                    raise RuntimeError('A goal inside the inner reserve was accepted')
                active = None
                start_xy = truth['x'],truth['y']
                until(lambda: now() - start >= 1., 8)
                assert math.dist(start_xy,(truth['x'],truth['y'])) < .02
                results.append(dict(leg=leg, status='rejected', no_motion=True))
                continue
            if not active.accepted:
                raise RuntimeError(f'{leg} rejected')
            if leg == 'escape_and_navigate':
                # Recreate the gate input left by a DWB cycle with no legal trajectory.
                fault = NavPath()
                fault.header = tf.lookup_transform(
                    'odom', 'gimbal_yaw_fake', rclpy.time.Time()).header
                empty_plan.publish(fault)
                print(json.dumps({'fixture_event':'empty_local_plan', 'sim':now()}), flush=True)
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
                          output_peak_mps=max(row.get('output', 0.) for row in samples),
                          minimum_raw_clearance_m=min(row['raw_clearance_m'] for row in samples
                              if row['raw_clearance_m'] is not None))
            assert result['minimum_raw_clearance_m'] > .3829, result
            assert max(row['center_cost'] for row in samples if row['center_cost'] is not None) < 254
            if leg == 'escape_and_navigate':
                exit_index = next(i for i,row in enumerate(samples) if
                    row['center_cost'] is not None and row['center_cost'] < 180)
                escape_samples = samples[:exit_index]
                assert escape_samples and max(row.get('controller',0.) for row in escape_samples) <= .201
                assert all(row['center_cost'] < 253 for row in samples[exit_index:] if
                           row['center_cost'] is not None)
                result.update(start_cost=first_cost, exit_cost=samples[exit_index]['center_cost'],
                    escape_controller_peak_mps=max(row.get('controller',0.) for row in escape_samples))
            else:
                assert all(row['center_cost'] < 253 for row in samples if row['center_cost'] is not None)
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
    run(Path(sys.argv[1]), (float(sys.argv[2]),float(sys.argv[3])))
