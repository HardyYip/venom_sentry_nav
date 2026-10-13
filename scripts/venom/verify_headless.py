"""Verify the official single-robot mapping or navigation stack without RViz."""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=('nav', 'mapping'), default='nav')
parser.add_argument('--startup-timeout', type=float, default=180)
parser.add_argument('--goal-timeout', type=float, default=180)
options = parser.parse_args()
rclpy.init(args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
node = Node('venom_headless_verify', namespace='/red_standard_robot1')
node.set_parameters([Parameter('use_sim_time', value=True)])
messages = {}
counts = {}
nonzero_commands = 0
sensor_qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
cloud_qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.RELIABLE)
map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def receive(key, message):
    global nonzero_commands
    counts[key] = counts.get(key, 0) + 1
    if isinstance(message, PointCloud2):
        messages[key] = {'points': message.width * message.height,
                         'frame': message.header.frame_id}
    else:
        messages[key] = message
    if key == 'cmd_vel':
        if max(abs(message.linear.x), abs(message.linear.y), abs(message.angular.z)) > .02:
            nonzero_commands += 1


subscriptions = []
for message_type, topic, key, qos in (
    (Clock, '/clock', 'clock', sensor_qos),
    (PointCloud2, 'livox/lidar', 'lidar', sensor_qos),
    (PointCloud2, 'velodyne_points', 'converted_cloud', sensor_qos),
    (Imu, 'livox/imu', 'imu', sensor_qos),
    (Odometry, 'aft_mapped_to_init', 'lio', sensor_qos),
    (PointCloud2, 'cloud_registered', 'lio_cloud', cloud_qos),
    (Odometry, 'lidar_odometry', 'adapted_odometry', sensor_qos),
    (PointCloud2, 'registered_scan', 'registered_scan', sensor_qos),
    (Odometry, 'odometry', 'odometry', sensor_qos),
    (Odometry, 'chassis_odometry_gt', 'ground_truth', sensor_qos),
    (PointCloud2, 'terrain_map', 'terrain_map', sensor_qos),
    (PointCloud2, 'terrain_map_ext', 'terrain_map_ext', sensor_qos),
    (OccupancyGrid, 'map', 'map', map_qos),
    (LaserScan, 'obstacle_scan', 'scan', sensor_qos),
    (Twist, 'cmd_vel', 'cmd_vel', 10),
):
    subscriptions.append(node.create_subscription(
        message_type, topic, lambda message, key=key: receive(key, message), qos))

buffer = Buffer()
listener = TransformListener(buffer, node)
servers = ('controller_server', 'planner_server', 'bt_navigator', 'velocity_smoother')
clients = {server: node.create_client(GetState, f'{server}/get_state') for server in servers}
goal_handle = None
goal_done = False


def emit(event, **values):
    print(json.dumps({'event': event, 'mode': options.mode, **values}), flush=True)


def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)


def wait_future(future, seconds):
    end = time.monotonic() + seconds
    next_report = time.monotonic() + 15
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
        if time.monotonic() >= next_report:
            emit('waiting_for_action', counts=counts)
            next_report += 15
    return future.result() if future.done() else None


def pose():
    return buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform


def clock_seconds():
    return messages['clock'].clock.sec + messages['clock'].clock.nanosec * 1e-9


try:
    required = {'clock', 'lidar', 'converted_cloud', 'imu', 'lio', 'lio_cloud',
                'adapted_odometry', 'registered_scan', 'odometry',
                'ground_truth', 'terrain_map', 'terrain_map_ext', 'map'}
    if options.mode == 'mapping':
        required.add('scan')
    deadline = time.monotonic() + options.startup_timeout
    next_report = time.monotonic()
    states = {}
    state_futures = {}
    next_states = 0
    ready = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        now = time.monotonic()
        for name, (future, sent_at) in list(state_futures.items()):
            if future.done():
                states[name] = future.result().current_state.id
                del state_futures[name]
            elif now - sent_at > 5:
                # Discovery during component startup may leave an unanswered
                # request. Retry the check, without assuming the node is active.
                clients[name].remove_pending_request(future)
                future.cancel()
                del state_futures[name]
                emit('lifecycle_retry', server=name)
        if now >= next_states:
            for name, client in clients.items():
                if name not in state_futures and client.service_is_ready():
                    state_futures[name] = (client.call_async(GetState.Request()), now)
            next_states = now + 1
        frames_ready = False
        try:
            pose()
            buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            frames_ready = True
        except Exception:
            pass
        clouds_ready = all(messages.get(key, {}).get('points', 0) > 0
                           for key in ('lidar', 'converted_cloud', 'lio_cloud',
                                       'registered_scan', 'terrain_map', 'terrain_map_ext'))
        ready = (required.issubset(messages) and frames_ready and clouds_ready
                 and all(states.get(name) == 3 for name in servers))
        if ready:
            break
        if now >= next_report:
            emit('startup', missing=sorted(required - messages.keys()),
                 frames_ready=frames_ready, states=states, counts=counts)
            next_report = now + 15
    assert ready, f'Stack not ready: missing={required - messages.keys()}, states={states}'
    initial_clock = clock_seconds()
    initial_counts = counts.copy()
    spin_for(12)
    assert clock_seconds() > initial_clock + .05, 'Simulation clock is not progressing'
    for key in required - {'map'}:
        assert counts.get(key, 0) > initial_counts.get(key, 0), f'Output stalled: {key}'
    graph = node.get_node_names_and_namespaces()
    for server in servers:
        instances = sum(name == server and namespace == '/red_standard_robot1'
                        for name, namespace in graph)
        assert instances == 1, f'Expected one {server}, found {instances}'
    transform = pose()
    x, y = transform.translation.x, transform.translation.y
    assert math.isfinite(x) and math.isfinite(y), 'Invalid robot pose'
    emit('ready', map_xy=[x, y], sim_time=clock_seconds(), states=states, counts=counts)
    if options.mode == 'mapping':
        spin_for(8)
        assert messages['map'].info.width > 0 and messages['map'].info.height > 0
        emit('PASS', counts=counts, sim_time=clock_seconds())
    else:
        grid = messages['map']
        origin = grid.info.origin
        q = origin.orientation
        origin_yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        co, si = math.cos(origin_yaw), math.sin(origin_yaw)

        def clear(px, py):
            radius = .45
            for ix in range(-5, 6):
                for iy in range(-5, 6):
                    ox, oy = ix*radius/5, iy*radius/5
                    if ox*ox + oy*oy > radius*radius:
                        continue
                    dx, dy = px + ox - origin.position.x, py + oy - origin.position.y
                    col = math.floor((co*dx + si*dy) / grid.info.resolution)
                    row = math.floor((-si*dx + co*dy) / grid.info.resolution)
                    if not (0 <= col < grid.info.width and 0 <= row < grid.info.height):
                        return False
                    value = grid.data[row*grid.info.width + col]
                    if value < 0 or value > 10:
                        return False
            return True

        goal_xy = None
        for distance in (.8, .6):
            for angle in (0, math.pi/2, -math.pi/2, math.pi, math.pi/4,
                          -math.pi/4, 3*math.pi/4, -3*math.pi/4):
                if all(clear(x+d*math.cos(angle), y+d*math.sin(angle))
                       for d in (distance*i/8 for i in range(9))):
                    goal_xy = (x+distance*math.cos(angle), y+distance*math.sin(angle))
                    break
            if goal_xy is not None:
                break
        assert goal_xy is not None, 'No short goal with safe clearance in the official map'
        truth = messages['ground_truth'].pose.pose.position
        initial_truth = (truth.x, truth.y)
        client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        assert client.wait_for_server(timeout_sec=10), 'Navigation action missing'
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = goal_xy
        goal.pose.pose.orientation = buffer.lookup_transform(
            'map', 'gimbal_yaw_fake', rclpy.time.Time()).transform.rotation
        emit('goal', xy=goal_xy, initial_ground_truth=initial_truth)
        goal_handle = wait_future(client.send_goal_async(goal), 15)
        assert goal_handle is not None and goal_handle.accepted, 'Goal rejected'
        result = wait_future(goal_handle.get_result_async(), options.goal_timeout)
        assert result is not None, 'Navigation timed out'
        goal_done = True
        spin_for(1)
        final = pose()
        truth = messages['ground_truth'].pose.pose.position
        movement = math.hypot(truth.x-initial_truth[0], truth.y-initial_truth[1])
        error = math.hypot(final.translation.x-goal_xy[0], final.translation.y-goal_xy[1])
        emit('result', status=result.status, ground_truth_moved=movement,
             goal_error=error, nonzero_commands=nonzero_commands, counts=counts)
        assert result.status == 4, 'Navigation did not return SUCCEEDED'
        assert .2 < movement < 2.0, 'Ground truth does not show expected short movement'
        assert error < .35, 'Final estimated pose is too far from the target'
        assert nonzero_commands > 0, 'No nonzero chassis command observed'
        emit('PASS')
except Exception as error:
    emit('FAIL', error=str(error), counts=counts)
    raise
finally:
    if goal_handle is not None and goal_handle.accepted and not goal_done:
        wait_future(goal_handle.cancel_goal_async(), 5)
    node.destroy_node()
    rclpy.shutdown()
