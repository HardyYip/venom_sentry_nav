import bisect
import numpy as np
"""Verify the official single-robot mapping or navigation stack without RViz."""
import argparse
import re
import subprocess
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path as NavPath
from pathlib import Path
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=('nav', 'mapping', 'ready'), default='nav')
parser.add_argument('--startup-timeout', type=float, default=180)
parser.add_argument('--goal-timeout', type=float, default=180)
parser.add_argument('--scenario', choices=('simple','blocked','invalid','blocked_route'), default='simple')
parser.add_argument('--behavior-tree', type=str)
parser.add_argument('--goal-x', type=float)
parser.add_argument('--goal-y', type=float)
parser.add_argument('--record-dir', type=Path, required=True)
parser.add_argument('--stability-seconds', type=float, default=12)
parser.add_argument('--spin-mode', action='store_true')
options = parser.parse_args()
options.record_dir.mkdir(parents=True,exist_ok=True)
trace = []
snapshots = {}
paths = {}
rclpy.init(args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
node = Node('venom_headless_verify', namespace='/red_standard_robot1')
node.set_parameters([Parameter('use_sim_time', value=True)])
messages = {}
truth_history=[]
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
    if key=='ground_truth':
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        truth_history.append((stamp,message.pose.pose.position.x,message.pose.pose.position.y))
        if len(truth_history)>30000:
            del truth_history[:5000]
    if key in ('ground_truth','odometry','cmd_vel'):
        if key == 'cmd_vel':
            trace.append({'wall':time.time(),'kind':key,'value':[message.linear.x,message.linear.y,message.angular.z]})
        elif counts[key] % 10 == 0:
            p=message.pose.pose.position
            q=message.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            trace.append({'wall':time.time(),'kind':key,'stamp':message.header.stamp.sec+message.header.stamp.nanosec*1e-9,'value':[p.x,p.y,p.z],'yaw':yaw})
            if key == 'ground_truth':
                try:
                    transform=buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
                    estimate=transform.transform.translation
                    stamp=transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
                    trace.append({'wall':time.time(),'kind':'estimated_pose','stamp':stamp,'value':[estimate.x,estimate.y,estimate.z]})
                except Exception:
                    pass
    if isinstance(message, NavPath):
        paths[key]={'frame':message.header.frame_id,'poses':[[p.pose.position.x,p.pose.position.y] for p in message.poses]}
    if isinstance(message, OccupancyGrid):
        snapshots[key]={'width':message.info.width,'height':message.info.height,'resolution':message.info.resolution,'origin':[message.info.origin.position.x,message.info.origin.position.y],'data':list(message.data)}
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
    (OccupancyGrid, 'global_costmap/costmap', 'global_costmap', map_qos),
    (OccupancyGrid, 'local_costmap/costmap', 'local_costmap', map_qos),
    (NavPath, 'plan', 'global_plan', 10),
    (NavPath, 'local_plan', 'local_plan', 10),
):
    subscriptions.append(node.create_subscription(
        message_type, topic, lambda message, key=key: receive(key, message), qos))

buffer = Buffer()
listener = TransformListener(buffer, node)
servers = ('controller_server', 'planner_server', 'bt_navigator', 'velocity_smoother')
clients = {server: node.create_client(GetState, f'{server}/get_state') for server in servers}
goal_handle = None
goal_done = False
watchdog_active = options.mode == 'nav' and options.scenario in ('simple','blocked_route')
progress_checkpoint=None
progress_clock=None
stale_since=None


def emit(event, **values):
    print(json.dumps({'event': event, 'mode': options.mode, **values}), flush=True)


def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)


def wait_future(future, seconds):
    global watchdog_active, progress_checkpoint, progress_clock, stale_since
    end = time.monotonic() + seconds
    next_report = time.monotonic() + 15
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
        if watchdog_active and goal_handle is not None and goal_handle.accepted and not goal_done:
            transform=buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
            estimate=transform.transform.translation
            stamp=transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
            truth=messages['ground_truth'].pose.pose.position
            now_sim=clock_seconds()
            age=now_sim-stamp
            if age>1.0:
                if stale_since is None:
                    stale_since=now_sim
                elif now_sim-stale_since>2.0:
                    watchdog_active=False
                    raise RuntimeError(f'Localization transform stale: {age:.3f}s')
            else:
                stale_since=None
            if truth_history and truth_history[0][0]<=stamp<=truth_history[-1][0]:
                index=min(len(truth_history)-1,max(1,bisect.bisect_right(truth_history,(stamp,float('inf'),float('inf')))))
                before,after=truth_history[index-1],truth_history[index]
                weight=(stamp-before[0])/max(after[0]-before[0],1e-9)
                tx=before[1]+weight*(after[1]-before[1])
                ty=before[2]+weight*(after[2]-before[2])
                error=math.hypot(estimate.x-(tx+4.8),estimate.y-(ty-3.0))
                if error>.35:
                    sample={'wall':time.time(),'kind':'localization_guard','stamp':stamp,
                            'sim_time':now_sim,'tf_age_s':age,'error_m':error,'threshold_m':.35,
                            'estimated_xyz':[estimate.x,estimate.y,estimate.z],
                            'truth_map_xy':[tx+4.8,ty-3.0],
                            'truth_before':list(before),'truth_after':list(after),
                            'interpolation_weight':weight}
                    trace.append(sample)
                    (options.record_dir/'guard-trigger.json').write_text(json.dumps(sample,indent=2)+'\n')
                    emit('localization_guard_trigger',**sample)
                    watchdog_active=False
                    raise RuntimeError(f'Timestamp-aligned localization error exceeded .35m: {error:.3f}m; TF age {age:.3f}s')
            command=messages.get('cmd_vel',Twist())
            if math.hypot(command.linear.x,command.linear.y)>.05:
                now_sim=clock_seconds()
                if progress_checkpoint is None or math.hypot(truth.x-progress_checkpoint[0],truth.y-progress_checkpoint[1])>.05:
                    progress_checkpoint=(truth.x,truth.y)
                    progress_clock=now_sim
                elif now_sim-progress_clock>8.0:
                    watchdog_active=False
                    raise RuntimeError('No physical progress for 8 simulation seconds despite nonzero commands')
            else:
                progress_checkpoint=None

        if time.monotonic() >= next_report:
            emit('waiting_for_action', counts=counts)
            next_report += 15
    return future.result() if future.done() else None


def record_goal_ownership(action_name):
    identifier=bytes(goal_handle.goal_id.uuid).hex()
    value={'action':action_name,'goal_uuid':identifier,'owner_node':node.get_fully_qualified_name(),'sent_wall_epoch':time.time(),'cancel_scope':'this UUID only'}
    (options.record_dir/'goal-ownership.json').write_text(json.dumps(value,indent=2)+'\n')
    emit('goal_accepted',**value)


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
    spin_for(options.stability_seconds)
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
    if options.mode in ('mapping','ready'):
        spin_for(8)
        assert messages['map'].info.width > 0 and messages['map'].info.height > 0
        if options.mode == 'ready':
            estimates=[e['value'] for e in trace if e['kind']=='estimated_pose']
            assert len(estimates)>2
            drift=max(math.hypot(e[0]-estimates[0][0],e[1]-estimates[0][1]) for e in estimates)
            truth=messages['ground_truth'].pose.pose.position
            estimate=pose().translation
            error=math.hypot(estimate.x-(truth.x+4.8),estimate.y-(truth.y-3.0))
            emit('localization_stability',max_xy_drift_m=drift,ground_truth_error_m=error)
            assert drift<.10 and error<.35, 'Stationary localization is unstable or inaccurate'
        emit('PASS', counts=counts, sim_time=clock_seconds())
    else:

        final_trace = []
        grid = messages['map']
        origin = grid.info.origin
        q = origin.orientation
        origin_yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        co, si = math.cos(origin_yaw), math.sin(origin_yaw)

        footprint_rotation = pose().rotation
        footprint_yaw = math.atan2(2*(footprint_rotation.w*footprint_rotation.z + footprint_rotation.x*footprint_rotation.y), 1-2*(footprint_rotation.y**2+footprint_rotation.z**2))
        fyc, fys = math.cos(footprint_yaw), math.sin(footprint_yaw)

        def clear(px, py):
            for ix in range(-5, 6):
                for iy in range(-5, 6):
                    fx, fy = ix*.31/5, iy*.27/5
                    ox, oy = fyc*fx-fys*fy, fys*fx+fyc*fy
                    dx, dy = px+ox-origin.position.x, py+oy-origin.position.y
                    col = math.floor((co*dx+si*dy)/grid.info.resolution)
                    row = math.floor((-si*dx+co*dy)/grid.info.resolution)
                    if not (0 <= col < grid.info.width and 0 <= row < grid.info.height):
                        return False
                    value = grid.data[row*grid.info.width+col]
                    relative_x = fyc*(px+ox-x)+fys*(py+oy-y)
                    relative_y = -fys*(px+ox-x)+fyc*(py+oy-y)
                    under_current_robot = abs(relative_x) <= .310001 and abs(relative_y) <= .270001
                    if value > 10 or (value < 0 and not under_current_robot):
                        return False
            return True

        if options.scenario in ('blocked','invalid'):
            candidates = []
            for index, value in enumerate(grid.data):
                if value < 95:
                    continue
                row, col = divmod(index, grid.info.width)
                gx = origin.position.x + (col+.5)*grid.info.resolution
                gy = origin.position.y + (row+.5)*grid.info.resolution
                distance = math.hypot(gx-x, gy-y)
                if .8 < distance < 2.:
                    candidates.append((distance, gx, gy))
            assert candidates, 'No nearby occupied target for blocked-path test'
            _, gx, gy = min(candidates)
            before = messages['ground_truth'].pose.pose.position
            initial_truth = (before.x, before.y)
            action_type=NavigateToPose if options.scenario=='invalid' else ComputePathToPose
            action_name='navigate_to_pose' if options.scenario=='invalid' else 'compute_path_to_pose'
            client = ActionClient(node, action_type, action_name)
            assert client.wait_for_server(timeout_sec=10)
            goal=action_type.Goal()
            target=goal.pose if options.scenario=='invalid' else goal.goal
            target.header.frame_id='map'
            target.header.stamp=node.get_clock().now().to_msg()
            target.pose.position.x,target.pose.position.y=gx,gy
            target.pose.orientation.w=1.
            if options.scenario=='invalid':
                if options.behavior_tree:
                    goal.behavior_tree=options.behavior_tree
            else:
                goal.planner_id='GridBased'
            emit('invalid_goal' if options.scenario=='invalid' else 'blocked_goal',xy=[gx,gy],occupancy=100)
            started=time.monotonic()
            goal_handle=wait_future(client.send_goal_async(goal),15)
            assert goal_handle is not None and goal_handle.accepted
            record_goal_ownership(action_name)
            result=wait_future(goal_handle.get_result_async(),15 if options.scenario=='invalid' else 45)
            assert result is not None,'Occupied target was not rejected promptly'
            goal_done=True
            spin_for(3)
            after=messages['ground_truth'].pose.pose.position
            movement=math.hypot(after.x-initial_truth[0],after.y-initial_truth[1])
            points=0 if options.scenario=='invalid' else len(result.result.path.poses)
            emit('invalid_result' if options.scenario=='invalid' else 'blocked_result',status=result.status,path_points=points,ground_truth_moved=movement,elapsed_wall_s=time.monotonic()-started,nonzero_commands=nonzero_commands)
            assert result.status==6 and points==0
            assert movement<.05
            if options.scenario=='invalid':
                assert nonzero_commands==0,'Invalid goal triggered a moving recovery'
            emit('PASS',scenario=options.scenario)
        else:
            goal_xy = None
            if options.goal_x is not None and options.goal_y is not None:
                goal_xy = (options.goal_x, options.goal_y)
                assert clear(*goal_xy), 'Requested target does not have safe map clearance'
            else:
                for distance in (.8, .6):
                    for angle in (0, math.pi/2, -math.pi/2, math.pi, math.pi/4, -math.pi/4):
                        if all(clear(x+d*math.cos(angle), y+d*math.sin(angle))
                               for d in (distance*i/8 for i in range(9))):
                            goal_xy = (x+distance*math.cos(angle), y+distance*math.sin(angle))
                            break
                    if goal_xy is not None:
                        break
            assert goal_xy is not None, 'No short target with safe clearance'
            truth = messages['ground_truth'].pose.pose.position
            initial_truth = (truth.x, truth.y)
            client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
            assert client.wait_for_server(timeout_sec=10)
            goal = NavigateToPose.Goal()
            if options.behavior_tree:
                goal.behavior_tree=options.behavior_tree
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = goal_xy
            heading_started = time.monotonic()
            heading_deadline = heading_started + min(options.startup_timeout, 10.)
            while not buffer.can_transform('map', 'gimbal_yaw_fake', rclpy.time.Time()):
                assert time.monotonic() < heading_deadline, 'Goal heading TF is not ready'
                rclpy.spin_once(node, timeout_sec=.1)
            goal.pose.pose.orientation = buffer.lookup_transform(
                'map', 'gimbal_yaw_fake', rclpy.time.Time()).transform.rotation
            emit('goal_heading_ready', waited_wall_s=time.monotonic()-heading_started)
            emit('goal', xy=goal_xy, initial_ground_truth=initial_truth)
            started = time.monotonic()
            goal_handle = wait_future(client.send_goal_async(goal), 15)
            assert goal_handle is not None and goal_handle.accepted, 'Goal rejected'
            record_goal_ownership('navigate_to_pose')
            result = wait_future(goal_handle.get_result_async(), options.goal_timeout)
            assert result is not None, 'Navigation timed out'
            goal_done = True
            initial_sim_time = clock_seconds()
            stop_deadline = time.monotonic() + 45
            while clock_seconds() < initial_sim_time + 1.0 and time.monotonic() < stop_deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            first_velocity = messages['ground_truth'].twist.twist
            first_command = messages.get('cmd_vel', Twist())
            initial_stop_snapshot = {'at_sim_s':clock_seconds()-initial_sim_time,
                'translation_speed_mps':math.hypot(first_velocity.linear.x,first_velocity.linear.y),
                'translation_command_mps':math.hypot(first_command.linear.x,first_command.linear.y),
                'angular_speed_rps':abs(first_velocity.angular.z),'angular_command_rps':abs(first_command.angular.z)}
            allow_arrival_spin = options.spin_mode and options.scenario == 'simple' and result.status == 4
            stop_samples=[];stable_since=None;stable_stop_confirmed=False
            settle_wall_deadline=time.monotonic()+45
            while clock_seconds() <= initial_sim_time+2.0 and time.monotonic()<settle_wall_deadline:
                observed=messages['ground_truth'].twist.twist
                observed_command=messages.get('cmd_vel',Twist())
                sim_now=clock_seconds()
                linear_speed=math.hypot(observed.linear.x,observed.linear.y)
                linear_command=math.hypot(observed_command.linear.x,observed_command.linear.y)
                stopped=linear_speed<.03 and linear_command<.03
                if not allow_arrival_spin:stopped=stopped and abs(observed.angular.z)<.03 and abs(observed_command.angular.z)<.03
                stop_samples.append({'after_result_sim_s':sim_now-initial_sim_time,'translation_speed_mps':linear_speed,
                    'translation_command_mps':linear_command,'angular_speed_rps':abs(observed.angular.z),
                    'angular_command_rps':abs(observed_command.angular.z),'within_original_threshold':stopped})
                if stopped:
                    if stable_since is None:stable_since=sim_now
                    if sim_now-stable_since>=.2:stable_stop_confirmed=True;break
                else:stable_since=None
                rclpy.spin_once(node,timeout_sec=.02)
            stop_observation={'budget_sim_s':2.0,'stable_window_sim_s':.2,'threshold':.03,
                'first_snapshot_after_1s':initial_stop_snapshot,'confirmed':stable_stop_confirmed,
                'confirmed_after_result_sim_s':clock_seconds()-initial_sim_time,'samples':stop_samples}
            (options.record_dir/'stop-observation.json').write_text(json.dumps(stop_observation,indent=2)+'\n')
            final = pose()
            truth = messages['ground_truth'].pose.pose.position
            movement = math.hypot(truth.x-initial_truth[0], truth.y-initial_truth[1])
            error = math.hypot(final.translation.x-goal_xy[0], final.translation.y-goal_xy[1])
            truth_map = (truth.x + 4.8, truth.y - 3.0)
            truth_error = math.hypot(truth_map[0]-goal_xy[0], truth_map[1]-goal_xy[1])
            velocity = messages['ground_truth'].twist.twist
            command = messages.get('cmd_vel', Twist())
            stopped_speed = max(abs(velocity.linear.x), abs(velocity.linear.y), abs(velocity.angular.z))
            stopped_command = max(abs(command.linear.x),abs(command.linear.y),abs(command.angular.z))
            translation_speed=math.hypot(velocity.linear.x,velocity.linear.y)
            translation_command=math.hypot(command.linear.x,command.linear.y)
            angular_speed=abs(velocity.angular.z)
            angular_command=abs(command.angular.z)
            still_spinning = angular_speed > .03 or angular_command > .03
            allow_arrival_spin = options.spin_mode and options.scenario == 'simple' and result.status == 4
            if allow_arrival_spin:
                stop_classification=('translation_stopped_still_spinning' if still_spinning else 'safety_stop') if translation_speed<.03 and translation_command<.03 else 'translation_moving'
            else:
                stop_classification='safety_stop' if stopped_speed<.03 and stopped_command<.03 else 'moving'
            stop_state={'mode':'spin-arrival' if allow_arrival_spin else 'strict-safety-stop',
                        'classification':stop_classification,
                        'still_spinning':bool(still_spinning),
                        'translation_speed_mps':translation_speed,'translation_command_mps':translation_command,
                        'angular_speed_rps':angular_speed,'angular_command_rps':angular_command,
                        'legacy_max_speed':stopped_speed,'legacy_max_command':stopped_command,
                        'stable_stop_confirmed':stable_stop_confirmed,'stop_observation_budget_sim_s':2.0,
                        'stop_confirmation_delay_sim_s':stop_observation['confirmed_after_result_sim_s'],
                        'threshold':.03,'source':'chassis_odometry_gt twist and cmd_vel received at final check'}
            (options.record_dir/'stop-state.json').write_text(json.dumps(stop_state,indent=2)+'\n')
            emit('result', status=result.status, ground_truth_moved=movement,
                 goal_error=error, ground_truth_goal_error=truth_error,
                 ground_truth_map_xy=truth_map, estimated_map_xy=[final.translation.x,final.translation.y],
                 ground_truth_z=truth.z, stopped_speed=stopped_speed,
                 stopped_command=stopped_command, elapsed_wall_s=time.monotonic()-started,
                 nonzero_commands=nonzero_commands, counts=counts, stop_state=stop_state)
            if options.scenario == 'blocked_route':
                assert result.status == 6, 'Blocked route did not return ABORTED without test cancellation'
                assert truth_error > .35, 'Blocked-route test unexpectedly reached target'
            else:
                assert result.status == 4, 'Navigation did not return SUCCEEDED'
                assert movement > .2, 'Ground truth does not show movement'
                assert error < .35 and truth_error < .35, 'Endpoint error exceeds .35m'
            physical = subprocess.run(['ign','model','-m','red_standard_robot1','-p'], capture_output=True, text=True, timeout=15)
            (options.record_dir/'gazebo-model-pose.txt').write_text(physical.stdout+physical.stderr)
            coordinates = re.findall(r'^\s*\[([-+0-9.eE ]+)\]\s*$',physical.stdout,re.M)
            assert physical.returncode == 0 and len(coordinates) == 2, 'Actual Gazebo model pose unavailable'
            model_xyz = [float(v) for v in coordinates[0].split()]
            model_rpy = [float(v) for v in coordinates[1].split()]
            emit('physical_pose',xyz=model_xyz,rpy=model_rpy)
            assert .02 < model_xyz[2] < .15 and max(abs(v) for v in model_rpy[:2]) < .1, 'Robot dropped, floated or tipped on the flat test area'
            assert stable_stop_confirmed, 'Stop did not remain below original thresholds for 0.2s within fixed 2.0s observation budget'
            assert time.monotonic()-started <= options.goal_timeout, 'Arrival and stable stop exceeded original goal timeout'
            if allow_arrival_spin:
                assert translation_speed < .03 and translation_command < .03, 'Translation did not stop (spin-mode; still spinning is allowed and recorded separately)'
            else:
                assert stopped_speed < .03 and stopped_command < .03, 'Robot did not stop'
            if options.scenario == 'simple':
                assert nonzero_commands > 0
            emit('PASS', scenario=options.scenario)

except Exception as error:
    try:
        physical=subprocess.run(['ign','model','-m','red_standard_robot1','-p'],capture_output=True,text=True,timeout=5)
        (options.record_dir/'failure-model-pose.txt').write_text(physical.stdout+physical.stderr)
    except Exception as physical_error:
        (options.record_dir/'failure-model-pose.txt').write_text(str(physical_error))
    emit('FAIL', error=str(error), counts=counts)
    raise
finally:
    if goal_handle is not None and goal_handle.accepted and not goal_done:
        wait_future(goal_handle.cancel_goal_async(), 5)
    transforms={}
    for parent,child in [('map','odom'),('odom','base_footprint'),('base_footprint','front_mid360')]:
        try:
            transform=buffer.lookup_transform(parent,child,rclpy.time.Time())
            t,q=transform.transform.translation,transform.transform.rotation
            transforms[parent+'->'+child]={'translation':[t.x,t.y,t.z],'rotation_xyzw':[q.x,q.y,q.z,q.w],'stamp':transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9}
        except Exception as error:
            transforms[parent+'->'+child]={'error':str(error)}
    live_costmaps={}
    for name in ['global_costmap','local_costmap']:
        try:
            client=node.create_client(GetCostmap,f'{name}/get_costmap')
            if not client.wait_for_service(timeout_sec=.5):
                continue
            future=client.call_async(GetCostmap.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=2)
            if future.done():
                grid=future.result().map
                live_costmaps[name]={'frame':grid.header.frame_id,'width':grid.metadata.size_x,'height':grid.metadata.size_y,'resolution':grid.metadata.resolution,'origin':[grid.metadata.origin.position.x,grid.metadata.origin.position.y],'data':list(grid.data)}
        except Exception as error:
            live_costmaps[name]={'error':str(error)}
    (options.record_dir/'live-costmaps.json').write_text(json.dumps(live_costmaps)+'\n')
    (options.record_dir/'tf.json').write_text(json.dumps(transforms,indent=2)+'\n')
    (options.record_dir/'paths.json').write_text(json.dumps(paths)+'\n')
    (options.record_dir/'trace.json').write_text(json.dumps(trace)+'\n')
    (options.record_dir/'maps.json').write_text(json.dumps(snapshots)+'\n')
    (options.record_dir/'topics.json').write_text(json.dumps({'counts':counts,'cloud_frames':{k:v for k,v in messages.items() if isinstance(v,dict)}})+'\n')
    node.destroy_node()
    rclpy.shutdown()
