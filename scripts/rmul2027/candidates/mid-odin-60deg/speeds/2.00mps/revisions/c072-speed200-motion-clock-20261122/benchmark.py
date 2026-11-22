"""Drive Nav2 goals and record independent Gazebo speed and contact evidence."""
import collections
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

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


class ContactRecorder:
    def __init__(self, output, active_leg):
        self.counts = collections.Counter()
        self.message_counts = collections.Counter()
        self.errors = collections.Counter()
        self.active_leg = active_leg
        self.workers = []
        self.output = output

    def start(self, topics):
        for topic in topics:
            slug = topic.rsplit('/', 1)[1]
            logfile = (self.output / (slug + '-contact-cli.log')).open('w')
            child = subprocess.Popen(
                ['stdbuf', '-oL', 'ign', 'topic', '-e', '-t', topic, '--json-output'],
                stdout=subprocess.PIPE, stderr=logfile, text=True, bufsize=1)
            worker = threading.Thread(target=self.read, args=(child, slug), daemon=True)
            self.workers.append((child, worker, logfile))
            worker.start()

    def read(self, child, source):
        with (self.output / (source + '-contacts.jsonl')).open('w', buffering=1) as evidence:
            for line in child.stdout:
                if not line.strip():
                    continue
                try:
                    message = json.loads(line)
                    contacts = message.get('contact', [])
                    self.message_counts[source] += 1
                    for contact in contacts:
                        self.record(evidence, source, contact)
                except (ValueError, TypeError, AttributeError):
                    self.errors[source] += 1
                    if self.errors[source] <= 3:
                        print('contact decode', source, repr(line[:240]), flush=True)

    def record(self, evidence, source, contact):
        names = [contact.get(key, {}).get('name', '') for key in ('collision1', 'collision2')]
        if not any('red_standard_robot1' in name for name in names):
            return
        points = contact.get('position', [])
        if source == 'cart_contacts':
            kind = 'box'
        elif not points:
            kind = 'unknown'
        else:
            height = max(point.get('z', 0.) for point in points)
            wheel = any('_wheel_collision' in name for name in names)
            if height <= .025 and wheel:
                kind = 'ground'
            elif height > .025:
                kind = 'wall'
            else:
                kind = 'unknown'
        leg = self.active_leg()
        key = (leg, source, kind)
        self.counts[key] += 1
        if self.counts[key] <= 4 or kind not in ('ground', 'elevated'):
            evidence.write(json.dumps(dict(leg=leg, kind=kind, wall_clock=time.time(),
                                           contact=contact)) + '\n')

    def totals(self, leg):
        return {kind: sum(count for (key_leg, _, key_kind), count in self.counts.copy().items()
                          if key_leg == leg and key_kind == kind)
                for kind in ('ground', 'wall', 'box', 'elevated', 'unknown')}

    def close(self):
        for child, _, _ in self.workers:
            if child.poll() is None:
                child.terminate()
        for child, worker, logfile in self.workers:
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            worker.join(timeout=2)
            logfile.close()
        report = dict(messages=dict(self.message_counts), malformed=dict(self.errors),
                      counts=[dict(leg=leg, source=src, kind=kind, records=count)
                              for (leg, src, kind), count in sorted(self.counts.items())])
        (self.output / 'contact-summary.json').write_text(json.dumps(report, indent=2) + '\n')


def run(output):
    routes = json.loads((output / 'routes.json').read_text())
    rclpy.init(args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
    node = Node('long_route_benchmark', namespace='/red_standard_robot1',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf = Buffer()
    listener = TransformListener(tf, node)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    permit = node.create_publisher(Bool, 'decision_stop', 1)
    truth = {}
    commands = {}
    active = None
    stopping = True
    leg = 'setup'
    rows = []
    results = []
    summary = output / 'results.json'
    sample_log = (output / 'samples.jsonl').open('w', buffering=1)
    feedback_log = (output / 'odometry-feedback.jsonl').open('w', buffering=1)
    smoother_log = (output / 'smoothed-command.jsonl').open('w', buffering=1)
    last_stamp = -math.inf
    contacts = ContactRecorder(output, lambda: leg)

    def odometry(message):
        nonlocal last_stamp
        pos = message.pose.pose.position
        orientation = message.pose.pose.orientation
        twist = message.twist.twist
        stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
        truth.update(x=pos.x + 4.8, y=pos.y - 3., z=pos.z,
                     yaw=math.atan2(2*(orientation.w*orientation.z+orientation.x*orientation.y),
                                    1-2*(orientation.y**2+orientation.z**2)),
                     speed=math.hypot(twist.linear.x, twist.linear.y), angular=abs(twist.angular.z),
                     sim=stamp, received=time.monotonic())
        if stamp <= last_stamp:
            return
        last_stamp = stamp
        record = {k: v for k, v in truth.items() if k != 'received'}
        record.update(leg=leg, **{key: value['speed'] for key, value in commands.items()})
        rows.append(record)
        sample_log.write(json.dumps(record) + '\n')

    def velocity(message, key):
        commands[key] = dict(speed=math.hypot(message.linear.x, message.linear.y),
                             angular=abs(message.angular.z), received=time.monotonic())

    def until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(leg)
            rclpy.spin_once(node, timeout_sec=.04)

    def response(future, timeout):
        until(future.done, timeout)
        return future.result()

    def one_second_stop(timeout):
        first_stop = None

        def stable():
            nonlocal first_stop
            values = [truth, commands.get('output')]
            fresh = all(v and time.monotonic() - v['received'] < .35 for v in values)
            if not fresh or max(max(v['speed'], v['angular']) for v in values) >= .03:
                first_stop = None
                return False
            if first_stop is None:
                first_stop = truth['sim']
            return truth['sim'] - first_stop >= 1.

        until(stable, timeout)

    def feedback(message):
        velocity = message.twist.twist.linear
        p = message.pose.pose.position
        stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
        feedback_log.write(json.dumps(dict(leg=leg, sim=stamp, v=[velocity.x, velocity.y],
                                           xy=[p.x, p.y])) + '\n')

    node.create_subscription(Odometry, 'odometry', feedback, qos_profile_sensor_data)
    node.create_subscription(Odometry, 'chassis_odometry_gt', odometry, qos_profile_sensor_data)
    def smoothed(message):
        v = message.linear
        stamp = node.get_clock().now().nanoseconds / 1e9
        smoother_log.write(json.dumps(dict(leg=leg, sim=stamp, v=[v.x, v.y])) + '\n')
    node.create_subscription(Twist, 'cmd_vel_nav2_result', smoothed, 10)
    node.create_subscription(Twist, 'cmd_vel', lambda msg: velocity(msg, 'output'), 10)
    node.create_subscription(Twist, 'cmd_vel_controller', lambda msg: velocity(msg, 'controller'), 10)
    node.create_timer(.05, lambda: permit.publish(Bool(data=stopping)))
    try:
        until(lambda: truth and tf.can_transform('map', 'gimbal_yaw_fake', rclpy.time.Time()), 35)
        if node.count_publishers('decision_stop') != 1 or not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Unexpected stop publishers or missing NavigateToPose')
        advertised = set((output / 'gazebo-topics.log').read_text().splitlines())
        topics = ['/rmul_route_test/field/collision_part_000', '/rmul_route_test/field/collision_part_001',
                  '/c020/cart_contacts']
        if not set(topics).issubset(advertised):
            raise RuntimeError('Missing Gazebo contact topics: ' + str(set(topics) - advertised))
        contacts.start(topics)
        until(lambda: contacts.totals('setup')['ground'] >= 10, 15)
        one_second_stop(10)
        print(json.dumps({'start': [truth['x'], truth['y']], 'contact_startup': contacts.totals('setup')}), flush=True)
        for leg, x, y in routes['goals']:
            start = [truth['x'], truth['y']]
            start_sim = truth['sim']
            start_index = len(rows)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = (x, y)
            goal.pose.pose.orientation = tf.lookup_transform(
                'map', 'gimbal_yaw_fake', rclpy.time.Time()).transform.rotation
            stopping = False
            status = None
            stopped = False
            failure = None
            print(json.dumps(dict(leg=leg, start=start, goal=[x, y], sim=start_sim, started_at=time.time())), flush=True)
            try:
                active = response(client.send_goal_async(goal), 10)
                if not active.accepted:
                    raise RuntimeError('goal rejected')
                status = response(active.get_result_async(), 100).status
                active = None
                if status != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError('Nav2 status ' + str(status))
                one_second_stop(12)
                stopped = True
            except (RuntimeError, TimeoutError) as error:
                failure = repr(error)
                stopping = True
                if active is not None and active.accepted:
                    response(active.cancel_goal_async(), 7)
                active = None
                one_second_stop(10)

            samples = rows[start_index:]
            end = [truth['x'], truth['y']]
            peak = max(samples, key=lambda row: row['speed'], default=truth)
            goal_error = math.dist(end, [x, y])
            statistics = dict(leg=leg, start=start, goal=[x, y], end=end,
                              direct_m=math.dist(start, [x, y]),
                              travelled_m=sum(math.hypot(b['x']-a['x'], b['y']-a['y'])
                                              for a, b in zip(samples, samples[1:])),
                              start_sim=start_sim, end_sim=truth['sim'],
                              samples=len(samples), goal_status=status, error=failure,
                              stop_verified=stopped, goal_error_m=goal_error,
                              truth_peak_mps=peak['speed'], peak_at_sim=peak['sim'],
                              peak_at_xy=[peak['x'], peak['y']],
                              controller_peak_mps=max((row.get('controller', 0.) for row in samples), default=0.),
                              output_peak_mps=max((row.get('output', 0.) for row in samples), default=0.),
                              contacts=contacts.totals(leg))
            results.append(statistics)
            print(json.dumps(statistics), flush=True)
            summary.write_text(json.dumps(results, indent=2) + '\n')
    finally:
        stopping = True
        permit.publish(Bool(data=True))
        try:
            if active is not None and active.accepted:
                response(active.cancel_goal_async(), 5)
        finally:
            contacts.close()
            smoother_log.close()
            feedback_log.close()
            sample_log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run(Path(sys.argv[1]))
