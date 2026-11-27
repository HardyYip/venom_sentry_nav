"""Exercise the cart's velocity smoother with a diagonal overspeed request in simulation."""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts

out = Path(sys.argv[1])
rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
node = Node('cart_speed_bench')
publisher = node.create_publisher(Twist, 'nav_cmd', 10)
commands = []
odometry = []
contacts = []
now = lambda: node.get_clock().now().nanoseconds * 1e-9

def read_pose(msg):
    stamp = msg.header.stamp
    stamp_s = stamp.sec + stamp.nanosec * 1e-9
    odometry.append([stamp_s, msg.twist.twist.linear.x, msg.twist.twist.linear.y])

node.create_subscription(Odometry, '/cart/odom', read_pose, 100)
node.create_subscription(Twist, '/cart/cmd',
                         lambda msg: commands.append([now(), msg.linear.x, msg.linear.y]), 100)
node.create_subscription(Contacts, '/world/navigation/model/cart/link/body/sensor/contacts/contact',
                         lambda msg: contacts.extend([now()] * len(msg.contacts)), 100)
start = None
sent_at = -1
deadline = time.monotonic() + 60
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.02)
    clock = now()
    if not odometry or not publisher.get_subscription_count() or not clock:
        continue
    if start is None:
        start = clock
    elapsed = clock - start
    if clock - sent_at >= 0.04:
        request = Twist()
        if 2 <= elapsed < 8:
            request.linear.x = 2.0
            request.linear.y = 2.0
        publisher.publish(request)
        sent_at = clock
    if elapsed >= 12:
        break
else:
    raise RuntimeError('timeout waiting for cart and smoother')
node.destroy_node()
rclpy.shutdown()
(out / 'ramp-trace.json').write_text(json.dumps({'start': start, 'commands': commands,
                                                 'odometry': odometry, 'contacts': contacts}))
a, b = np.array(commands), np.array(odometry)
speeds = np.linalg.norm(b[:, 1:], axis=1)
cruise = b[(b[:, 0] >= start + 5) & (b[:, 0] <= start + 7.8)]
stopped = b[b[:, 0] >= start + 10.5]
assert len(cruise) >= 100 and len(stopped) >= 45
duration = float(cruise[-1, 0] - cruise[0, 0])
step = float(np.max(np.linalg.norm(np.diff(a[:, 1:], axis=0), axis=1)))
result = {'requested_mps': math.sqrt(8), 'max_odom_mps': float(speeds.max()),
          'cruise_min_mps': float(np.linalg.norm(cruise[:, 1:], axis=1).min()),
          'cruise_seconds': duration, 'max_cmd_mps': float(np.linalg.norm(a[:, 1:],axis=1).max()),
          'max_step_mps': step, 'max_stopped_mps': float(np.linalg.norm(stopped[:, 1:],axis=1).max()),
          'contacts': len(contacts)}
result['passed'] = (duration >= 2.7 and result['cruise_min_mps'] >= 2.499 and
                    result['max_odom_mps'] <= 2.501 and result['max_cmd_mps'] <= 2.500001 and
                    step <= 1.75 / 20 + 1e-8 and result['max_stopped_mps'] < .03 and not contacts)
(out/'ramp-result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
sys.exit(0 if result['passed'] else 1)
