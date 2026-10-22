import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from rplidar_obstacle_cloud import RplidarObstacleCloud, inside_footprint

parser = argparse.ArgumentParser()
parser.add_argument('--record-dir', type=Path, required=True)
args = parser.parse_args()
args.record_dir.mkdir(exist_ok=False, parents=True)
rclpy.init(args=['--ros-args', '-p', 'self_filter_enabled:=true'])
node = RplidarObstacleCloud()
received = []
sub = node.create_subscription(PointCloud2, 'rplidar_obstacles', received.append, qos_profile_sensor_data)

def spin(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)

def transform(frame, stamp, tx):
    msg = TransformStamped()
    msg.header.frame_id = 'base_footprint'
    msg.child_frame_id = frame
    msg.header.stamp.sec = stamp
    msg.transform.translation.x = tx
    msg.transform.rotation.w = 1.0
    return msg

def scan(frame, ranges, stamp=10):
    msg = LaserScan()
    msg.header.frame_id = frame
    msg.header.stamp.sec = stamp
    msg.angle_min = 0.0
    msg.angle_increment = math.pi / 2
    msg.range_min, msg.range_max = .05, 12.
    msg.ranges = ranges
    return msg

checks = {}
try:
    node.buffer.set_transform_static(transform('laser', 0, .155), 'contract')
    spin(.8)
    node.convert(scan('laser', [.1, .3, .6]))
    spin(.3)
    assert len(received) == 1
    cloud = received[-1]
    pts = point_cloud2.read_points(cloud, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
    assert cloud.width == 2
    assert abs(float(pts['y'][0]) - .3) < 1e-6 and abs(float(pts['x'][1]) + .6) < 1e-6
    assert cloud.header.frame_id == 'laser' and cloud.header.stamp.sec == 10 and all(float(v) == 1. for v in pts['intensity'])
    checks['self_removed_external_preserved_header_and_intensity_preserved'] = True
    node.buffer.set_transform(transform('dynamic_laser', 10, .155), 'contract')
    node.buffer.set_transform(transform('dynamic_laser', 11, .555), 'contract')
    node.convert(scan('dynamic_laser', [.1], 10))
    spin(.3)
    assert len(received) == 2 and received[-1].width == 0
    checks['source_timestamp_used_instead_of_latest_transform'] = True
    node.convert(scan('unknown_laser', [.6]))
    spin(.3)
    assert len(received) == 2 and node.counters['missing_transform_scans'] == 1
    checks['missing_transform_drops_scan_instead_of_unfiltered_fallback'] = True
    node.self_filter_enabled = False
    node.convert(scan('unknown_laser', [.1, .3, .6]))
    spin(.3)
    assert len(received) == 3 and received[-1].width == 3
    checks['legacy_disabled_filter_retains_all_points_and_needs_no_tf'] = True
    polygon = [[.29, .25], [.29, -.25], [-.29, -.25], [-.29, .25]]
    assert all(inside_footprint(x, y, polygon) for x, y in [(0, 0), (.29, .25), (-.29, -.1), (0, .25)])
    assert all(not inside_footprint(x, y, polygon) for x, y in [(.291, 0), (0, .251), (-.3, -.3)])
    checks['polygon_boundary_inside_external_points_outside'] = True
    result = {'passed': True, 'checks': checks, 'runtime_counters': node.counters}
    (args.record_dir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
