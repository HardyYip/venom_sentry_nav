from array import array
import json
import math
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener


def inside_footprint(x, y, polygon):
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        cross = (x - a[0]) * dy - (y - a[1]) * dx
        if abs(cross) <= 1e-9 and min(a[0], b[0]) - 1e-9 <= x <= max(a[0], b[0]) + 1e-9 and min(a[1], b[1]) - 1e-9 <= y <= max(a[1], b[1]) + 1e-9:
            return True
        if (a[1] > y) != (b[1] > y) and x < a[0] + (y - a[1]) * dx / dy:
            inside = not inside
    return inside


class RplidarObstacleCloud(Node):
    def __init__(self):
        super().__init__('rplidar_obstacle_cloud')
        self.self_filter_enabled = self.declare_parameter('self_filter_enabled', False).value
        self.self_filter_frame = self.declare_parameter('self_filter_frame', 'base_footprint').value
        footprint = self.declare_parameter('self_filter_footprint', '[[0.29,0.25],[0.29,-0.25],[-0.29,-0.25],[-0.29,0.25]]').value
        self.footprint = json.loads(footprint)
        if len(self.footprint) < 3 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in self.footprint):
            raise ValueError('Self-filter footprint must contain finite XY vertices')
        self.buffer = Buffer(cache_time=Duration(seconds=10), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointCloud2, 'rplidar_obstacles', qos_profile_sensor_data)
        self.subscription = self.create_subscription(LaserScan, 'rplidar_a2/scan', self.convert, qos_profile_sensor_data)
        self.counters = {'scans': 0, 'published': 0, 'removed_self_points': 0, 'missing_transform_scans': 0}
        self.create_timer(5.0, self.report)

    def report(self):
        self.get_logger().info(json.dumps({'event': 'rplidar_self_filter', 'enabled': self.self_filter_enabled, **self.counters}))

    def convert(self, scan):
        self.counters['scans'] += 1
        transform = None
        if self.self_filter_enabled:
            try:
                transform = self.buffer.lookup_transform(self.self_filter_frame, scan.header.frame_id, rclpy.time.Time.from_msg(scan.header.stamp)).transform
            except Exception as error:
                self.counters['missing_transform_scans'] += 1
                self.get_logger().warning('Self-filter source-stamped TF unavailable; scan not published: ' + str(error), throttle_duration_sec=5.0)
                return
        values = array('f')
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            x, y = distance * math.cos(angle), distance * math.sin(angle)
            if transform is not None:
                q, t = transform.rotation, transform.translation
                bx = (1 - 2 * (q.y * q.y + q.z * q.z)) * x + 2 * (q.x * q.y - q.z * q.w) * y + t.x
                by = 2 * (q.x * q.y + q.z * q.w) * x + (1 - 2 * (q.x * q.x + q.z * q.z)) * y + t.y
                if inside_footprint(bx, by, self.footprint):
                    self.counters['removed_self_points'] += 1
                    continue
            values.extend((x, y, 0.0, 1.0))
        cloud = PointCloud2()
        cloud.header = scan.header
        cloud.height, cloud.width = 1, len(values) // 4
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1) for i, name in enumerate(['x', 'y', 'z', 'intensity'])]
        cloud.is_bigendian = sys.byteorder == 'big'
        cloud.point_step, cloud.row_step = 16, cloud.width * 16
        cloud.is_dense = True
        cloud.data = values.tobytes()
        self.publisher.publish(cloud)
        self.counters['published'] += 1


def main():
    rclpy.init()
    node = RplidarObstacleCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
