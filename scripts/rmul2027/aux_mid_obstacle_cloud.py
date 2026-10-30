import json
import math

import numpy as np

FOOTPRINT = [[.29, .25], [.29, -.25], [-.29, -.25], [-.29, .25]]


def filter_points(points, rotation, translation, *, footprint=FOOTPRINT,
                  min_height=.06, max_height=2., min_range=.2, max_range=5., bins=400):
    points = np.asarray(points, dtype=float).reshape((-1, 3))
    rotation, translation = np.asarray(rotation, dtype=float), np.asarray(translation, dtype=float)
    if rotation.shape != (3, 3) or translation.shape != (3,) or not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError('Invalid source-time transform')
    if not isinstance(bins, int) or bins < 1 or bins > 10000:
        raise ValueError('Invalid angular bin count')
    if not all(math.isfinite(x) for x in (min_height, max_height, min_range, max_range)) or not (0 <= min_range < max_range and min_height < max_height):
        raise ValueError('Invalid filter limits')
    polygon = np.asarray(footprint, dtype=float)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3 or not np.isfinite(polygon).all():
        raise ValueError('Invalid footprint')
    total = len(points)
    points = points[np.isfinite(points).all(axis=1)]
    distances = np.linalg.norm(points, axis=1)
    keep = (distances >= min_range) & (distances <= max_range)
    points, distances = points[keep], distances[keep]
    base = points @ rotation.T + translation
    keep = (base[:, 2] >= min_height) & (base[:, 2] <= max_height)
    points, distances, base = points[keep], distances[keep], base[keep]
    inside, boundary = np.zeros(len(base), dtype=bool), np.zeros(len(base), dtype=bool)
    x, y = base[:, 0], base[:, 1]
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        dx, dy = b - a
        cross = (x - a[0]) * dy - (y - a[1]) * dx
        boundary |= (abs(cross) <= 1e-9) & (x >= min(a[0], b[0])-1e-9) & (x <= max(a[0], b[0])+1e-9) & (y >= min(a[1], b[1])-1e-9) & (y <= max(a[1], b[1])+1e-9)
        if abs(dy) > 1e-15:
            inside ^= ((a[1] > y) != (b[1] > y)) & (x < a[0] + (y-a[1])*dx/dy)
    points, distances = points[~(inside | boundary)], distances[~(inside | boundary)]
    angles = np.floor((np.arctan2(points[:, 1], points[:, 0])+math.pi)*bins/(2*math.pi)).astype(int) % bins
    order = np.argsort(distances, kind='stable')
    _, first = np.unique(angles[order], return_index=True)
    selected = points[order[first]]
    return selected, dict(input_points=total, output_points=len(selected))


def xyz_from_cloud(cloud):
    formats = {7: 'f4', 8: 'f8'}
    fields = {f.name: f for f in cloud.fields}
    if any(k not in fields or fields[k].datatype not in formats or fields[k].count != 1 for k in ('x', 'y', 'z')):
        raise ValueError('Expected scalar floating XYZ')
    endian = '>' if cloud.is_bigendian else '<'
    dtype = np.dtype(dict(names=['x', 'y', 'z'],
        formats=[endian+formats[fields[k].datatype] for k in ('x', 'y', 'z')],
        offsets=[fields[k].offset for k in ('x', 'y', 'z')], itemsize=cloud.point_step))
    if cloud.row_step < cloud.width*cloud.point_step or len(cloud.data) < cloud.row_step*cloud.height:
        raise ValueError('Truncated or invalid cloud layout')
    values = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                        strides=(cloud.row_step, cloud.point_step))
    return np.column_stack([values[k].ravel() for k in ('x', 'y', 'z')])


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = x*x+y*y+z*z+w*w
    if not math.isfinite(norm) or abs(norm-1) > .001:
        raise ValueError('Invalid TF quaternion')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import Buffer, TransformListener

    class AuxiliaryCloud(Node):
        def __init__(self):
            super().__init__('aux_mid_obstacle_cloud')
            self.expected_frame = self.declare_parameter('source_frame', 'aux_mid360').value
            self.target_frame = self.declare_parameter('filter_frame', 'base_footprint').value
            self.limits = {k: self.declare_parameter(k, v).value for k, v in
                           dict(min_height=.06, max_height=2., min_range=.2, max_range=5., bins=400).items()}
            self.limits['footprint'] = json.loads(self.declare_parameter('footprint', json.dumps(FOOTPRINT)).value)
            filter_points([], np.eye(3), np.zeros(3), **self.limits)
            self.buffer = Buffer(cache_time=Duration(seconds=10), node=self)
            self.listener = TransformListener(self.buffer, self)
            self.pub = self.create_publisher(PointCloud2, 'auxiliary/obstacles', qos_profile_sensor_data)
            self.sub = self.create_subscription(PointCloud2, 'auxiliary/livox/lidar', self.convert, qos_profile_sensor_data)
            self.counts = dict(received=0, published=0, dropped=0, input_points=0, output_points=0)
            self.create_timer(5., lambda: self.get_logger().info(json.dumps(self.counts)))

        def convert(self, msg):
            self.counts['received'] += 1
            try:
                if msg.header.frame_id != self.expected_frame or (msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0):
                    raise ValueError('Wrong frame or zero source stamp')
                tf = self.buffer.lookup_transform(self.target_frame, msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp)).transform
                points, counts = filter_points(xyz_from_cloud(msg), rotation_matrix(tf.rotation),
                    [tf.translation.x, tf.translation.y, tf.translation.z], **self.limits)
                values = np.column_stack([points, np.ones(len(points))]).astype('<f4')
                cloud = PointCloud2()
                cloud.header = msg.header
                cloud.height, cloud.width = 1, len(points)
                cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
                cloud.is_bigendian = False
                cloud.point_step, cloud.row_step = 16, 16*len(points)
                cloud.is_dense = True
                cloud.data = values.tobytes()
                self.pub.publish(cloud)
                self.counts['published'] += 1
                for k, v in counts.items():
                    self.counts[k] += v
            except Exception as error:
                self.counts['dropped'] += 1
                self.get_logger().warning('Source-stamped auxiliary frame dropped: '+str(error), throttle_duration_sec=5.)

    rclpy.init()
    node = AuxiliaryCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(json.dumps(node.counts))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
