from storage_entry import ensure_storage_guard
ensure_storage_guard(__file__)

import argparse
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--record-dir', type=Path, required=True)
parser.add_argument('--topic', choices=['/red_standard_robot1/livox/lidar', '/red_standard_robot1/velodyne_points'], default='/red_standard_robot1/livox/lidar')
args = parser.parse_args()
args.record_dir.mkdir(parents=True, exist_ok=False)
subprocess.run(['ros2', 'bag', 'record', '--max-cache-size', '67108864',
                '--max-bag-size', '33554432', '-o', str(args.record_dir / 'inputs'),
                '/clock', '/red_standard_robot1/tf', '/red_standard_robot1/tf_static',
                '/red_standard_robot1/livox/imu', args.topic], check=True)
