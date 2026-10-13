"""Headless wrapper around the official simulator's robot and bridge launch files."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulator = get_package_share_directory('rmu_gazebo_simulator')
    bridge = get_package_share_directory('ros_gz_sim')
    with open(os.path.join(simulator, 'config', 'gz_world.yaml'), encoding='utf-8') as stream:
        world = yaml.safe_load(stream)['world']
    if world != 'rmuc_2025':
        raise RuntimeError('This baseline expects the official rmuc_2025 world.')
    world_path = os.path.join(simulator, 'resource', 'worlds', f'{world}_world.sdf')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bridge, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_version': '6',
                'gz_args': f'-r -s --headless-rendering {world_path}',
            }.items(),
        ),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(simulator, 'launch', 'spawn_robots.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(simulator, 'launch', 'referee_system.launch.py'))),
    ])
