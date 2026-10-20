import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    folder = Path(__file__).resolve().parent
    simulator = Path(get_package_share_directory('rmu_gazebo_simulator'))
    bridge = Path(get_package_share_directory('ros_gz_sim'))
    resource_path = os.pathsep.join([str(folder / 'scene' / 'models'),
                                    os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    world = folder / 'scene' / 'worlds' / 'rmul_2027_3v3_world.sdf'
    return LaunchDescription([
        DeclareLaunchArgument('spawn_robot', default_value='True'),
        DeclareLaunchArgument('world_sdf', default_value=str(world)),
        DeclareLaunchArgument('gz_world_path', default_value=str(folder / 'config' / 'gz_world.yaml')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        ExecuteProcess(cmd=[sys.executable, str(folder / 'publish_field_reference.py')], output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bridge / 'launch' / 'gz_sim.launch.py')),
            launch_arguments={'gz_version': '6',
                              'gz_args': ['-r -s --headless-rendering ',LaunchConfiguration('world_sdf')]}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(simulator / 'launch' / 'spawn_robots.launch.py')),
            condition=IfCondition(LaunchConfiguration('spawn_robot')),
            launch_arguments={'gz_world_path': LaunchConfiguration('gz_world_path')}.items()),
    ])
