"""LiDAR perception only; hardware drivers and vehicle control start separately."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('ar_perception'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Set true only for rosbag replay with /clock'),
        Node(package='ar_perception', executable='lidar_perception',
             name='lidar_perception', output='screen',
             parameters=[str(share / 'config/lidar.yaml'),
                         str(share / 'config/lidar_real.yaml'),
                         {'use_sim_time': ParameterValue(
                             LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
