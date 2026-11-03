"""Run current LiDAR and team supervisor in the competition Gazebo world."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arena = Path(get_package_share_directory('kudos_arena'))
    perception = Path(get_package_share_directory('ar_perception'))
    supervisor = Path(get_package_share_directory('ar_mission_supervisor'))
    defaults = {'x': '0.72', 'y': '1.50', 'z': '0.03', 'yaw': '1.5708',
                'gui': 'true'}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(arena / 'launch/competition.launch.py')),
            launch_arguments={k: LaunchConfiguration(k) for k in defaults}.items()),
        Node(package='ar_perception', executable='lidar_perception',
             name='lidar_perception', output='screen',
             arguments=['--ros-args', '--log-level', 'warn'],
             parameters=[str(perception / 'config/lidar.yaml'), {'use_sim_time': True}]),
        # No camera in this cone-only test: stop outside LiDAR missions.
        Node(package='ar_mission_supervisor', executable='supervisor',
             name='mission_supervisor', output='screen', parameters=[
                 str(supervisor / 'config/params.yaml'),
                 str(supervisor / 'config/params_sim.yaml'),
                 {'use_sim_time': True, 'failsafe.creep_speed': 0.0,
                  'failsafe.stop_after_sec': 0.0}]),
    ])
