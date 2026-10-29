"""Spawn the existing LIMO model in the new arena without changing limo_car."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arena = Path(get_package_share_directory('kudos_arena'))
    car = Path(get_package_share_directory('limo_car'))
    config = json.loads((arena / 'config/track.json').read_text())
    spawn = config['robot_spawn']
    robot = ET.fromstring(xacro.process_file(
        str(car / 'gazebo/ackermann_with_sensor.xacro')
    ).toxml())

    # Remove legacy ROS1 control and invalid optional steering-wheel joint
    # only from this launch's expanded description, preserving source files.
    for gazebo in robot.findall('gazebo'):
        for plugin in list(gazebo.findall('plugin')):
            if plugin.get('filename') == 'libgazebo_ros_control.so':
                gazebo.remove(plugin)
            elif plugin.get('filename') == 'libgazebo_ros_ackermann_drive.so':
                for joint in list(plugin.findall('steering_wheel_joint')):
                    if (joint.text or '').strip() == '/':
                        plugin.remove(joint)

    arguments = [DeclareLaunchArgument(key, default_value=str(spawn[key]))
                 for key in ('x', 'y', 'z', 'yaw')]
    return LaunchDescription(arguments + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(arena / 'launch/arena.launch.py'))),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[{
                'robot_description': ET.tostring(robot, encoding='unicode'),
                'use_sim_time': True,
            }],
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=[
                '-entity', 'limo', '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw'),
                '-timeout', '120',
            ],
        ),
    ])
