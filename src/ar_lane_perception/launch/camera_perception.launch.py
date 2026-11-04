"""camera_perception_node 실행.

실차:  ros2 launch ar_lane_perception camera_perception.launch.py
시뮬:  ros2 launch ar_lane_perception camera_perception.launch.py sim:=true
       (image_topic 기본값이 /rgb/image_raw, lane_width_m 0.52, use_sim_time true)
다른 토픽: image_topic:=/my/image
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REAL_TOPIC = '/camera/color/image_raw'
SIM_TOPIC = '/rgb/image_raw'


def _setup(context):
    share = Path(get_package_share_directory('ar_lane_perception'))
    sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1', 'yes')
    topic = LaunchConfiguration('image_topic').perform(context) or (SIM_TOPIC if sim else REAL_TOPIC)
    params = [str(share / 'config/camera_perception.yaml')]
    if sim:
        params.append(str(share / 'config/camera_perception_sim.yaml'))
    params.append({'image_topic': topic, 'use_sim_time': sim})
    return [Node(package='ar_lane_perception', executable='camera_perception_node',
                 name='camera_perception_node', output='screen', parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true: Gazebo (camera_perception_sim.yaml, use_sim_time)'),
        DeclareLaunchArgument('image_topic', default_value='',
                              description='비우면 sim에 따라 %s / %s' % (SIM_TOPIC, REAL_TOPIC)),
        OpaqueFunction(function=_setup),
    ])
