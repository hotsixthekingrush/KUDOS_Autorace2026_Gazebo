"""camera_perception_node 실행.

실차:  ros2 launch ar_lane_perception camera_perception.launch.py
시뮬:  ros2 launch ar_lane_perception camera_perception.launch.py sim:=true
       (image_topic 기본값이 /rgb/image_raw, use_sim_time true)
다른 토픽: image_topic:=/my/image
색 필터:   color_space:=lab (비우면 yaml 값)
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
    extra = {'image_topic': topic, 'use_sim_time': sim}
    cs = LaunchConfiguration('color_space').perform(context)
    if cs:
        extra['color_space'] = cs
    params.append(extra)
    more = LaunchConfiguration('lane_extra').perform(context)
    # 맨 마지막에 덮어씀 (예: config/sim_baseline.yaml). 쉼표로 여러 개: 앞에서부터 차례로 덧씌움
    params += [m.strip() for m in more.split(',') if m.strip()]
    return [Node(package='ar_lane_perception', executable='camera_perception_node',
                 name='camera_perception_node', output='screen', parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true: Gazebo (camera_perception_sim.yaml, use_sim_time)'),
        DeclareLaunchArgument('image_topic', default_value='',
                              description='비우면 sim에 따라 %s / %s' % (SIM_TOPIC, REAL_TOPIC)),
        DeclareLaunchArgument('lane_extra', default_value='',
                              description='추가로 덮어쓸 파라미터 yaml 경로 (쉼표로 여러 개, 뒤가 우선)'),
        DeclareLaunchArgument('color_space', default_value='',
                              description='hls | lab. 비우면 yaml 값'),
        OpaqueFunction(function=_setup),
    ])
