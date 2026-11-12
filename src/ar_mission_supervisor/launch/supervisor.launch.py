import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config = os.path.join(get_package_share_directory('ar_mission_supervisor'), 'config')
    params = [os.path.join(config, 'params.yaml')]

    sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1', 'yes')
    if sim:
        # 기본값 위에 시뮬레이터용 값만 덮어쓴다
        params.append(os.path.join(config, 'params_sim.yaml'))

    # (11/12) 실차 첫 시험처럼 기본 속도만 바꿔 볼 때: base_speed:=0.2
    base = LaunchConfiguration('base_speed').perform(context).strip()
    if base:
        params.append({'base_speed': float(base)})

    return [Node(package='ar_mission_supervisor',
                 executable='supervisor',
                 name='mission_supervisor',
                 output='screen',
                 parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true 면 Gazebo 용 파라미터를 덮어씀'),
        DeclareLaunchArgument('base_speed', default_value='',
                              description='비우면 params.yaml 값. 예: 0.2 (실차 첫 시험)'),
        OpaqueFunction(function=_launch_setup),
    ])