"""Gazebo 경기장 + LIMO + 판단(ar_mission_supervisor, sim) + 차선(ar_lane_perception, sim) 한 번에 실행.

YOLO(kudos_yolo_tools)는 넣지 않는다 (ultralytics/torch 필요, NVIDIA 없는 PC에서는 안 돌 수 있음).
YOLO가 없으면 판단은 LANE_FOLLOWING 에 머물고 YOLO 미션(신호등·방지턱 서행 등)으로 안 넘어간다.

  ros2 launch ar_lane_perception sim_full.launch.py                     # 기본: hls, gui, 라이다 인지 켬
  ros2 launch ar_lane_perception sim_full.launch.py color_space:=lab gui:=false
  ros2 launch ar_lane_perception sim_full.launch.py lidar:=false light:=false
런타임 전환: ros2 param set /camera_perception_node color_space lab
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _supervisor(context, sup):
    """판단 노드. sup_extra 가 있으면 supervisor.launch.py 와 같은 파라미터 파일(params.yaml, params_sim.yaml) 뒤에
    그 yaml 을 하나 더 얹어 직접 띄운다 (판단 패키지 파일은 안 고침, 시험용 덮어쓰기)."""
    if LaunchConfiguration('supervisor').perform(context).lower() not in ('true', '1'):
        return []
    extra = LaunchConfiguration('sup_extra').perform(context)
    if not extra:
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sup / 'launch/supervisor.launch.py')),
            launch_arguments={'sim': 'true'}.items())]
    return [Node(package='ar_mission_supervisor', executable='supervisor', name='mission_supervisor',
                 output='screen',
                 parameters=[str(sup / 'config/params.yaml'), str(sup / 'config/params_sim.yaml'), extra])]


def generate_launch_description():
    arena = Path(get_package_share_directory('kudos_arena'))
    lane = Path(get_package_share_directory('ar_lane_perception'))
    sup = Path(get_package_share_directory('ar_mission_supervisor'))
    per = Path(get_package_share_directory('ar_perception'))
    L = LaunchConfiguration
    args = {
        'gui': ('true', 'false 면 gzclient 없이 (가볍게)'),
        'branch_sign': ('A', 'random | A | B'),
        'color_space': ('hls', 'hls | lab'),
        'lane_extra': ('', '차선 노드에 추가로 덮어쓸 yaml (예: share/ar_lane_perception/config/sim_baseline.yaml)'),
        'supervisor': ('true', 'false 면 판단 노드 없이 (scripts/simple_follow.py 등으로 단독 평가)'),
        'sup_extra': ('', '판단 노드에 추가로 덮어쓸 yaml (시험용, 예: base_speed 바꾸기)'),
        'lidar': ('true', 'ar_perception lidar_perception 실행'),
        'light': ('true', '신호등 컨트롤러 실행 (신호 순환)'),
        'x': ('0.6', ''), 'y': ('5.58', ''), 'z': ('0.03', ''), 'yaw': ('0.0', ''),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in args.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(arena / 'launch/competition.launch.py')),
            launch_arguments={k: L(k) for k in ('gui', 'branch_sign', 'x', 'y', 'z', 'yaw')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lane / 'launch/camera_perception.launch.py')),
            launch_arguments={'sim': 'true', 'color_space': L('color_space'),
                              'lane_extra': L('lane_extra')}.items()),
        OpaqueFunction(function=_supervisor, args=[sup]),
        Node(package='ar_perception', executable='lidar_perception', name='lidar_perception',
             output='screen', arguments=['--ros-args', '--log-level', 'warn'],
             parameters=[str(per / 'config/lidar.yaml'), {'use_sim_time': True}],
             condition=IfCondition(L('lidar'))),
        Node(package='kudos_arena', executable='traffic_light_controller.py',
             name='traffic_light_controller', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(L('light'))),
    ])
