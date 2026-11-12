"""
Gazebo 에서 YOLO 추론 노드 실행 (판단팀 연동 테스트용)

  ros2 launch kudos_yolo_tools yolo_detector_sim.launch.py            (기본 모델 models/best_sim_full_1111.pt)
  ros2 launch kudos_yolo_tools yolo_detector_sim.launch.py weights:=/절대경로/다른모델.pt
  ros2 launch kudos_yolo_tools yolo_detector_sim.launch.py cpu_threads:=0   (CPU 스레드 제한 안 함)
  rqt_image_view /perception/yolo_debug
  ros2 topic echo /perception/mission_objects

cpu_threads (기본 2): YOLO 가 CPU 코어를 다 잡아 차선 노드가 밀리지 않게 OMP/BLAS/OpenCV 스레드 수 제한
  (11/10 판단팀 확인: OMP_NUM_THREADS=2 로 차선 처리 초당 8.5장, 가장 긴 끊김 0.3초)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

THREAD_VARS = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'KUDOS_CPU_THREADS')


def setup(context):
    share = get_package_share_directory('kudos_yolo_tools')
    cfg = os.path.join(share, 'config', 'yolo_detector_sim.yaml')
    n = LaunchConfiguration('cpu_threads').perform(context).strip()
    env = {} if n in ('', '0') else {k: n for k in THREAD_VARS}
    return [Node(package='kudos_yolo_tools', executable='yolo_detector_node.py',
                 name='yolo_detector_node', output='screen', additional_env=env,
                 parameters=[cfg, {'weights': LaunchConfiguration('weights').perform(context),
                                   'use_sim_time': True}])]


def generate_launch_description():
    share = get_package_share_directory('kudos_yolo_tools')
    return LaunchDescription([
        DeclareLaunchArgument('weights', default_value=os.path.join(share, 'models', 'best_sim_full_1111.pt'),
                              description='모델 경로 (기본: 시뮬 미션 1~5 모델)'),
        DeclareLaunchArgument('cpu_threads', default_value='2',
                              description='YOLO CPU 스레드 수 (0 = 제한 안 함)'),
        OpaqueFunction(function=setup),
    ])
