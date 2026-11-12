"""
실차 LIMO(Jetson) 에서 YOLO 추론 노드 실행 (11/9)

  ros2 launch kudos_yolo_tools yolo_detector.launch.py                      (기본: models/best_sim_full_1111.pt)
  ros2 launch kudos_yolo_tools yolo_detector.launch.py weights:=$HOME/models/best.engine
  ros2 launch kudos_yolo_tools yolo_detector.launch.py depth_topic:=none     (깊이 안 씀 → 크기·바닥으로 거리)
  ros2 launch kudos_yolo_tools yolo_detector.launch.py publish_debug:=false  (대회 때)
  ros2 launch kudos_yolo_tools yolo_detector.launch.py cpu_threads:=0        (CPU 스레드 제한 안 함, 기본 2)

  확인: ros2 topic echo /perception/mission_objects
        노드 로그 '처리 평균 ... ms' = 한 장 처리 시간 (66 ms 이하면 15 Hz 가능)
설정: config/yolo_detector.yaml (카메라 토픽, 표지판 실제 크기, 신호등 색 문턱)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('kudos_yolo_tools')
    cfg = os.path.join(share, 'config', 'yolo_detector.yaml')
    get = lambda n: LaunchConfiguration(n).perform(context)
    over = {'weights': get('weights'), 'device': get('device'),
            'publish_debug': get('publish_debug').lower() in ('1', 'true', 'yes')}
    d = get('depth_topic')
    if d:
        over['depth_topic'] = '' if d.lower() in ('none', 'off', 'false') else d
    n = get('cpu_threads')
    env = {} if n in ('', '0') else {k: n for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                   'MKL_NUM_THREADS', 'KUDOS_CPU_THREADS')}
    return [Node(package='kudos_yolo_tools', executable='yolo_detector_node.py',
                 name='yolo_detector_node', output='screen', additional_env=env,
                 parameters=[cfg, over])]


def generate_launch_description():
    share = get_package_share_directory('kudos_yolo_tools')
    return LaunchDescription([
        DeclareLaunchArgument('weights', default_value=os.path.join(share, 'models', 'best_sim_full_1111.pt'),
                              description='.pt 또는 Jetson 에서 export 한 .engine'),
        DeclareLaunchArgument('device', default_value='0', description='0 = Jetson GPU, cpu = CPU'),
        DeclareLaunchArgument('depth_topic', default_value='',
                              description='비우면 yaml 값. none = 깊이 안 씀'),
        DeclareLaunchArgument('cpu_threads', default_value='2',
                              description='YOLO CPU 스레드 수 — 차선 등 다른 노드 몫 남기기 (0 = 제한 안 함)'),
        DeclareLaunchArgument('publish_debug', default_value='true',
                              description='/perception/yolo_debug 박스 그림 (대회 때 false)'),
        OpaqueFunction(function=setup),
    ])
