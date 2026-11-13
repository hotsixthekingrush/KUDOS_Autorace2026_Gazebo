"""판단 + 상위 제어 통합 노드 (SMACH 기반 Rule-based FSM) - Autorace 2026 규정.

State 9개
    LANE_FOLLOWING (기본)  TRAFFIC_LIGHT  LEFT_A_RIGHT_B  CROSSWALK_STOP  PARKING
    BUMP  SLOW_ZONE  AVOID_STATIC  AVOID_CONE

기본 흐름은 이전과 같다. LANE_FOLLOWING 이 이벤트를 보고 미션 State 로 보내고,
미션 State 는 끝나면 LANE_FOLLOWING 으로 돌아온다.
예외: SLOW_ZONE 은 보행자 표지판을 보면 LANE_FOLLOWING 을 거치지 않고
CROSSWALK_STOP 으로 바로 넘어간다 (두 미션이 붙어 있음).
"""

import collections
import math
import threading

import rclpy
import smach
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from ar_msgs.msg import (ObstacleArray, LaneError,
                         MissionObject, MissionObjectArray)

from ar_mission_supervisor.stopline import StopLineTracker
from ar_mission_supervisor.three_lane import ThreeLanePlanner
from ar_mission_supervisor.states.lane_following import LaneFollowing
from ar_mission_supervisor.states.stub import StubState
from ar_mission_supervisor.states.traffic_light import TrafficLight
from ar_mission_supervisor.states.left_a_right_b import LeftARightB
from ar_mission_supervisor.states.crosswalk_stop import CrosswalkStop
from ar_mission_supervisor.states.parking import Parking
from ar_mission_supervisor.states.bump import Bump
from ar_mission_supervisor.states.slow_zone import SlowZone
from ar_mission_supervisor.states.avoid_cone import AvoidCone
from ar_mission_supervisor.states.avoid_static import AvoidStatic


# ----------------------------------------------------------------------
# Transition 테이블
# ----------------------------------------------------------------------
EVENT_TO_OUTCOME = {
    'traffic_light': 'to_traffic_light',
    'sign_ab': 'to_left_a_right_b',
    'pedestrian_sign': 'to_crosswalk',
    'parking_sign': 'to_parking',
    'slow_sign': 'to_bump',
    'slow_line': 'to_slow_zone',
    'cone': 'to_cone',
    'limo': 'to_avoid_static',
}

OUTCOME_TO_STATE = {
    'to_traffic_light': 'TRAFFIC_LIGHT',
    'to_left_a_right_b': 'LEFT_A_RIGHT_B',
    'to_crosswalk': 'CROSSWALK_STOP',
    'to_parking': 'PARKING',
    'to_bump': 'BUMP',
    'to_slow_zone': 'SLOW_ZONE',
    'to_avoid_static': 'AVOID_STATIC',
    'to_cone': 'AVOID_CONE',
}

MISSION_LABELS = list(OUTCOME_TO_STATE.values())

IMPLEMENTED = {
    'TRAFFIC_LIGHT': TrafficLight,
    'LEFT_A_RIGHT_B': LeftARightB,
    'CROSSWALK_STOP': CrosswalkStop,
    'PARKING': Parking,
    'BUMP': Bump,
    'SLOW_ZONE': SlowZone,
    'AVOID_CONE': AvoidCone,
    'AVOID_STATIC': AvoidStatic,
}

# 이벤트 발행원. 'test' 는 /perception/event (가짜 인지·손 테스트용)
# (추가로 supervisor 가 라이다 장애물에서 직접 만드는 'limo' 이벤트가 있다 - _path_event)
EVENT_SOURCES = ('yolo', 'lidar', 'test')

# 여러 이벤트가 동시에 보일 때의 우선순위 (앞이 우선)
# slow_line 이 pedestrian_sign 보다 앞: 유도선 입구에서는 보행자 표지판도 같이 보이므로
# 먼저 SLOW_ZONE 으로 들어가고, SLOW_ZONE 이 끝 무렵 CROSSWALK_STOP 으로 넘긴다.
EVENT_PRIORITY = ('traffic_light', 'slow_line', 'pedestrian_sign', 'cone', 'sign_ab',
                  'slow_sign', 'parking_sign', 'limo')

# YOLO 클래스 -> 이벤트 이름
YOLO_EVENT = {
    MissionObject.TRAFFIC_LIGHT: 'traffic_light',
    MissionObject.PEDESTRIAN_SIGN: 'pedestrian_sign',
    MissionObject.SIGN_A: 'sign_ab',
    MissionObject.SIGN_B: 'sign_ab',
    MissionObject.SLOW_GUIDE_LINE: 'slow_line',
    MissionObject.SLOW_SIGN: 'slow_sign',
    MissionObject.BUMP_SIGN: 'slow_sign',
    MissionObject.PARKING_SIGN: 'parking_sign',
    MissionObject.LIMO: 'limo',       # 진행 경로 앞에 있을 때만 (주차 구역 LIMO 무시)
}

LIGHT_NAMES = {
    MissionObject.LIGHT_UNKNOWN: 'unknown',
    MissionObject.LIGHT_RED: 'red',
    MissionObject.LIGHT_YELLOW: 'yellow',
    MissionObject.LIGHT_GREEN: 'green',
}


# ----------------------------------------------------------------------
# 파라미터 기본값 (YAML 에서 덮어씀). 실수형은 YAML 에서도 반드시 소수점 표기.
# ----------------------------------------------------------------------
PARAM_DEFAULTS = {
    '': {
        'control_rate': 20.0,
        'wheel_base': 0.2,
        'base_speed': 0.4,
        'max_steer': 0.409,         # 등가 조향각 한계. LIMO 드라이버 안쪽바퀴 28도 -> R_min 약 0.46 m
        'front_wheel_x': 0.10,     # base_link -> 앞바퀴 축 [m]. 규정 판정 기준점
        'cmd_angular_is_steering': False,  # True: /cmd_vel angular.z 에 조향각[rad]을 넣음 (Gazebo 아커만 플러그인용)
        # (11/13) 실차 리모가 아커만 모드면 WeGo 아커만 드라이버(limo_base_ackerman.launch.py) 하나만 켜고, 명령은
        #   /limo/ack_cmd (ackermann_msgs/AckermannDrive: speed [m/s], steering_angle [rad] -0.42 ~ 0.42) 로 받는다.
        #   차동용 limo_base 는 아커만 모드에서 /cmd_vel 은 받지만 odom 을 안 내고, 둘을 같이 켜면 같은 포트를 같이 읽어 꼬임.
        #   'twist' = /cmd_vel (예전, Gazebo), 'ackermann' = ackermann_topic 으로 AckermannDrive.
        'cmd_output': 'twist',
        'ackermann_topic': '/limo/ack_cmd',
        'ackermann_steer_scale': 1.0,   # steering_angle = 자전거 모델 조향각(atan(축간 x 회전속도 / 속도)) x 이 값 (실측 전)
        'ackermann_max_steer': 0.42,    # WeGo 문서: steering_angle -0.42 ~ 0.42 rad
        # (11/13) odom 기준점 -> 판단 기준점(차 몸통 가운데 = 앞뒤 바퀴 축 사이) 앞쪽 거리 [m].
        #   WeGo 아커만 드라이버는 base_link(odom 점)가 뒷바퀴 축 가운데 -> 0.10 (축간 0.2 의 절반). Gazebo·차동 드라이버는 0.
        'odom_center_offset_x': 0.0,
        # (11/13) 판단이 쓰는 차 위치·방향. 'odom' = /odom 그대로. 'imu' = 방향은 imu_topic 의 방향(판단 켤 때 0),
        #   위치는 /odom 속도(twist.linear.x)를 그 방향으로 적분. 실차 보정 주행(ack_calib4): WeGo 아커만 odom 방향이
        #   실제보다 30% 넘게 큼 (전체 회전 IMU 105도 vs odom 136도), IMU 방향은 자이로 적분과 0.3도 안으로 맞음.
        'odom_heading_source': 'odom',
        'imu_topic': '/imu',
    },
    'lane': {
        # 차선 노드가 lateral 을 재는 지점 = 차 앞 몇 m (차선 노드 reference_distance_m 과 같아야 함, 11/04 0.5 로 정함)
        # > 0: 그 점(차선 중앙 + 옮길 양)을 Pure Pursuit 로 따라감. 0: 예전 PD (아래 이득)
        'reference_distance_m': 0.5,
        # 미션 기준 좌표(3차선·주차)를 잡을 때 카메라 차선을 믿는 조건 (아니면 차 위치·방향 그대로) (11/05)
        'frame_min_confidence': 0.3,
        'frame_max_heading_deg': 15.0,
        # (11/11) 차선 노드가 한 선만 보고 추정한 값(single_lane_estimated)은 기준 좌표에 안 씀
        #   (차선 팀원 새 방식: 한 선이면 '마지막 폭의 절반' 떨어진 경로 기준 -> 넓어진 곳에선 실제 중앙과 최대 15 cm 다름)
        'frame_reject_single': True,
        'kp_lateral': 1.2,
        'kd_lateral': 0.15,
        'kp_heading': 0.8,
        # (11/10) 기본 주행(LANE_FOLLOWING) 코너 감속: 차선 방향이 크게 틀어졌거나 확신도가 낮으면 corner_speed.
        # 조건이 풀려도 corner_hold_sec 동안은 감속 유지 (코너 안에서 값이 흔들려 빨라졌다 느려졌다 안 하게)
        'corner_speed': 0.2,
        'corner_heading_deg': 15.0,
        'corner_min_confidence': 0.3,
        'corner_hold_sec': 1.0,
        'corner_single_lane': True,   # (11/11) 한 선 추정(single_lane_estimated) 프레임이어도 감속
        # (11/12) 차선 팀원 제안 (예전 limo_control 식, 사용자 선택). Gazebo 첫 코너 4/4·지그재그 2/2, 실차 곡선 2곳 통과.
        #   Pure Pursuit 는 lateral 이 정확한 m 단위여야 맞게 꺾는데 실차 카메라 값이 아직 시뮬 값이라 실차 곡선을 못 돎.
        # omega_pd: 조향 = 회전속도 pd_kp * lateral + pd_kd * (lateral 변화율), +-pd_max_omega (모든 차선 추종 State 공통).
        #   변화율은 새 차선 값이 왔을 때만 계산해 pd_d_smooth 로 평활, pd_d_jump 넘게 튀면(한 선<->두 선, 차로 바뀜) 버림.
        #   조향각 변화는 초당 steer_rate_max 까지 (명령이 툴툴 바뀌지 않게). false = Pure Pursuit (11/04)
        'omega_pd': True,
        'pd_kp': 4.0,              # [rad/s per m] 실차 시험값 (팀원 kp = kd = 4.0, base_speed 0.2)
        # (11/12 Gazebo pd_lane_run1: 한 선 구간에서 옆오차가 0.1초마다 1 cm 씩 흔들려 kd 5.5 의 D 항이 P 보다 커지고
        #   핸들이 0.5초마다 좌우로 뒤집힘 -> kd 1, 변화율·옆오차 평활. 같은 녹화 값을 넣어 다시 계산하면 좌우 바뀜
        #   코너1 9->4, 지그재그 25->9, 코너2 10->2, 코너에서 한쪽으로 꺾는 크기는 그대로 0.27~0.28 rad)
        'pd_kd': 1.0,              # [rad/s per m/s]
        'pd_d_smooth': 0.15,       # 변화율 평활: 새 값 비중
        'pd_lat_tau': 0.15,        # 옆오차 평활 시간 상수 [s] (0 = 안 함, 차선 10 Hz 면 새 값 비중 약 0.5). 튀면 새 값에서 다시 시작
        'pd_d_jump': 0.12,         # 이웃 차선 값끼리 이만큼 넘게 다르면 변화율 계산 안 함 [m]
        'pd_max_omega': 1.0,       # [rad/s]
        'steer_rate_max': 2.0,     # 조향각 변화 한계 [rad/s] (0.41 rad 를 0.2초에)
        # speed_by_lateral: LANE_FOLLOWING 속도 = base_speed * max(min_ratio, 1 - |lateral| / slow_lat),
        #   한 선 추정 프레임은 min_ratio. 최저 min_speed. 가속 accel_max, 감속 decel_max 까지.
        #   켜면 위 corner_* 감속은 안 씀 (겹치지 않게). false = corner_* 감속 (11/10)
        'speed_by_lateral': True,
        'slow_lat': 0.17,          # |lateral| 이 이 값이면 최저 비율 [m]
        'min_ratio': 0.4,
        'min_speed': 0.12,         # 이보다 느리면 모터가 섰다 갔다 함 (팀원 실차: 0.08 에서 끄릭끄릭) [m/s]
        'accel_max': 0.4,          # [m/s^2]
        'decel_max': 1.0,          # [m/s^2] 코너 앞에선 빨리 줄임
    },
    'failsafe': {
        'hold_sec': 1.0,
        'creep_speed': 0.12,
        'stop_after_sec': 5.0,
        'creep_keep_steer': True,  # (11/10) 저속 구간에서 직전 조향각 유지 (false = 예전처럼 직진)
    },
    'perception': {
        'stale_sec': 0.5,
        'debounce_count': 3,
        'lockout_sec': 8.0,
        'exit_miss_count': 3,
    },
    'yolo': {
        'event_range': 3.0,        # 위치가 유효할 때 이 거리 안에 들어와야 이벤트로 인정
        'pedestrian_range': 1.5,   # 보행자 표지판만 따로 (유도선보다 먼저 잡혀 SLOW_ZONE 을 건너뛰지 않게)
        'confirm_frames': 2,       # 같은 이벤트가 연속 N 프레임 검출돼야 인정 (오인식 방지)
        'hold_sec': 1.0,           # YOLO 결과 유지시간 (추론이 느려도 프레임 사이가 끊기지 않게)
        # (11/05) 옆 길 표지판 거르기: 위치가 있으면 |y_m| 이 이 안일 때만 씀 (이벤트·거리·신호등 색 전부)
        'max_lateral': 0.7,        # 표지판·신호등·유도선 [m]
        'sign_ab_max_lateral': 1.1,    # A/B 표지판 (3차선 가운데 섬 앞이라 옆 차선에서도 봐야 함) [m]
        'crosswalk_max_lateral': 0.35, # 정지 기준 횡단보도 [m]
        # (11/05) 정지선은 YOLO 횡단보도(줄무늬) 앞끝보다 이만큼 차 쪽에 있음 -> 그만큼 앞에서 선다 [m]
        'crosswalk_to_stopline': 0.06,
    },
    'traffic_light': {
        'approach_speed': 0.2,
        'min_speed': 0.08,
        'slowdown_distance': 0.0,  # >0 이면 정지 직전 이 거리부터 감속 (0 = 끝까지 approach_speed)
        'stop_gap': 0.03,          # 앞바퀴~정지선 남은 거리가 이 값 이하면 정지/통과 판정
        'green_confirm_sec': 0.3,  # 정지 후 초록불이 이 시간 이상 유지되면 출발
        'approach_timeout': 25.0,  # 정지선 거리를 끝내 못 받으면 이 시간 뒤 현 위치에서 판정
        'max_wait_sec': 10.0,      # 정지 후 이 시간 안에 초록불을 못 보면 출발 (0 = 무한 대기)
    },
    'crosswalk': {
        'approach_speed': 0.2,
        'min_speed': 0.08,
        'slowdown_distance': 0.0,
        'stop_gap': 0.03,
        'wait_sec': 4.0,           # 규정: 완전정지 후 3초 이상, 15초 이내 출발
        'approach_timeout': 25.0,
    },
    'bump': {
        'speed': 0.2,              # 규정: 기본 속도의 1/2, 최대 0.2 m/s
        'after_sign_distance': 1.8,    # 표지판을 지나 이만큼 더 가면 끝 (odom) [m] (11/05)
        'unknown_sign_distance': 2.0,  # 표지판 위치를 한 번도 못 받았을 때 표지판까지 거리로 침 [m]
        'duration_sec': 8.0,       # odom 이 없을 때만: 진입 후 이 시간 뒤 끝 (예전 방식)
        'max_duration': 30.0,      # 안전용 최대 시간 [s]
    },
    'slow_zone': {
        'speed': 0.2,
        'lock_offset': 0.0,        # 유도선 시작점 도달 후 이만큼 더 가서 헤딩 고정 [m]
        'lock_delay_sec': 1.0,     # 유도선 거리를 모를 때: 진입 후 이 시간 뒤 헤딩 고정
        'k_yaw': 1.5,              # 고정 헤딩 유지 게인 [rad 조향 / rad 오차]
        'lane_blend': 0.3,         # 차선 보정 비율 (0 = 순수 헤딩 유지, 1 = 일반 차선 추종과 같은 세기)
        'handoff_stopline': 0.5,   # 보행자 표지판을 본 뒤, 정지선이 이 거리 안일 때 CROSSWALK_STOP 으로
        'handoff_sign': 1.0,       # 정지선이 안 보이면 표지판이 이 거리 안일 때 (0 = 이 조건 끔)
        'max_duration': 20.0,      # 보행자 표지판을 끝내 못 보면 이 시간 뒤 LANE_FOLLOWING 복귀
    },
    'left_a_right_b': {
        'trigger_distance': 0.8,
        'approach_speed': 0.25,
        'shift_speed': 0.25,
        'lane_width': 0.50,
        'shift_duration': 2.5,
        'settle_sec': 1.0,
        'approach_timeout': 10.0,
        'max_duration': 20.0,
        'use_lane_context': True,  # AVOID_STATIC 에서 이어받으면 odom 모드 (3차선 차선 번호 사용)
        'a_lane': 0,               # odom 모드에서 A(좌) 경로로 이어지는 차선 (0 왼쪽, 1 가운데, 2 오른쪽)
        'b_lane': 2,
        'odom_hold_distance': 0.6, # 목표 차선 도착 후 이만큼 더 간 뒤 종료 [m]
    },
    'avoid_static': {
        'trigger_distance': 1.2,   # 라이다: 진행 경로 앞 이 거리 안 장애물이면 진입 [m]
        'trigger_half_width': 0.3, # 진행 경로 폭의 절반 (주차 구역 LIMO 처럼 옆에 있는 건 무시) [m]
        'yolo_range': 1.5,         # YOLO LIMO 는 이 거리 안 + 경로 폭 안일 때만
        'lidar_trigger': True,     # 라이다 장애물로도 진입할지
        # (11/12 사용자 선택) 코너 도는 중엔 라이다 진입을 미룸. Gazebo pd_lane_run1/2: 두 번째 코너를 20도 남기고
        #   (초당 21도로 도는 중) 진입 -> 카메라는 한 선 추정이라 차 방향(-160도, 도로 -180도)을 기준으로 잡아 바깥 선 위까지 감.
        #   odom 회전 속도가 defer_turn_rate_deg 넘으면 기본 주행을 계속하다 잦아들면 진입.
        #   단 장애물이 defer_min_distance 안으로 오면 바로 진입. 0 이면 안 미룸
        'defer_turn_rate_deg': 10.0,
        'defer_min_distance': 0.5,
        'min_obstacle_radius': 0.10,  # 라바콘(작음)과 구분 [m]
        'max_obstacle_radius': 0.40,
        'start_lane': 1,           # 3차선 시작 차선 (항상 가운데)
        'lane_width': 0.52,        # 차선 중앙 간격 (도면: 차선 500 + 점선 20 mm)
        'speed': 0.25,
        'lookahead': 0.45,         # 차선 따라가기 Pure Pursuit 거리 [m]
        'lookahead_short': 0.25,   # 보통으로는 막힐 때 급하게 꺾어 보는 거리 (0 이면 안 씀) [m] (11/04)
        'plan_horizon': 1.8,       # 후보 차선마다 이 거리만큼 미리 굴려 보고 충돌 검사 [m]
        'plan_depth': 2,           # 2: 그 차선으로 가다가 옆 차선으로 한 번 더 옮기는 길까지 봄 (1 = 차선 하나만, 예전) (11/05)
        'plan_horizon_long': 3.0,  # plan_depth 2 일 때 굴려 보는 거리 [m]
        'switch_step': 0.3,        # 이 거리마다 '여기서 옆 차선으로 옮기면' 을 굴려 봄 [m]
        'switch_margin': 0.15,     # 목표 차선은 이만큼 더 멀리 갈 수 있을 때만 바꿈 [m]
        'switch_hold': 1.0,        # 목표 차선을 바꾼 뒤 이 시간은 유지 (지금 목표가 곧 막히면 예외) [s] (11/12: 0.5 -> 1.0)
        # (11/12) 차선 바꾸기 확정: 켜면 목표 차선에 들어가기 전엔 목표 고정 (목표가 commit_distance 안에서 막히고
        #   switch_hold 지났을 때만 예외). pd_lane_run4 의 목표 뒤집힘 때문에 넣었지만 0.55 배치는 계속 다시 골라야
        #   빠져나가는 경우가 있어 (run.py 4개 막힘) 기본 끔. '바로 옆 차선만' 은 0.55 배치에서 막혀서 코드에 안 넣음
        'lane_commit': False,
        'lane_from_odom': True,    # 바꾸는 중에도 차선 번호는 odom 위치 (11/12, pd_lane_run4: 가운데~오른쪽인데 계속 '왼쪽')
        'edge_check': True,        # 굴려 볼 때 3차선 바깥 선을 넘으면 막힌 것으로 봄 (11/05)
        'edge_margin': 0.0,        # 바깥 선(1.5 x lane_width)에 더하는 여유 [m]
        'limo_length': 0.322,      # 장애물 LIMO 박스 (진행방향 길이 x 폭) [m]
        'limo_width': 0.22,
        'car_length': 0.322,       # 우리 차
        'car_width': 0.22,
        'wheel_box_length': 0.0,   # 바퀴 네 개를 감싼 사각형도 따로 검사 (0 이면 안 씀. Gazebo 만 씀) [m]
        'wheel_box_width': 0.0,
        'clearance': 0.03,         # 장애물 박스에 더하는 여유 [m]
        # (11/12 사용자 선택) 지금 자리에서 이미 여유 박스와 겹친 LIMO 는 지금 안 겹칠 만큼만 여유를 줄여서 봄
        #   (진짜 크기보다 overlap_extra 까지). static_run6: 진짜 틈 2.6 cm 인데 '이미 닿음' 으로 봐서 22초 정지
        'overlap_relax': True,
        'overlap_extra': 0.03,     # 지도 오차 몫 [m]
        'overlap_slack': 0.02,     # 안 겹칠 만큼에 더 줄이는 몫 (방향 틀 때 차 뒤쪽이 옆으로 도는 몫) [m]
        'commit_distance': 0.9,    # 고른 차선으로 이 거리도 못 가면 감속 [m]
        'slow_speed': 0.10,        # 그때 감속 속도
        'stop_distance': 0.25,     # 그래도 이 거리 안에서 닿게 되면 정지 [m]
        'backup_after': 1.0,       # 이만큼 서 있으면 후진 [s] (11/04)
        'backup_distance': 0.20,   # 한 번에 후진하는 거리 [m]
        'backup_speed': 0.10,      # 후진 속도 [m/s]
        'backup_max': 3,           # 한 미션에 후진하는 최대 횟수
        'backup_k_heading': 1.5,   # 후진하며 차 방향을 차선 방향으로 펴는 게인 [rad/rad]
        'settle_tolerance': 0.08,  # 목표 차선 중앙 +- 이 안 + 방향 오차 settle_heading 이하면 변경 완료
        'settle_heading': 0.20,
        'merge_distance': 0.20,
        'confirm_hits': 2,
        'map_average': True,       # LIMO 위치 = 지금까지 본 것의 평균 (false = 예전 방식, 새 값 쪽으로 0.3) (11/04)
        'ghost_min_hits': 5,       # 이만큼 본 LIMO 박스와 겹치는 새 검출은 버림 (0 = 안 함) (11/04)
        'ghost_margin': 0.05,      # 겹침 판정 때 박스 길이·폭에 더하는 값 [m]
        'exit_clear_distance': 0.8,  # 마지막 장애물을 지나 이만큼 더 가면 종료 [m]
        'max_distance': 7.0,       # 진입 후 최대 주행거리 (3차선 직선 5.5 m) [m]
        'max_duration': 40.0,
        'context_timeout': 5.0,    # 끝난 뒤 이 시간 안에 A/B 가 오면 차선 번호를 넘겨줌 [s]
        # (11/12 사용자 선택) 카메라 차선을 못 믿어 차 방향을 기준으로 쓸 때, 트랙 축(axis_ref_deg + 90도 단위)에서
        #   axis_snap_max_deg 안이면 그 축으로 맞춤. Gazebo pd_lane_run3: 코너 16도 덜 돈 채(-164도) 진입 -> 바깥 선 위.
        #   가정: 트랙은 직각, odom yaw 0 = 출발 직선 방향 (출발선에 똑바로 놓고 odom 켜기)
        'axis_snap': True,
        'axis_ref_deg': 0.0,
        'axis_snap_max_deg': 30.0,
        # (11/12 사용자 선택) 차 방향 기준일 때 옆 위치(d = 0)도 차 위치라 차가 비켜 있으면 세 차선이 다 밀림
        #   (pd_lane_run4: 11 cm). 차 앞 lat_snap_range 안에서 확실히 본 LIMO 가 lat_snap_min_items 대 이상,
        #   차선 가운데에서 비킨 값이 lat_snap_spread 안에 절반 넘게 모이면 그 평균만큼 한 번 옮김.
        #   lat_snap_max 넘으면 애매해서 안 옮김. 가정: LIMO 는 차선 가운데에 서 있음 (연습 배치는 그럼, 대회 확정 아님)
        'lat_snap': True,
        'lat_snap_min_items': 2,
        'lat_snap_spread': 0.08,
        'lat_snap_max': 0.20,
        'lat_snap_range': 2.0,
        'gap_speed': 0.12,         # 비상용 빈 공간 따라가기
        'gap_range': 1.5,
        'gap_gain': 1.0,
        'robot_width': 0.30,
    },
    'cone': {
        'speed': 0.25,
        'creep_speed': 0.10,
        'lookahead': 0.45,         # 게이트 중점을 이은 선 위 목표점 거리 [m]
        'curve_gain': 0.35,
        'roi_min_x': 0.05,
        'roi_max_x': 2.0,
        'roi_max_y': 1.0,
        'max_radius': 0.18,
        'pair_max_dx': 0.45,
        'pair_max_angle_deg': 45.0,  # 게이트 두 콘을 이은 선이 차 옆방향에서 이만큼 넘게 틀어지면 게이트 아님
        'min_gate_width': 0.30,
        'side_offset': 0.22,
        'robot_width': 0.22,
        'max_duration': 40.0,
        # 미리 굴려보기 (11/02 사용자 선택: 급한 S자 대응)
        'use_rollout': True,       # false = 예전 방식 (게이트 선 위 lookahead 점 Pure Pursuit)
        'horizon': 1.0,            # 앞으로 굴려보는 거리 [m]
        'offset_max': 0.16,        # 게이트 길을 옆으로 옮겨 보는 최대 거리 [m]
        'offset_step': 0.04,       # 옮기는 간격 [m]
        'car_length': 0.322,       # 판단이 검사하는 차 몸통 (실차 LIMO)
        'car_width': 0.22,
        'clearance': 0.02,         # 콘 반지름(라이다)에 더하는 여유 [m]
        'lookahead_short': 0.25,   # 후보: 짧게 보고 꽉 꺾어 도는 길 [m] (11/04, 0 = 안 씀)
        'lookahead_long': 0.60,    # 후보: 길게 보고 부드럽게 가는 길 [m] (0 = 안 씀)
        'w_lookahead': 0.1,        # 기본 lookahead 와 다른 만큼 벌점
        'tight_speed': 0.15,       # 안 닿는 길이 없을 때 속도 [m/s] (멈추지 않고 덜 닿는 쪽으로. 0.05 는 기어가다 40초 넘김 - 11/04 녹화)
        'w_offset': 1.0,           # 길을 옆으로 옮긴 만큼 벌점 (가운데 선호)
        'w_margin': 1.0,           # 콘과 여유 보너스 (margin_cap 까지만)
        'margin_cap': 0.10,
        'steer_rate_deg': 0.0,     # 바퀴 조향이 바뀌는 속도 [도/s] 흉내. 0 = 바로 바뀜
        'memory_merge': 0.10,      # 같은 콘으로 볼 거리 [m]
        'memory_behind': 0.6,      # 차 뒤로 이만큼 지나간 콘은 잊음 [m]
        # 게이트를 못 만들 때 한쪽 콘 선 따라가기 (11/02 사용자 제안)
        'side_use_rollout': True,  # 한쪽 줄 -> 가상 가운데 길을 미리 굴려보기로 (false: 차선식 조향)
        'side_kp_lateral': 1.2,    # 선에서 원하는 거리와의 차이 게인 (차선 주행과 같은 값)
        'side_kp_heading': 0.8,    # 선과 틀어진 각도 게인
        'side_half_width': 0.36,   # 게이트를 한 번도 못 봤을 때 쓰는 통로 반폭 [m]
        'side_max_step': 0.62,     # 같은 줄 다음 콘으로 볼 최대 거리 [m]
        'side_max_turn_deg': 60.0, # 같은 줄 다음 콘은 지금 줄 방향에서 이 각도 안 (건너편 콘은 거의 90도)
        'pass_margin': 0.10,       # 마지막 게이트 선을 앞바퀴가 이만큼 더 지날 때까지 구간을 안 끝냄 [m]
    },
    'parking': {
        'search_speed': 0.15,
        'final_speed': 0.08,
        'min_speed': 0.04,
        'turn_speed': 0.10,
        'turn_radius': 0.50,       # 계획 회전반경. 실제 최소(실차 약 0.46 m)보다 조금 크게 [m]
        'k_radial': 2.0,           # 원호 오차 보정 게인 [rad/m]
        'bay_pitch': 0.52,         # 칸 간격 (칸 500 + 선 20 mm)
        'pitch_tolerance': 0.22,   # LIMO 쌍 간격(2 x pitch) 허용 오차 [m]
        'pair_d_tolerance': 0.25,
        'bay_d_min': -0.95,        # 윗줄 칸에 선 LIMO 만 보기 위한 범위 (기준 차선 중앙, 왼쪽 +) [m]
        'bay_d_max': -0.30,
        'park_depth': 0.57,        # 차선 중앙 -> 칸 깊이 가운데 (차선 반폭 0.25 + 선 0.02 + 칸 반 0.30) [m]
        # (11/10) 칸 깊이 목표 = LIMO 두 대 가운데. 차선 기준(park_depth)과 depth_max_diff 넘게 다르면 둘의 중간
        'limo_depth': True,
        'depth_max_diff': 0.15,    # LIMO 가 칸 안이면 칸 가운데서 최대 (0.6 - 0.322)/2 = 0.14 m 벗어남 [m]
        'min_obstacle_radius': 0.05,
        'max_obstacle_radius': 0.40,
        'merge_distance': 0.20,
        'confirm_hits': 2,
        'gap_confirm': 3,
        'gap_match': 0.15,
        'slow_distance': 0.15,     # 회전 시작점 이 거리 앞부터 final_speed 로 [m]
        'overshoot_tolerance': 0.04,
        'heading_tolerance': 0.05, # -90도 에서 이 안이면 회전 끝 [rad]
        'k_heading': 1.5,
        'k_lateral': 2.0,
        'k_long': 1.0,
        'stop_tolerance': 0.02,
        'search_timeout': 30.0,
        'search_distance': 6.0,
        'turn_timeout': 15.0,
        'align_timeout': 15.0,
    },
    'debug': {
        'enabled': True,
    },
}

# 파라미터 그룹 -> 노드 속성 이름 (다른 경우만)
GROUP_ATTR = {'lane': 'lane_gains'}


# ----------------------------------------------------------------------
# 보조 클래스
# ----------------------------------------------------------------------
class Buffer(object):
    """최신 인지값 + 수신 시각. 오래되면 무효 처리.

    (값, 시각)을 한 덩어리로 저장해, 수신 스레드와 상태머신 스레드가
    동시에 접근해도 새 값과 옛 시각이 섞여 읽히지 않게 한다.
    """

    def __init__(self, stale_sec):
        self._data = (None, -1e9)
        self.stale_sec = stale_sec

    @property
    def value(self):
        return self._data[0]

    @value.setter
    def value(self, v):
        self._data = (v, self._data[1])

    def set(self, value, now):
        self._data = (value, now)

    def get(self, now):
        value, stamp = self._data
        if value is None or (now - stamp) > self.stale_sec:
            return None
        return value

    def get_with_stamp(self, now):
        """(값, 수신 시각) 또는 None. 새 측정인지 구분할 때 사용."""
        value, stamp = self._data
        if value is None or (now - stamp) > self.stale_sec:
            return None
        return value, stamp


class Debouncer(object):
    """같은 값이 N회 연속 확인될 때만 통과시킨다."""

    def __init__(self, count):
        self.count = count
        self.last = None
        self.n = 0

    def push(self, value):
        if value == self.last:
            self.n += 1
        else:
            self.last = value
            self.n = 1
        return value if self.n >= self.count else None


def _pose_from_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (float(p.x), float(p.y), yaw)


def _stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


# 차선 인지 한 개. lateral/heading 은 차 앞 x_ref 지점 값 (x_ref 는 찍은 뒤 간 거리만큼 줄인 값),
# stamp 는 영상 찍은 시각 (없으면 받은 시각). 예전처럼 [0] [1] [2] 로도 읽힌다.
# single = 차선 노드가 한 선만 보고 추정한 값 (LaneError.single_lane_estimated, 11/11)
LaneSample = collections.namedtuple('LaneSample', 'lateral heading confidence x_ref stamp single',
                                    defaults=(False,))


# ----------------------------------------------------------------------
# 노드
# ----------------------------------------------------------------------
class MissionSupervisor(Node):

    BUFFER_KEYS = ('lane', 'scan', 'stopline', 'obstacles', 'odom',
                   'event_yolo', 'event_lidar', 'event_test',
                   'light_color', 'marker_type', 'marker_dist', 'ped_dist', 'slowline_dist',
                   'bump_dist')
    YOLO_KEYS = ('event_yolo', 'light_color', 'marker_type', 'marker_dist', 'ped_dist',
                 'slowline_dist', 'bump_dist')

    def __init__(self):
        super().__init__('mission_supervisor')
        self._load_params()

        # --- 발행 ---
        self.ack_msg_type = None
        if self.cmd_output == 'ackermann':
            from ackermann_msgs.msg import AckermannDrive
            self.ack_msg_type = AckermannDrive
            self.cmd_pub = self.create_publisher(AckermannDrive, self.ackermann_topic, 10)
            self.get_logger().info('명령 출력: %s (AckermannDrive, 조향각 x %.3f, 한계 %.2f rad), odom 기준점 앞 %.2f m'
                                   % (self.ackermann_topic, self.ackermann_steer_scale, self.ackermann_max_steer,
                                      self.odom_center_offset_x))
        else:
            if self.cmd_output != 'twist':
                self.get_logger().warn('cmd_output=%r 모름 - twist(/cmd_vel) 로 보냄' % self.cmd_output)
                self.cmd_output = 'twist'
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_ack_steer = 0.0
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(String, '/supervisor/state', latched)
        self.current_label = 'INIT'
        self.last_v = 0.0          # 마지막으로 보낸 선속도 (지연 보정용)
        self._debug_pubs = {}
        self._debug_lock = threading.Lock()

        # --- 인지 버퍼 ---
        stale = self.perception['stale_sec']
        self.buf = dict((k, Buffer(stale)) for k in self.BUFFER_KEYS)
        self.yaw_hist = collections.deque(maxlen=80)   # (수신 시각, yaw) - 회전 속도 계산용 (11/12)
        self._defer_log = None
        for k in self.YOLO_KEYS:
            self.buf[k].stale_sec = max(stale, self.yolo['hold_sec'])
        self._yolo_streak = {}
        self.stopline = StopLineTracker(self)     # 정지선 거리 추정 (State 들이 공유)
        self.three_lane = ThreeLanePlanner(self)  # 3차선 좌표·차선 번호 (AVOID_STATIC -> LEFT_A_RIGHT_B)

        # --- 구독 ---
        store = self._store
        sub = self.create_subscription
        sub(LaneError, '/perception/lane_error', self._cb_lane, 10)
        sub(MissionObjectArray, '/perception/mission_objects', self._cb_mission_objects, 10)
        sub(LaserScan, '/scan', store('scan', lambda m: m), qos_profile_sensor_data)
        for src in ('lidar', 'test'):
            topic = '/perception/event' if src == 'test' else '/perception/event/' + src
            sub(String, topic, store('event_' + src, lambda m: m.data), 10)
        sub(ObstacleArray, '/perception/obstacles',
            store('obstacles', lambda m: list(m.obstacles)), 10)
        sub(Odometry, '/odom', self._cb_odom, 10)
        self._imu_yaw = None       # IMU 방향 (판단 켤 때 0) [rad]
        self._imu_yaw0 = None
        self._dr = None            # odom 속도 + IMU 방향 적분 [x, y, yaw, stamp]
        if self.odom_heading_source == 'imu':
            from sensor_msgs.msg import Imu
            sub(Imu, self.imu_topic, self._cb_imu, 10)
            self.get_logger().info('차 위치: /odom 속도 + %s 방향 (판단 켤 때 방향 0)' % self.imu_topic)
        elif self.odom_heading_source != 'odom':
            self.get_logger().warn('odom_heading_source=%r 모름 - /odom 그대로 씀' % self.odom_heading_source)
            self.odom_heading_source = 'odom'
        sub(Bool, '/supervisor/reset', self._cb_reset, 10)

        # --- 전이 판정 ---
        self.debouncer = Debouncer(self.perception['debounce_count'])
        self.lockout = {}
        self._reset_flag = False
        self._lock = threading.Lock()

        self.create_timer(0.5, self._publish_state_tick)
        self.get_logger().info('mission_supervisor 준비 완료 (%.1f Hz)'
                               % self.control_rate)

    # ------------------------------------------------------------------
    # 파라미터
    # ------------------------------------------------------------------
    def _load_params(self):
        for group, items in PARAM_DEFAULTS.items():
            values = {}
            for key, default in items.items():
                name = '%s.%s' % (group, key) if group else key
                self.declare_parameter(name, default)
                values[key] = type(default)(self.get_parameter(name).value)
            if group:
                setattr(self, GROUP_ATTR.get(group, group), values)
            else:
                for key, value in values.items():
                    setattr(self, key, value)

        self.control_period = 1.0 / self.control_rate

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------
    def _cb_imu(self, msg):
        q = msg.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._imu_yaw0 is None:
            self._imu_yaw0 = yaw
        d = yaw - self._imu_yaw0
        self._imu_yaw = math.atan2(math.sin(d), math.cos(d))

    def _dead_reckon(self, msg):
        """(11/13) odom 속도 + IMU 방향으로 위치 적분. IMU 가 아직 없으면 None."""
        yaw = self._imu_yaw
        if yaw is None:
            return None
        t = _stamp_sec(msg.header)
        if t <= 0.0:
            t = self.clock_sec()
        v = float(msg.twist.twist.linear.x)
        if self._dr is None:
            self._dr = [0.0, 0.0, yaw, t]
        else:
            x, y, yaw_prev, t_prev = self._dr
            dt = t - t_prev
            if 0.0 < dt < 0.5:
                dyaw = math.atan2(math.sin(yaw - yaw_prev), math.cos(yaw - yaw_prev))
                mid = yaw_prev + 0.5 * dyaw
                x += v * math.cos(mid) * dt
                y += v * math.sin(mid) * dt
            self._dr = [x, y, yaw, t]
        return (self._dr[0], self._dr[1], yaw)

    def _cb_odom(self, msg):
        if self.odom_heading_source == 'imu':
            pose = self._dead_reckon(msg)
            if pose is None:
                return
        else:
            pose = _pose_from_odom(msg)
        off = self.odom_center_offset_x
        if off:
            # (11/13) odom 점(WeGo 아커만: 뒷바퀴 축) -> 판단 기준점(몸통 가운데)
            pose = (pose[0] + off * math.cos(pose[2]), pose[1] + off * math.sin(pose[2]), pose[2])
        now = self.clock_sec()
        self.buf['odom'].set(pose, now)
        self.yaw_hist.append((now, pose[2]))

    def turn_rate(self, now, window=0.4):
        """odom 으로 구한 최근 window 초 회전 속도 [rad/s] (왼쪽 +). 기록이 모자라면 None."""
        hist = [h for h in list(self.yaw_hist) if now - window - 0.05 <= h[0] <= now + 0.05]
        if len(hist) < 2 or hist[-1][0] - hist[0][0] < 0.15:
            return None
        dyaw = math.atan2(math.sin(hist[-1][1] - hist[0][1]), math.cos(hist[-1][1] - hist[0][1]))
        return dyaw / (hist[-1][0] - hist[0][0])

    def _store(self, key, extract):
        def callback(msg):
            self.buf[key].set(extract(msg), self.clock_sec())
        return callback

    MAX_AGE = 1.0   # 지연 보정 상한 [s]. 시계가 어긋났을 때 거리가 터무니없이 줄지 않게

    def _yolo_lateral_ok(self, o):
        """YOLO 물체가 우리 길 것인지 (옆 거리로). 위치를 못 구했으면 판단 못 하니 인정.
        11/05 lane_yolo_run1/run2: 위쪽 길에서 안쪽 길의 신호등(오른쪽 1.15 m)·주차 표지판(1.2 m)·
        횡단보도(0.6~0.7 m)를 보고 TRAFFIC_LIGHT·PARKING 에 잘못 들어감.
        트랙 파일 기준 우리 길 표지판은 차선 중앙에서 0~0.43 m.
        LIMO 는 따로 (avoid_static.trigger_half_width, 주차는 라이다)."""
        if o.class_id == MissionObject.LIMO or not o.position_valid:
            return True
        cfg = self.yolo
        if o.class_id == MissionObject.CROSSWALK:
            limit = cfg['crosswalk_max_lateral']
        elif o.class_id in (MissionObject.SIGN_A, MissionObject.SIGN_B):
            limit = cfg['sign_ab_max_lateral']
        else:
            limit = cfg['max_lateral']
        return abs(o.y_m) <= limit

    def _latency_compensated(self, x_m, header, now):
        """사진 촬영 후 지금까지 이동한 거리만큼 앞쪽 거리를 줄인다."""
        stamp = _stamp_sec(header)
        age = max(0.0, now - stamp) if stamp > 0.0 else 0.0
        age = min(age, self.MAX_AGE)
        return x_m - self.last_v * age, age

    def _cb_lane(self, msg):
        if msg.valid:
            now = self.clock_sec()
            stamp = _stamp_sec(msg.header)
            if stamp <= 0.0 or stamp > now:
                stamp = now
            self.buf['lane'].set((float(msg.lateral_offset_m),
                                  float(msg.heading_error_rad),
                                  float(msg.confidence), stamp,
                                  bool(msg.single_lane_estimated)), now)
        else:
            self.buf['lane'].value = None        # 차선 유실 -> 페일세이프

    def _cb_reset(self, msg):
        if msg.data:
            with self._lock:
                self._reset_flag = True
            self.lockout.clear()
            self.get_logger().warn('외부 리셋 요청 수신')

    def _cb_mission_objects(self, msg):
        """YOLO 객체 배열 -> 이벤트, 신호등 색, 갈림길 방향·거리로 분해."""
        now = self.clock_sec()
        rng = self.yolo['event_range']
        ped_rng = self.yolo['pedestrian_range']

        best = {}                                    # 클래스별 가장 확실한 1개
        for o in msg.objects:
            if not self._yolo_lateral_ok(o):
                continue                             # 옆 길 표지판 (11/05)
            prev = best.get(o.class_id)
            if prev is None or o.confidence > prev.confidence:
                best[o.class_id] = o

        st = self.avoid_static

        def near(o):
            if o.class_id == MissionObject.LIMO:
                # 진행 경로 앞에 있는 LIMO 만 (주차 구역처럼 옆에 선 LIMO 는 무시)
                return (o.position_valid and o.x_m <= st['yolo_range']
                        and abs(o.y_m) <= st['trigger_half_width'])
            # 위치를 못 구했으면 거리 조건 없이 인정 (가까워서 화면 밖일 수 있음)
            limit = ped_rng if o.class_id == MissionObject.PEDESTRIAN_SIGN else rng
            return (not o.position_valid) or o.x_m <= limit

        # 이번 프레임에 보이는 이벤트들 -> 연속 프레임 수 갱신
        seen = set()
        for cls, o in best.items():
            name = YOLO_EVENT.get(cls)
            if cls == MissionObject.LIMO:
                continue                         # LIMO 는 여러 대 중 하나라도 경로 앞이면 (아래)
            if name is not None and near(o):
                seen.add(name)
        if any(o.class_id == MissionObject.LIMO and near(o) for o in msg.objects):
            seen.add(YOLO_EVENT[MissionObject.LIMO])
        for name in list(self._yolo_streak):
            if name not in seen:
                del self._yolo_streak[name]
        for name in seen:
            self._yolo_streak[name] = self._yolo_streak.get(name, 0) + 1

        # 확정된 이벤트 전부를 우선순위 순으로 저장 (동시에 여러 개 보일 수 있음)
        need = max(1, self.yolo['confirm_frames'])
        confirmed = tuple(n for n in EVENT_PRIORITY if self._yolo_streak.get(n, 0) >= need)
        if confirmed or not seen:
            # 확정 이벤트가 있거나, 아무것도 안 보이면 갱신.
            # (보이긴 하는데 아직 확정 전이면 이전 값을 hold_sec 동안 유지)
            self.buf['event_yolo'].set(confirmed, now)

        # 정지 기준 = YOLO 횡단보도의 가까운 끝 (11/02: 카메라 정지선 대신).
        # 위치를 못 구한 프레임(가까워서 박스 아래가 화면 밖 등)은 버리고, 판단이 odom 으로 이어서 추정한다.
        cw = best.get(MissionObject.CROSSWALK)
        if cw is not None and cw.position_valid:
            line_x = float(cw.x_m) - self.yolo['crosswalk_to_stopline']
            dist, age = self._latency_compensated(line_x, msg.header, now)
            self.buf['stopline'].set(dist, now)
            self.dbg('stopline/age', age)

        for cls, key in ((MissionObject.PEDESTRIAN_SIGN, 'ped_dist'),
                         (MissionObject.SLOW_GUIDE_LINE, 'slowline_dist')):
            o = best.get(cls)
            if o is not None and o.position_valid:
                dist, _ = self._latency_compensated(float(o.x_m), msg.header, now)
                self.buf[key].set(dist, now)

        # 방지턱·서행 표지판까지 거리 -> BUMP 가 서행을 끝낼 곳을 정할 때 씀 (11/05).
        # 옆으로 먼 검출은 위 _yolo_lateral_ok 에서 이미 빠짐 (lane_yolo_run1: 왼쪽 2.2 m 오검출 1번).
        # 둘 다 보이면 먼 쪽 (서행을 더 길게 = 안전한 쪽)
        bump_signs = [o for o in (best.get(MissionObject.BUMP_SIGN),
                                  best.get(MissionObject.SLOW_SIGN))
                      if o is not None and o.position_valid]
        if bump_signs:
            far = max(bump_signs, key=lambda o: o.x_m)
            dist, _ = self._latency_compensated(float(far.x_m), msg.header, now)
            self.buf['bump_dist'].set(dist, now)

        light = best.get(MissionObject.TRAFFIC_LIGHT)
        if light is not None:
            self.buf['light_color'].set(int(light.light_color), now)

        signs = [o for o in (best.get(MissionObject.SIGN_A),
                             best.get(MissionObject.SIGN_B)) if o]
        if signs:
            sign = max(signs, key=lambda o: o.confidence)
            self.buf['marker_type'].set(
                'A' if sign.class_id == MissionObject.SIGN_A else 'B', now)
            if sign.position_valid:
                dist, _ = self._latency_compensated(float(sign.x_m), msg.header, now)
                self.buf['marker_dist'].set(dist, now)

        stamp = _stamp_sec(msg.header)
        self.dbg('yolo/age', max(0.0, now - stamp) if stamp > 0.0 else 0.0)

    # ------------------------------------------------------------------
    # 공통 헬퍼
    # ------------------------------------------------------------------
    def ok(self):
        return rclpy.ok()

    def clock_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _latest(self, key):
        return self.buf[key].get(self.clock_sec())

    def consume_reset(self):
        with self._lock:
            flag = self._reset_flag
            self._reset_flag = False
        return flag

    def set_lockout(self, label):
        self.lockout[label] = self.clock_sec() + self.perception['lockout_sec']

    def dbg(self, name, value):
        """튜닝용 내부 값 발행. /supervisor/debug/<name>
        숫자·bool 은 Float32, 문자열은 String 으로 나간다."""
        if not self.debug['enabled'] or value is None:
            return
        is_text = isinstance(value, str)
        with self._debug_lock:
            pub = self._debug_pubs.get(name)
            if pub is None:
                msg_type = String if is_text else Float32
                pub = self.create_publisher(msg_type, '/supervisor/debug/' + name, 10)
                self._debug_pubs[name] = pub
        msg = String() if is_text else Float32()
        msg.data = value if is_text else float(value)
        pub.publish(msg)

    def publish_state(self, label):
        self.current_label = label
        msg = String()
        msg.data = label
        self.state_pub.publish(msg)

    def _publish_state_tick(self):
        msg = String()
        msg.data = self.current_label
        self.state_pub.publish(msg)

    # ------------------------------------------------------------------
    # 인지 조회 (State 들이 사용)
    # ------------------------------------------------------------------
    def lane_error(self, now=None):
        """LaneSample(lateral, heading, confidence, x_ref, stamp, single) 또는 None.

        x_ref = 차선 노드 기준점(lane.reference_distance_m) - 영상 찍은 뒤 지금까지 간 거리 (0.1 m 아래로는 안 줄임).
        기준점이 0 이면 x_ref 도 0 (예전 방식).
        """
        if now is None:
            now = self.clock_sec()
        value = self.buf['lane'].get(now)
        if value is None:
            return None
        lateral, heading, confidence, stamp, single = value
        ref = float(self.lane_gains.get('reference_distance_m', 0.0))
        x_ref = 0.0
        if ref > 0.0:
            age = min(max(0.0, now - stamp), self.MAX_AGE)
            x_ref = max(0.1, ref - self.last_v * age)
        return LaneSample(lateral, heading, confidence, x_ref, stamp, single)

    def lane_for_frame(self, now=None):
        """(차선값 또는 None, 이유). 미션 기준 좌표(LaneFrame)를 잡을 때 쓸 카메라 차선 - 믿을 만할 때만.
        None 이면 차 위치·방향을 그대로 기준으로 쓴다 (가짜 인지 때와 같음).
        11/05 static_run4: 3차선 안에서 카메라 차선이 valid 인데 confidence 0.00, 방향 -24도라
        기준 좌표가 24도 틀어져 트랙 밖으로 감."""
        lane = self.lane_error(now)
        if lane is None:
            return None, '카메라 차선 없음'
        g = self.lane_gains
        if lane[2] < g['frame_min_confidence']:
            return None, '카메라 차선 confidence %.2f < %.2f' % (lane[2], g['frame_min_confidence'])
        if g['frame_reject_single'] and lane.single:
            return None, '카메라 차선이 한 선만 보고 추정한 값 (single_lane_estimated)'
        if abs(lane[1]) > math.radians(g['frame_max_heading_deg']):
            return None, '카메라 차선 방향 %.1f도 > %.1f도' % (math.degrees(lane[1]), g['frame_max_heading_deg'])
        return lane, '카메라 차선 (방향 %.1f도, 옆 %+.2fm)' % (math.degrees(lane[1]), lane[0])

    def merged_event(self, now=None, allowed=None, skip_locked=False):
        """여러 인지 노드의 이벤트를 하나로 합친다. 해당 없으면 None.

        allowed: 이 outcome 으로 이어지는 이벤트만 고려 (미션 State 안에서 바로 넘어갈 때).
        skip_locked: 재진입 금지 중인 미션의 이벤트는 빼고 고른다
                     (잠긴 이벤트가 우선순위가 높아 다른 이벤트를 가리지 않게).
        """
        if now is None:
            now = self.clock_sec()
        found = []
        for src in EVENT_SOURCES:
            value = self.buf['event_' + src].get(now)
            if value is None:
                continue
            names = value if isinstance(value, tuple) else (value,)
            for name in names:
                if name != 'none' and name not in found:
                    found.append(name)
        path = self._path_event(now, quiet=allowed is not None)
        if path is not None and path not in found:
            found.append(path)
        if allowed is not None:
            found = [n for n in found if EVENT_TO_OUTCOME.get(n) in allowed]
        if skip_locked:
            found = [n for n in found if not self._locked(n, now)]
        if not found:
            return None
        for name in EVENT_PRIORITY:
            if name in found:
                return name
        return found[0]

    def _path_event(self, now, quiet=False):
        """라이다 장애물이 진행 경로 앞에 있으면 'limo' (AVOID_STATIC 진입용).
        quiet: 미션 State 안에서 부를 때 '진입 미룸' 로그를 안 남김."""
        cfg = self.avoid_static
        if not cfg['lidar_trigger']:
            return None
        obstacles = self.buf['obstacles'].get(now)
        if not obstacles:
            return None
        near = None
        for o in obstacles:
            if not (cfg['min_obstacle_radius'] <= o.radius <= cfg['max_obstacle_radius']):
                continue
            if 0.1 < o.x <= cfg['trigger_distance'] and abs(o.y) <= cfg['trigger_half_width']:
                near = o.x if near is None else min(near, o.x)
        if near is None:
            return None
        # (11/12) 코너 도는 중이면 미룸 (장애물이 아주 가까우면 예외)
        limit = math.radians(cfg['defer_turn_rate_deg'])
        rate = self.turn_rate(now) if limit > 0.0 else None
        if rate is not None and abs(rate) > limit and near > cfg['defer_min_distance']:
            self.dbg('avoid/deferred', near)
            if not quiet and (self._defer_log is None or now - self._defer_log > 1.0):
                self._defer_log = now
                self.get_logger().info('[AVOID_STATIC] 코너 도는 중 (초당 %.0f도) - 진입 미룸 (앞 장애물 %.2fm)'
                                       % (math.degrees(abs(rate)), near))
            return None
        return 'limo'

    def _locked(self, event, now):
        outcome = EVENT_TO_OUTCOME.get(event)
        if outcome is None:
            return False
        return self.lockout.get(OUTCOME_TO_STATE[outcome], 0.0) > now

    def current_event(self):
        return self.merged_event()

    def detect_mission(self, allowed=None):
        """Supervisor 전이 판정: 유효시간 + 디바운스 + 재진입 차단.

        allowed: 미션 State 안에서 바로 다른 미션으로 넘어갈 때 허용할 outcome 목록.
        """
        now = self.clock_sec()
        raw = self.merged_event(now, allowed, skip_locked=True)
        if allowed is None:
            self.dbg('event', raw if raw else 'none')
        event = self.debouncer.push(raw)
        if event is None:
            return None
        outcome = EVENT_TO_OUTCOME.get(event)
        if outcome is None:
            return None
        if allowed is not None and outcome not in allowed:
            return None
        if self.lockout.get(OUTCOME_TO_STATE[outcome], 0.0) > now:
            return None
        return outcome

    def stopline_distance(self):
        """base_link -> 정지선 앞쪽 가장자리 거리 [m] (지연 보정됨) 또는 None."""
        return self._latest('stopline')

    def stopline_measurement(self):
        """(거리, 수신 시각) 또는 None. 같은 측정을 두 번 쓰지 않게 시각도 준다."""
        return self.buf['stopline'].get_with_stamp(self.clock_sec())

    def slow_line_distance(self):
        """지그재그 유도선 시작(가까운 끝)까지 전방 거리 [m] 또는 None."""
        return self._latest('slowline_dist')

    def pedestrian_sign_distance(self):
        """보행자 표지판까지 전방 거리 [m] 또는 None."""
        return self._latest('ped_dist')

    def bump_sign_measurement(self):
        """(방지턱·서행 표지판까지 전방 거리 [m], 수신 시각) 또는 None. 새 측정인지 시각으로 구분."""
        return self.buf['bump_dist'].get_with_stamp(self.clock_sec())

    def light_color(self):
        """MissionObject.LIGHT_* 값 또는 None(최근에 신호등을 못 봄)."""
        return self._latest('light_color')

    def light_name(self, color):
        return LIGHT_NAMES.get(color, 'none')

    def obstacles(self):
        return self._latest('obstacles')

    def marker_type(self):
        """'A' / 'B' / None"""
        return self._latest('marker_type')

    def marker_distance(self):
        return self._latest('marker_dist')

    def odom_pose(self):
        """(x, y, yaw) 또는 None."""
        return self._latest('odom')

    def odom_yaw(self):
        pose = self._latest('odom')
        return None if pose is None else pose[2]

    def forward_scan_points(self, max_range):
        """전방 스캔을 (각도, 거리) 목록으로. Follow the Gap 용."""
        scan = self._latest('scan')
        if scan is None:
            return []
        points = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= scan.range_min:
                continue
            a = scan.angle_min + i * scan.angle_increment
            if abs(a) > math.pi / 2.0:
                continue
            points.append((a, min(r, max_range)))
        points.sort(key=lambda p: p[0])
        return points

    # ------------------------------------------------------------------
    # 제어 출력
    # ------------------------------------------------------------------
    def publish_cmd(self, v, omega):
        if not (math.isfinite(v) and math.isfinite(omega)):
            self.get_logger().warn('비정상 제어값 - 정지')
            self.publish_stop()
            return
        if self.ack_msg_type is not None:
            self._publish_ackermann(float(v), float(omega))
            return
        msg = Twist()
        msg.linear.x = float(v)
        if self.cmd_angular_is_steering:
            # Gazebo gazebo_ros_ackermann_drive 는 angular.z 를 각속도가 아니라 조향각으로 쓴다
            # (target_rot = angular.z * sign(linear.x)). 실차 limo_base 는 각속도(r = v / w).
            # 판단은 어디서나 각속도로 계산하고, 보낼 때만 같은 곡률의 조향각으로 바꾼다.
            if abs(v) > 1e-3:
                steer = math.atan(self.wheel_base * float(omega) / abs(v))
                msg.angular.z = float(max(-self.max_steer, min(self.max_steer, steer)))
            else:
                msg.angular.z = 0.0
        else:
            msg.angular.z = float(omega)
        self.cmd_pub.publish(msg)
        self.last_v = float(v)

    def ackermann_steer(self, v, omega):
        """(11/13) 회전속도 -> AckermannDrive.steering_angle. 같은 곡률의 자전거 모델 조향각 x ackermann_steer_scale.
        후진은 부호 있는 속도 그대로 (조향각이 같으면 후진 때 도는 방향이 반대 = 실차 바퀴와 같음).
        거의 서 있으면 직전 조향각 유지 (출발할 때 바퀴가 엉뚱한 쪽으로 안 가게)."""
        if abs(v) > 1e-3:
            steer = math.atan(self.wheel_base * omega / v) * self.ackermann_steer_scale
            lim = self.ackermann_max_steer
            self.last_ack_steer = max(-lim, min(lim, steer))
        return self.last_ack_steer

    def _publish_ackermann(self, v, omega):
        msg = self.ack_msg_type()
        msg.speed = float(v)
        msg.steering_angle = float(self.ackermann_steer(v, omega))
        self.cmd_pub.publish(msg)
        self.last_v = float(v)

    def publish_stop(self):
        if self.ack_msg_type is not None:
            msg = self.ack_msg_type()
            msg.speed = 0.0
            msg.steering_angle = float(self.last_ack_steer)
            self.cmd_pub.publish(msg)
        else:
            self.cmd_pub.publish(Twist())
        self.last_v = 0.0


# ----------------------------------------------------------------------
# 상태머신 구성
# ----------------------------------------------------------------------
def build_state_machine(node):
    sm = smach.StateMachine(outcomes=['shutdown'])

    with sm:
        lane = LaneFollowing(node)
        transitions = dict((o, OUTCOME_TO_STATE[o]) for o in lane.OUTCOMES)
        transitions['aborted'] = 'LANE_FOLLOWING'
        smach.StateMachine.add('LANE_FOLLOWING', lane, transitions=transitions)

        for label in MISSION_LABELS:
            cls = IMPLEMENTED.get(label)
            state = cls(node) if cls else StubState(node, label)
            transitions = {'done': 'LANE_FOLLOWING', 'aborted': 'LANE_FOLLOWING'}
            # 미션끼리 바로 넘어가는 경우 (예: SLOW_ZONE -> CROSSWALK_STOP)
            for outcome in getattr(state, 'HANDOFF', ()):
                transitions[outcome] = OUTCOME_TO_STATE[outcome]
            smach.StateMachine.add(label, state, transitions=transitions)

    return sm


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()

    # SMACH execute() 가 블로킹이므로 ROS 콜백은 별도 스레드에서 처리
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sm = build_state_machine(node)
    try:
        sm.execute()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
