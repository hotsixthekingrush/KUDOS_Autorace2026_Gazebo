"""기본 주행 State. 차선 유실 페일세이프(직전 명령 유지 -> 저속(직전 조향각 유지) -> 정지)는 LaneDriver 내부에서 처리한다.

(11/10) 코너 감속: 카메라 차선 방향(heading)이 lane.corner_heading_deg 보다 크게 틀어졌거나
확신도가 lane.corner_min_confidence 보다 낮으면 lane.corner_speed 로 간다. 조건이 풀려도
lane.corner_hold_sec 동안은 감속을 유지한다. 차선 팀원 시험: 0.4 m/s 는 두 번째 코너에서 이탈, 0.2 m/s 는 통과.
코너 입구에서 확신도가 먼저 0 으로 떨어지고(lane_yolo_run3 x 8.2), 지그재그에서도 방향 -27~-37도라 같이 느려진다.
(11/11) 차선 노드가 한 선만 보고 추정한 프레임(single_lane_estimated)도 감속 조건 (lane.corner_single_lane).
  차선 팀원 새 방식은 코너 입구(안쪽 선이 화면 밖)에서 한 선 추정으로 바뀌고 확신도는 높을 수도 있어서.
(11/12) lane.speed_by_lateral (기본, 차선 팀원 제안 = 예전 limo_control 식): 위 코너 감속 대신
  속도 = base_speed * max(min_ratio, 1 - |lateral| / slow_lat), 한 선 추정이면 min_ratio, 최저 min_speed.
  목표 속도로 갈 때 가속 accel_max, 감속 decel_max 까지 (새 차선 값마다 속도가 툴툴 바뀌지 않게).
  (11/15) 최저 비율로 줄이는 조건 = lane.slow_rule: 'quality'(기본) 확신도 < corner_min_confidence 또는
  |방향| > corner_heading_deg. 'single' = 예전처럼 한 선 추정이면 (실차는 한 선이 평소라 거의 항상 최저 속도였음).
  조향은 LaneDriver 의 omega_pd. 팀원 Gazebo: 원본(코너 평균 0.42 m/s) 첫 코너 이탈 -> 이 방식(0.18 m/s) 4/4 통과.
(11/16 사용자) 3차선 입구 직진 (entry_hold): 우회전을 끝내 차 방향이 3차선 방향(avoid_static.three_lane_axis_deg) 에서
  entry_hold.start_deg 안으로 들어오면, entry_hold.distance 만큼은 차선을 따라가지 않고 방향만 잡고 곧게 간다.
  3차선 입구는 사선으로 넓어져서 차선 추종이 왼쪽 선을 따라 왼쪽 차선으로 들어가는 일이 있음 (차선 팀원) -> AVOID_STATIC 이
  '지금 가운데 차선' 으로 가정하는 것과 어긋남. 들어오던 차선은 가운데 차선으로 곧게 이어지므로 곧게 가면 가운데 차선.
  트랙이 시계 방향 한 바퀴라 3차선 방향으로 꺾이는 코너는 한 곳뿐. 한 번 하면 방향이 arm_deg 넘게 벗어났다 돌아올 때까지 다시 안 함.
  판단을 켠 직후(차를 3차선 앞에 놓고 start_yaw_deg:=180)에도 바로 함. 미션 이벤트는 그대로 봄 (AVOID_STATIC 이 오면 거기서 끝).
"""

import math

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.controllers import clamp, steering_to_omega
from ar_mission_supervisor.geometry import normalize_angle
from ar_mission_supervisor.lane_driver import LaneDriver


class LaneFollowing(SafeState):

    OUTCOMES = ['to_traffic_light', 'to_left_a_right_b', 'to_crosswalk',
                'to_parking', 'to_bump', 'to_slow_zone', 'to_avoid_static',
                'to_cone', 'to_slow']

    def __init__(self, node):
        SafeState.__init__(self, node, 'LANE_FOLLOWING', self.OUTCOMES,
                           lockout_on_exit=False)
        self.driver = LaneDriver(node)
        self.slow_until = None
        self.slow = False
        self.v_cmd = None           # speed_by_lateral: 가속·감속 제한을 건 지금 속도
        self.v_time = None
        self.hold = None            # (11/16) 3차선 입구 직진 중이면 (시작 x, 시작 y, 고정 방향)

    def on_enter(self):
        self.driver.reset()
        self.hold = None
        self.slow_until = None
        self.slow = False
        self.v_cmd = None
        self.v_time = None

    def _speed(self, now):
        """기본 속도 또는 감속 속도."""
        node = self.node
        cfg = node.lane_gains
        lane = node.lane_error(now)
        if cfg['speed_by_lateral']:
            return self._lateral_speed(lane, now)
        if lane is not None:
            uncertain = lane.confidence < cfg['corner_min_confidence']
            turned = abs(lane.heading) > math.radians(cfg['corner_heading_deg'])
            single = cfg['corner_single_lane'] and lane.single
            if uncertain or turned or single:
                self.slow_until = now + cfg['corner_hold_sec']
                if not self.slow:
                    node.get_logger().info('[LANE] 감속 %.2f m/s (차선 방향 %.0f도, 확신도 %.2f%s)'
                                           % (cfg['corner_speed'], math.degrees(lane.heading),
                                              lane.confidence, ', 한 선 추정' if lane.single else ''))
        slow = self.slow_until is not None and now < self.slow_until
        if self.slow and not slow:
            node.get_logger().info('[LANE] 기본 속도로 (%.2f m/s)' % node.base_speed)
        self.slow = slow
        node.dbg('lane/corner_slow', 1.0 if slow else 0.0)
        return min(node.base_speed, cfg['corner_speed']) if slow else node.base_speed

    @staticmethod
    def _slow_frame(lane, cfg):
        """speed_by_lateral 에서 최저 비율(min_ratio)로 줄일 차선 값인지.
        (11/15, 차선 팀원 제안) slow_rule 'quality' (기본): 확신도 < corner_min_confidence 또는 |방향| > corner_heading_deg.
          실차는 한 선으로 경로를 만드는 게 평소 상태라(차선 팀원 real_1114_e: 직선에서도 79%, single 표시 고친 뒤에도 83%)
          single 로 줄이면 거의 항상 최저 속도 -> 느릴수록 조향이 세져 흔들림 (lane_run3 도 single 70%, 목표 속도 70% 가 0.12).
        'single': 예전 (11/12) 처럼 single_lane_estimated 면 줄임."""
        if str(cfg.get('slow_rule', 'quality')) == 'single':
            return bool(lane.single)
        return (lane.confidence < cfg['corner_min_confidence']
                or abs(lane.heading) > math.radians(cfg['corner_heading_deg']))

    def _lateral_speed(self, lane, now):
        """(11/12) 차선 오차에 비례해 감속 + 가속·감속 제한."""
        node = self.node
        cfg = node.lane_gains
        base = node.base_speed
        target = base
        if lane is not None:
            ratio = max(cfg['min_ratio'], 1.0 - abs(lane.lateral) / max(cfg['slow_lat'], 1e-6))
            if self._slow_frame(lane, cfg):
                ratio = cfg['min_ratio']
            target = max(min(base, cfg['min_speed']), base * ratio)
        # 기준은 마지막으로 실제 보낸 속도 (페일세이프·정지 뒤에도 거기서부터 이어짐)
        prev = max(0.0, getattr(node, 'last_v', 0.0))
        dt = 0.05 if self.v_time is None else max(0.0, min(now - self.v_time, 0.5))
        self.v_cmd = min(prev + cfg['accel_max'] * dt, max(prev - cfg['decel_max'] * dt, target))
        self.v_time = now
        node.dbg('lane/speed_target', target)
        node.dbg('lane/speed', self.v_cmd)
        return self.v_cmd

    def _entry_hold(self, now, v):
        """(11/16) 3차선 입구 직진. 이번 주기 명령을 여기서 보냈으면 True."""
        node = self.node
        cfg = getattr(node, 'entry_hold', None)
        if cfg is None or not cfg['enable'] or not node.avoid_static['three_lane_axis_use']:
            self.hold = None
            return False
        pose = node.odom_pose()
        if pose is None:
            self.hold = None
            return False
        axis = node.axis_rad(node.avoid_static['three_lane_axis_deg'])   # (11/18) 트랙 방향 자동 보정 포함
        err = normalize_angle(axis - pose[2])
        log = node.get_logger()
        if self.hold is None:
            if abs(err) > math.radians(cfg['arm_deg']):
                node.entry_armed = True
                return False
            if not node.entry_armed or abs(err) > math.radians(cfg['start_deg']):
                return False
            node.entry_armed = False
            self.hold = (pose[0], pose[1], axis)
            log.info('[LANE] 3차선 입구 직진 시작: 방향 %.0f도로 %.1fm (지금 %.0f도, %s)'
                     % (math.degrees(axis), cfg['distance'], math.degrees(pose[2]),
                        '차선은 안 따라감' if cfg['lane_blend'] <= 0.0
                        else '차선 옆 위치는 조향 %.2f rad 까지만' % cfg['lane_max']))
        dist = math.hypot(pose[0] - self.hold[0], pose[1] - self.hold[1])
        if dist >= cfg['distance'] or abs(err) > math.radians(cfg['abort_deg']):
            log.info('[LANE] 3차선 입구 직진 끝 (%.2fm, 방향 오차 %+.0f도) - 차선 따라가기로'
                     % (dist, math.degrees(err)))
            self.hold = None
            self.driver.reset()
            return False
        steer = cfg['k_yaw'] * err
        lane = node.lane_error(now)
        if lane is not None and cfg['lane_blend'] > 0.0:
            part = cfg['lane_blend'] * clamp(lane[2], 0.0, 1.0) * self.driver.lane_steer(lane, now)
            if cfg['lane_max'] > 0.0:
                # (11/19 사용자) 차선 몫 한계: 입구에서 차선 노드가 바깥 선을 따라가도 크게 안 끌려가게
                part = clamp(part, -cfg['lane_max'], cfg['lane_max'])
            steer += part
            node.dbg('lane/entry_hold_lane', part)
        steer = clamp(steer, -node.max_steer, node.max_steer)
        v = min(v, cfg['max_speed'])
        node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))
        node.dbg('lane/entry_hold', dist)
        node.dbg('lane/entry_hold_err', err)
        return True

    def run(self):
        node = self.node
        while node.ok():
            if self.reset_requested():
                return 'aborted'

            outcome = node.detect_mission()
            if outcome is not None:
                return outcome

            now = node.clock_sec()
            v = self._speed(now)
            cap = node.after_ab['speed_cap']
            if cap > 0.0 and v > cap and node.after_ab_active():
                v = cap                 # (11/19 사용자·차선 팀원) 갈림길 뒤 ~ 라바콘 전 (C3 코너)
                node.dbg('lane/after_ab_cap', cap)
            if not self._entry_hold(now, v):
                self.driver.step(v)
            self.sleep()
        return 'aborted'
