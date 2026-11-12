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
  조향은 LaneDriver 의 omega_pd. 팀원 Gazebo: 원본(코너 평균 0.42 m/s) 첫 코너 이탈 -> 이 방식(0.18 m/s) 4/4 통과.
"""

import math

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.lane_driver import LaneDriver


class LaneFollowing(SafeState):

    OUTCOMES = ['to_traffic_light', 'to_left_a_right_b', 'to_crosswalk',
                'to_parking', 'to_bump', 'to_slow_zone', 'to_avoid_static',
                'to_cone']

    def __init__(self, node):
        SafeState.__init__(self, node, 'LANE_FOLLOWING', self.OUTCOMES,
                           lockout_on_exit=False)
        self.driver = LaneDriver(node)
        self.slow_until = None
        self.slow = False
        self.v_cmd = None           # speed_by_lateral: 가속·감속 제한을 건 지금 속도
        self.v_time = None

    def on_enter(self):
        self.driver.reset()
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

    def _lateral_speed(self, lane, now):
        """(11/12) 차선 오차에 비례해 감속 + 가속·감속 제한."""
        node = self.node
        cfg = node.lane_gains
        base = node.base_speed
        target = base
        if lane is not None:
            ratio = max(cfg['min_ratio'], 1.0 - abs(lane.lateral) / max(cfg['slow_lat'], 1e-6))
            if lane.single:
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

    def run(self):
        node = self.node
        while node.ok():
            if self.reset_requested():
                return 'aborted'

            outcome = node.detect_mission()
            if outcome is not None:
                return outcome

            self.driver.step(self._speed(node.clock_sec()))
            self.sleep()
        return 'aborted'
