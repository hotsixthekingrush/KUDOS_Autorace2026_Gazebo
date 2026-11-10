"""기본 주행 State. 차선 유실 페일세이프(직전 명령 유지 -> 저속(직전 조향각 유지) -> 정지)는 LaneDriver 내부에서 처리한다.

(11/10) 코너 감속: 카메라 차선 방향(heading)이 lane.corner_heading_deg 보다 크게 틀어졌거나
확신도가 lane.corner_min_confidence 보다 낮으면 lane.corner_speed 로 간다. 조건이 풀려도
lane.corner_hold_sec 동안은 감속을 유지한다. 차선 팀원 시험: 0.4 m/s 는 두 번째 코너에서 이탈, 0.2 m/s 는 통과.
코너 입구에서 확신도가 먼저 0 으로 떨어지고(lane_yolo_run3 x 8.2), 지그재그에서도 방향 -27~-37도라 같이 느려진다.
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

    def on_enter(self):
        self.driver.reset()
        self.slow_until = None
        self.slow = False

    def _speed(self, now):
        """기본 속도 또는 코너 감속 속도."""
        node = self.node
        cfg = node.lane_gains
        lane = node.lane_error(now)
        if lane is not None:
            uncertain = lane.confidence < cfg['corner_min_confidence']
            turned = abs(lane.heading) > math.radians(cfg['corner_heading_deg'])
            if uncertain or turned:
                self.slow_until = now + cfg['corner_hold_sec']
                if not self.slow:
                    node.get_logger().info('[LANE] 감속 %.2f m/s (차선 방향 %.0f도, 확신도 %.2f)'
                                           % (cfg['corner_speed'], math.degrees(lane.heading),
                                              lane.confidence))
        slow = self.slow_until is not None and now < self.slow_until
        if self.slow and not slow:
            node.get_logger().info('[LANE] 기본 속도로 (%.2f m/s)' % node.base_speed)
        self.slow = slow
        node.dbg('lane/corner_slow', 1.0 if slow else 0.0)
        return min(node.base_speed, cfg['corner_speed']) if slow else node.base_speed

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
