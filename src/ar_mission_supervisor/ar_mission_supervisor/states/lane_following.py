"""기본 주행 State. 차선 유실 페일세이프(직전 명령 유지 -> 저속 직진 -> 정지)는 LaneDriver 내부에서 처리한다."""

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

    def on_enter(self):
        self.driver.reset()

    def run(self):
        node = self.node
        while node.ok():
            if self.reset_requested():
                return 'aborted'

            outcome = node.detect_mission()
            if outcome is not None:
                return outcome

            self.driver.step(node.base_speed)
            self.sleep()
        return 'aborted'