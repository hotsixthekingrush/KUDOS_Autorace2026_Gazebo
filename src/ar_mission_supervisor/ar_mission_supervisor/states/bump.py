"""미션 5 - 서행 구간 (방지턱).

진입: YOLO 가 서행 표지판 또는 방지턱 표지판을 인식하면.
동작: 차선 추종은 그대로 두고 속도만 speed(0.2 m/s)로 낮춘다.
      duration_sec(8초)가 지나면 LANE_FOLLOWING 으로 자동 복귀.
규정: 기본 주행 속도의 1/2 수준, 최대 0.2 m/s 이하. 심사위원 육안 판정.
"""

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.lane_driver import LaneDriver


class Bump(SafeState):

    OUTCOMES = ['done']

    def __init__(self, node):
        SafeState.__init__(self, node, 'BUMP', self.OUTCOMES)
        self.driver = LaneDriver(node)
        self.entered_at = None

    def on_enter(self):
        self.driver.reset()
        self.entered_at = self.node.clock_sec()
        self.node.get_logger().info('[BUMP] 서행 시작 - %.2f m/s, %.1f초'
                                    % (self.node.bump['speed'], self.node.bump['duration_sec']))

    def run(self):
        node = self.node
        cfg = node.bump

        while node.ok():
            if self.reset_requested():
                return 'aborted'

            elapsed = node.clock_sec() - self.entered_at
            node.dbg('bump/elapsed', elapsed)
            if elapsed >= cfg['duration_sec']:
                node.get_logger().info('[BUMP] %.1f초 경과 - 서행 종료' % elapsed)
                return 'done'

            self.driver.step(cfg['speed'])
            self.sleep()
        return 'aborted'
