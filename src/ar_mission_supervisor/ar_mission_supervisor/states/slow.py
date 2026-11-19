"""(11/17 사용자) 서행 표지판 -> SLOW.

진입: YOLO 서행 표지판(MissionObject.SLOW_SIGN, 세워진 원형 표지판). 방지턱 표지판(BUMP_SIGN)은 예전처럼 BUMP.
동작: 차선 추종은 그대로, 속도만 slow.speed (0.2 m/s).
나가기: 아래 네 미션으로만 바로 넘어간다 (사용자). 서행 표지판은 서행 구간마다 그 앞에 놓임 (11/15).
  신호등(TRAFFIC_LIGHT), 지그재그 유도선(SLOW_ZONE), 방지턱 표지판(BUMP), 보행자 표지판(CROSSWALK_STOP).
  다른 이벤트(A/B, 주차, 라바콘, LIMO)는 SLOW 안에서는 안 봄.
  slow.max_duration > 0 이면 그 시간 뒤 LANE_FOLLOWING 으로 (서행 표지판을 잘못 봤을 때 남은 바퀴를 다 서행하지 않게).
  0 이면 위 네 미션 중 하나가 나올 때까지 계속 서행.
"""

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.lane_driver import LaneDriver


class Slow(SafeState):

    HANDOFF = ('to_traffic_light', 'to_slow_zone', 'to_bump', 'to_crosswalk')
    OUTCOMES = ['done'] + list(HANDOFF)
    NAMES = {'to_traffic_light': '신호등', 'to_slow_zone': '지그재그 유도선', 'to_bump': '방지턱 표지판',
             'to_crosswalk': '보행자 표지판'}

    def __init__(self, node):
        SafeState.__init__(self, node, 'SLOW', self.OUTCOMES)
        self.driver = LaneDriver(node)
        self.entered_at = None

    def on_enter(self):
        node = self.node
        self.driver.reset()
        self.entered_at = node.clock_sec()
        cfg = node.slow
        node.get_logger().info('[SLOW] 서행 표지판 - %.2f m/s 로 차선 따라감 (신호등·지그재그·방지턱·보행자 표지판으로만 넘어감%s)'
                               % (cfg['speed'], '' if cfg['max_duration'] <= 0.0
                                  else ', %.0f초 뒤 끝' % cfg['max_duration']))

    def run(self):
        node = self.node
        cfg = node.slow
        while node.ok():
            if self.reset_requested():
                return 'aborted'
            elapsed = node.clock_sec() - self.entered_at
            node.dbg('slow/elapsed', elapsed)
            if cfg['max_duration'] > 0.0 and elapsed >= cfg['max_duration']:
                node.get_logger().warn('[SLOW] %.0f초 동안 다음 미션이 안 나옴 - 서행 끝' % cfg['max_duration'])
                return 'done'
            outcome = node.detect_mission(allowed=self.HANDOFF)
            if outcome is not None:
                node.get_logger().info('[SLOW] %s 인식 (%.1f초 서행) - 바로 전환' % (self.NAMES[outcome], elapsed))
                return outcome
            self.driver.step(cfg['speed'])
            self.sleep()
        return 'aborted'
