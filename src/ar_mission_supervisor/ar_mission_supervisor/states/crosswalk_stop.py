"""미션 3 - 보행자 표지판 정지.

진입: YOLO 가 보행자 표지판을 인식하면 (또는 SLOW_ZONE 에서 바로 넘어옴).
동작:
  1) 차선 추종을 유지하며 approach_speed(0.2 m/s)로 정지선에 접근.
  2) 앞바퀴 ~ 정지선이 stop_gap 이하가 되면 정지.
  3) wait_sec(4초) 정지 후 LANE_FOLLOWING 으로 복귀.
규정: 정지선 ±5cm 안에 완전정지, 정지 확인 후 3초 이상 대기, 15초 이내 출발.
"""

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.lane_driver import LaneDriver
from ar_mission_supervisor.stopline import approach_speed


class CrosswalkStop(SafeState):

    OUTCOMES = ['done']

    def __init__(self, node):
        SafeState.__init__(self, node, 'CROSSWALK_STOP', self.OUTCOMES)
        self.driver = LaneDriver(node)
        self.tracker = node.stopline
        self.entered_at = None
        self.stopped_at = None
        self.cmd_speed = 0.0

    def on_enter(self):
        self.driver.reset()
        self.tracker = self.node.stopline        # 공용 추정기 (SLOW_ZONE 에서 이어받을 수 있음)
        self.tracker.begin(self.node.clock_sec())
        self.entered_at = self.node.clock_sec()
        self.stopped_at = None
        self.cmd_speed = 0.0

    def run(self):
        node = self.node
        cfg = node.crosswalk

        while node.ok():
            if self.reset_requested():
                return 'aborted'

            now = node.clock_sec()

            # --- 1단계: 정지선까지 접근 ---
            if self.stopped_at is None:
                self.tracker.update(now, node.last_v)
                gap = self.tracker.gap()
                node.dbg('crosswalk/gap', gap)

                if gap is not None and gap <= cfg['stop_gap']:
                    node.get_logger().info('[CROSSWALK] 앞바퀴~정지선 %.3fm - 정지' % gap)
                    self._stop(now)
                elif gap is None and now - self.entered_at > cfg['approach_timeout']:
                    node.get_logger().warn('[CROSSWALK] 정지선 거리 미확보 - 현 위치에서 정지')
                    self._stop(now)
                else:
                    self.cmd_speed = approach_speed(cfg, gap)
                    self.driver.step(self.cmd_speed)

                self.sleep()
                continue

            # --- 2단계: 정지 대기 ---
            node.publish_stop()
            waited = now - self.stopped_at
            node.dbg('crosswalk/waited', waited)
            if waited >= cfg['wait_sec']:
                node.get_logger().info('[CROSSWALK] %.1f초 정지 완료 - 출발' % waited)
                return 'done'

            self.sleep()
        return 'aborted'

    def _stop(self, now):
        self.stopped_at = now
        self.cmd_speed = 0.0
        self.node.publish_stop()
