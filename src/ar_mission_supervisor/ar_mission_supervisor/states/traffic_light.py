"""미션 1 - 신호등 정지·발진 (방해 광고판 포함).

진입: YOLO 가 신호등을 인식하면.
동작:
  1) 차선 추종을 유지하며 approach_speed(0.2 m/s)로 정지선에 접근한다.
     접근 중에는 신호 색과 상관없이 계속 접근한다.
  2) 앞바퀴 ~ 정지선이 stop_gap 이하가 된 순간 신호를 본다.
       - 초록불  : 멈추지 않고 바로 LANE_FOLLOWING 으로 돌아가 통과한다
                   (이 State 의 느린 속도로 지나가다 빨간불에 걸리지 않게).
       - 빨강/노랑/색 모름 : 정지.
  3) 정지 중 초록불이 green_confirm_sec 이상 보이면 LANE_FOLLOWING 으로 복귀.
규정: 정지선 ±5cm 안에 완전정지, 녹색 신호 후 15초 이내 출발.
"""

from ar_msgs.msg import MissionObject

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.lane_driver import LaneDriver
from ar_mission_supervisor.stopline import approach_speed


class TrafficLight(SafeState):

    OUTCOMES = ['done']

    APPROACH = 0
    WAIT = 1

    def __init__(self, node):
        SafeState.__init__(self, node, 'TRAFFIC_LIGHT', self.OUTCOMES)
        self.driver = LaneDriver(node)
        self.tracker = node.stopline
        self.phase = self.APPROACH
        self.entered_at = None
        self.wait_started = None
        self.green_since = None
        self.cmd_speed = 0.0

    def on_enter(self):
        self.driver.reset()
        self.tracker = self.node.stopline        # 공용 추정기 (SLOW_ZONE 에서 이어받을 수 있음)
        self.tracker.begin(self.node.clock_sec())
        self.phase = self.APPROACH
        self.entered_at = self.node.clock_sec()
        self.wait_started = None
        self.green_since = None
        self.cmd_speed = 0.0

    def run(self):
        node = self.node
        cfg = node.traffic_light

        while node.ok():
            if self.reset_requested():
                return 'aborted'

            now = node.clock_sec()
            self.tracker.update(now, node.last_v)
            gap = self.tracker.gap()
            color = node.light_color()
            node.dbg('traffic_light/gap', gap)
            node.dbg('traffic_light/color', node.light_name(color))
            node.dbg('traffic_light/phase', 'APPROACH' if self.phase == self.APPROACH else 'WAIT')

            if self.phase == self.APPROACH:
                result = self._approach(now, cfg, gap, color)
                if result is not None:
                    return result
            else:
                result = self._wait(now, cfg, color)
                if result is not None:
                    return result

            self.sleep()
        return 'aborted'

    # ------------------------------------------------------------------
    def _approach(self, now, cfg, gap, color):
        node = self.node
        at_line = gap is not None and gap <= cfg['stop_gap']
        timed_out = gap is None and (now - self.entered_at) > cfg['approach_timeout']

        if not (at_line or timed_out):
            self.cmd_speed = approach_speed(cfg, gap)
            self.driver.step(self.cmd_speed)
            return None

        if timed_out:
            node.get_logger().warn('[TRAFFIC_LIGHT] 정지선 거리 미확보 - 현 위치에서 신호 판정')

        if color == MissionObject.LIGHT_GREEN:
            node.get_logger().info('[TRAFFIC_LIGHT] 정지선 도달, 초록불 - 정지 없이 통과')
            return 'done'

        node.get_logger().info('[TRAFFIC_LIGHT] 정지선 도달 (앞바퀴~선 %s) - %s 신호, 정지'
                               % ('%.3fm' % gap if gap is not None else '?',
                                  node.light_name(color)))
        self._stop(now)
        return None

    def _wait(self, now, cfg, color):
        node = self.node
        node.publish_stop()

        if color == MissionObject.LIGHT_GREEN:
            if self.green_since is None:
                self.green_since = now
            if now - self.green_since >= cfg['green_confirm_sec']:
                node.get_logger().info('[TRAFFIC_LIGHT] 초록불 확인 (%.1f초 정지) - 출발'
                                       % (now - self.wait_started))
                return 'done'
        else:
            self.green_since = None

        if cfg['max_wait_sec'] > 0.0 and now - self.wait_started > cfg['max_wait_sec']:
            node.get_logger().warn('[TRAFFIC_LIGHT] %.0f초 동안 초록불 미확인 - 출발'
                                   % cfg['max_wait_sec'])
            return 'done'
        return None

    def _stop(self, now):
        self.phase = self.WAIT
        self.wait_started = now
        self.green_since = None
        self.cmd_speed = 0.0
        self.node.publish_stop()
