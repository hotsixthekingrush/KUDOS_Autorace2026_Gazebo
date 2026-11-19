"""미션 3 - 보행자 표지판 정지.

진입: YOLO 가 보행자 표지판을 인식하면 (또는 SLOW_ZONE 에서 바로 넘어옴).
동작:
  1) approach_speed(0.2 m/s)로 정지선에 접근.
     (11/16 사용자) 횡단보도가 지그재그 바로 뒤라 접근 중에도 SLOW_ZONE 처럼 헤딩 고정 (heading_hold).
     SLOW_ZONE 에서 넘어오면 그 고정 헤딩을 이어받고, LANE_FOLLOWING 에서 바로 오면(유도선을 못 잡음, stop_run3)
     회전이 잦아든 순간 고정 (트랙 축에서 hold_axis_snap_deg 안이면 그 축으로). 고정 전엔 차선 추종.
  2) 앞바퀴 ~ 정지선이 stop_gap 이하가 되면 정지.
  3) wait_sec(4초) 정지 후 LANE_FOLLOWING 으로 복귀.
규정: 정지선 ±5cm 안에 완전정지, 정지 확인 후 3초 이상 대기, 15초 이내 출발.
"""

import math

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.controllers import steering_to_omega
from ar_mission_supervisor.lane_driver import LaneDriver
from ar_mission_supervisor.states.slow_zone import heading_hold_steer
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
        self.target_yaw = None

    def on_enter(self):
        node = self.node
        self.driver.reset()
        self.tracker = node.stopline             # 공용 추정기 (SLOW_ZONE 에서 이어받을 수 있음)
        self.tracker.begin(node.clock_sec())
        self.entered_at = node.clock_sec()
        self.stopped_at = None
        self.cmd_speed = 0.0
        self.target_yaw = None
        hand = node.heading_handoff
        node.heading_handoff = None
        if node.crosswalk['heading_hold'] and hand is not None and self.entered_at - hand[1] <= 0.5:
            self.target_yaw = hand[0]
            node.get_logger().info('[CROSSWALK] 지그재그 고정 헤딩 이어받음 %.1f도' % math.degrees(hand[0]))

    def _try_lock(self, now):
        """(11/16) 회전이 잦아들었으면 헤딩 고정. 트랙 축에 가까우면 그 축으로."""
        node = self.node
        cfg = node.crosswalk
        yaw = node.odom_yaw()
        if yaw is None:
            return
        rate = node.turn_rate(now)
        if rate is not None and abs(rate) > math.radians(cfg['hold_turn_rate_deg']):
            node.dbg('crosswalk/hold_wait_rate', math.degrees(rate))
            return
        axis, err = node.track_axis(yaw)
        if cfg['hold_axis_snap_deg'] > 0.0 and abs(err) <= math.radians(cfg['hold_axis_snap_deg']):
            self.target_yaw = axis
            node.get_logger().info('[CROSSWALK] 헤딩 고정 - 트랙 축 %.0f도 (차 방향 %.1f도)'
                                   % (math.degrees(axis), math.degrees(yaw)))
        else:
            self.target_yaw = yaw
            node.get_logger().info('[CROSSWALK] 헤딩 고정 - 차 방향 %.1f도 (트랙 축에서 %.0f도 벗어남)'
                                   % (math.degrees(yaw), math.degrees(abs(err))))

    def _drive(self, now, v):
        """접근 주행: 헤딩 고정이면 SLOW_ZONE 과 같은 조향, 아니면 차선 추종."""
        node = self.node
        if node.crosswalk['heading_hold']:
            if self.target_yaw is None:
                self._try_lock(now)
            yaw = node.odom_yaw()
            if self.target_yaw is not None and yaw is not None:
                steer, yaw_err, lane_part = heading_hold_steer(node, self.driver, self.target_yaw, yaw, now)
                node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))
                node.dbg('crosswalk/mode', 'heading')
                node.dbg('crosswalk/yaw_err', yaw_err)
                node.dbg('crosswalk/lane_part', lane_part)
                return
        node.dbg('crosswalk/mode', 'lane')
        self.driver.step(v)

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
                    self._drive(now, self.cmd_speed)

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
