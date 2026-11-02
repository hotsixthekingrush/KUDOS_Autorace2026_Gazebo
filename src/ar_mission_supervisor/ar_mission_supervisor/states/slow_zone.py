"""미션 6 - 서행 유도선 통과 (지그재그 노면).

진입: YOLO 가 노면의 지그재그 서행 유도선을 인식하면.
동작:
  1) 유도선 시작점에 도달할 때까지는 speed(0.2 m/s)로 일반 차선 추종.
     (유도선을 코너 끝에서 미리 볼 수 있으므로 차가 직선에 들어선 뒤 방향을 잡는다)
     시작점 = 진입 순간 YOLO 가 준 유도선 거리 + lock_offset 만큼 주행한 지점 (odom).
     유도선 거리나 odom 이 없으면 진입 후 lock_delay_sec 뒤.
  2) 그 순간의 odom 헤딩을 목표로 고정하고,
       조향 = k_yaw x 헤딩 오차 + lane_blend x confidence x 차선 추종 조향
     으로 직진 헤딩을 유지한다. 지그재그 차선을 그대로 따라가 좌우로 흔들리는 것을 막는다.
     차선이 안 보이면 헤딩 유지만 한다.
  3) 보행자 표지판을 보면 LANE_FOLLOWING 을 거치지 않고 CROSSWALK_STOP 으로 바로 넘어간다.
     표지판은 유도선 입구에서부터 보일 수 있으므로(유도선 1.68m + 정지선까지 약 0.2m),
     유도선 끝 무렵까지는 헤딩 유지를 계속하고 넘어간다:
       - 정지선을 본 적이 있으면 정지선 추정 거리 <= handoff_stopline 일 때
       - 정지선이 안 보이면 표지판 거리 <= handoff_sign 일 때
       - 둘 다 거리 정보가 없으면 바로
     끝내 못 보면 max_duration 뒤 LANE_FOLLOWING 으로 복귀.
규정: 직진 헤딩을 유지하며 서행 통과. 일반 차로이탈 기준 적용.
"""

import math

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.controllers import clamp, steering_to_omega
from ar_mission_supervisor.geometry import normalize_angle
from ar_mission_supervisor.lane_driver import LaneDriver


class SlowZone(SafeState):

    OUTCOMES = ['done', 'to_crosswalk']
    HANDOFF = ('to_crosswalk',)          # 바로 넘어갈 수 있는 미션

    def __init__(self, node):
        SafeState.__init__(self, node, 'SLOW_ZONE', self.OUTCOMES)
        self.driver = LaneDriver(node)
        self.entered_at = None
        self.target_yaw = None
        self.start_pose = None
        self.lock_after = None

    def on_enter(self):
        node = self.node
        self.driver.reset()
        self.entered_at = node.clock_sec()
        self.target_yaw = None
        # 보행자 정지선 추정을 여기서부터 이어간다 (CROSSWALK_STOP 이 이어받음)
        node.stopline.begin(self.entered_at)
        # 헤딩 고정 시점: 유도선 시작점까지 남은 거리
        self.start_pose = node.odom_pose()
        line = node.slow_line_distance()
        self.lock_after = None
        if line is not None and self.start_pose is not None:
            self.lock_after = max(0.0, line) + node.slow_zone['lock_offset']
            node.get_logger().info('[SLOW_ZONE] 진입 - %.2fm 더 가서 헤딩 고정' % self.lock_after)
        else:
            node.get_logger().info('[SLOW_ZONE] 진입 - 유도선 거리 모름, %.1f초 뒤 헤딩 고정'
                                   % node.slow_zone['lock_delay_sec'])

    def run(self):
        node = self.node
        cfg = node.slow_zone

        while node.ok():
            if self.reset_requested():
                return 'aborted'

            now = node.clock_sec()
            node.stopline.update(now, node.last_v)

            outcome = node.detect_mission(allowed=self.HANDOFF)
            if outcome is not None and self._near_crosswalk(cfg):
                node.get_logger().info('[SLOW_ZONE] 보행자 표지판 인식 - CROSSWALK_STOP 으로 바로 전환')
                return outcome

            elapsed = now - self.entered_at
            if elapsed > cfg['max_duration']:
                node.get_logger().warn('[SLOW_ZONE] 보행자 표지판 미인식 %.0f초 - 복귀'
                                       % cfg['max_duration'])
                return 'done'

            yaw = node.odom_yaw()
            if self.target_yaw is None and yaw is not None and self._ready_to_lock(elapsed):
                self.target_yaw = yaw
                node.get_logger().info('[SLOW_ZONE] 헤딩 고정 %.1f도' % math.degrees(yaw))

            if self.target_yaw is None or yaw is None:
                # 고정 전이거나 odom 이 끊김 -> 일반 차선 추종으로 서행
                node.dbg('slow_zone/mode', 'lane')
                self.driver.step(cfg['speed'])
            else:
                self._hold_heading(now, yaw, cfg)

            self.sleep()
        return 'aborted'

    def _ready_to_lock(self, elapsed):
        node = self.node
        cfg = node.slow_zone
        pose = node.odom_pose()
        if self.lock_after is None or pose is None:
            return elapsed >= cfg['lock_delay_sec']
        travelled = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1])
        node.dbg('slow_zone/travelled', travelled)
        return travelled >= self.lock_after

    def _near_crosswalk(self, cfg):
        """보행자 표지판을 본 상태에서, 넘어갈 만큼 정지선에 가까운지."""
        node = self.node
        line = node.stopline.est            # 정지선 추정 (화면 밖으로 빠져도 이어서 추정)
        if line is not None:
            node.dbg('slow_zone/handoff_line', line)
            return line <= cfg['handoff_stopline']
        sign = node.pedestrian_sign_distance()
        if sign is not None and cfg['handoff_sign'] > 0.0:
            node.dbg('slow_zone/handoff_sign', sign)
            return sign <= cfg['handoff_sign']
        return True                      # 거리 정보가 전혀 없으면 바로 넘어간다

    def _hold_heading(self, now, yaw, cfg):
        node = self.node
        yaw_err = normalize_angle(self.target_yaw - yaw)
        steer = cfg['k_yaw'] * yaw_err

        lane = node.lane_error(now)
        lane_part = 0.0
        if lane is not None and cfg['lane_blend'] > 0.0:
            confidence = clamp(lane[2], 0.0, 1.0)
            lane_part = cfg['lane_blend'] * confidence * self.driver.lane_steer(lane, now)
        steer = clamp(steer + lane_part, -node.max_steer, node.max_steer)

        v = cfg['speed']
        node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))

        node.dbg('slow_zone/mode', 'heading')
        node.dbg('slow_zone/yaw_err', yaw_err)
        node.dbg('slow_zone/lane_part', lane_part)
        node.dbg('slow_zone/steer', steer)
