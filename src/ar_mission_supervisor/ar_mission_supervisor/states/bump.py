"""미션 5 - 서행 구간 (방지턱).

진입: YOLO 가 방지턱 표지판을 인식하면 (11/17: 서행 표지판은 이제 SLOW, states/slow.py. SLOW 에서 방지턱 표지판을 보면 여기로).
동작: 차선 추종은 그대로 두고 속도만 speed(0.2 m/s)로 낮춘다.
끝 (11/15 사용자: 표지판 지나고 3초):
  표지판까지 거리(YOLO)를 odom 으로 다 가면(= 표지판 옆) 그때부터 after_sign_sec 초 뒤 LANE_FOLLOWING 으로 복귀.
  after_sign_sec 가 0 이면 예전 방식 (11/05): 표지판까지 거리 + after_sign_distance 를 다 가면 끝.
  - 표지판을 지나기 전에는 새 측정이 올 때마다 표지판 위치를 다시 잡는다 (가까울수록 정확). 지난 뒤엔 안 바꿈.
  - 옆으로 먼 표지판 검출은 안 쓴다 (supervisor._yolo_lateral_ok, yolo.max_lateral).
  - 표지판 위치를 한 번도 못 받았으면 표지판까지 unknown_sign_distance 로 친다.
  - odom 이 없으면 예전처럼 진입 후 duration_sec 뒤 끝.
  - 무슨 일이 있어도 max_duration 이 지나면 끝.
  (lane_yolo_run1: 표지판을 1.7 m 앞에서 보고 8초 만에 끝나 방지턱 1 m 전에 서행이 풀렸음)
규정: 기본 주행 속도의 1/2 수준, 최대 0.2 m/s 이하. 심사위원 육안 판정.
(11/15) 서행 표지판이 모든 서행 구간 앞에 놓이게 바뀜 (사용자) -> 그때는 서행 표지판도 BUMP 라, 지그재그 앞 서행 표지판을
  BUMP 안에서 보면 끝 지점이 늘어나 지그재그까지 BUMP 로 지나갈 수 있어서 BUMP 중 서행 유도선이면 SLOW_ZONE 으로 넘겼음.
(11/17 사용자) 서행 표지판이 SLOW 로 나뉘어 그 이유가 없어짐 -> 끝 지점은 방지턱 표지판만 보고, 다른 미션으로 안 넘어감
  (방지턱과 지그재그는 한참 떨어져 있음).
"""

import math

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.lane_driver import LaneDriver


class Bump(SafeState):

    OUTCOMES = ['done']                  # (11/17 사용자) 다른 미션으로 안 넘어감 (11/15 의 SLOW_ZONE 넘어가기 뺌)
    MAX_STEP = 0.5      # odom 이 한 번에 이보다 많이 움직이면 (순간이동·리셋) 거리에 안 더함 [m]

    def __init__(self, node):
        SafeState.__init__(self, node, 'BUMP', self.OUTCOMES)
        self.driver = LaneDriver(node)
        self.entered_at = None
        self.travelled = 0.0
        self.last_pose = None
        self.sign_at = None         # 진입 후 이 거리를 가면 표지판 옆 [m]
        self.passed_at = None       # 표지판을 지난 시각 (after_sign_sec 방식)
        self.end_at = None          # 진입 후 이 거리를 가면 끝 [m] (after_sign_sec 0 일 때)
        self.sign_stamp = None      # 마지막으로 쓴 표지판 측정의 수신 시각

    def on_enter(self):
        node = self.node
        cfg = node.bump
        self.driver.reset()
        self.entered_at = node.clock_sec()
        self.travelled = 0.0
        self.last_pose = node.odom_pose()
        self.sign_at = None
        self.passed_at = None
        self.end_at = None
        self.sign_stamp = None
        sign = self._update_sign()
        if self.sign_at is None:
            self.sign_at = cfg['unknown_sign_distance']
            self._set_end()
        if cfg['after_sign_sec'] > 0.0:
            after = '지나고 %.1f초 뒤 끝' % cfg['after_sign_sec']
        else:
            after = '+ %.2fm = %.2fm 가면 끝' % (cfg['after_sign_distance'], self.end_at)
        if self.last_pose is None:
            how = 'odom 없음 - %.1f초 뒤 끝' % cfg['duration_sec']
        elif sign is None:
            how = '표지판 거리 모름 - %.1fm 로 치고 %s' % (cfg['unknown_sign_distance'], after)
        else:
            how = '표지판 %.2fm %s' % (sign, after)
        node.get_logger().info('[BUMP] 서행 시작 - %.2f m/s, %s' % (cfg['speed'], how))

    def _set_end(self):
        self.end_at = self.sign_at + self.node.bump['after_sign_distance']

    def _update_sign(self):
        """표지판을 지나기 전, 새 측정이 있으면 표지판 위치를 다시 잡는다. 쓴 거리(또는 None)를 돌려준다."""
        if self.passed_at is not None:
            return None
        m = self.node.bump_sign_measurement()
        if m is None or m[1] == self.sign_stamp:
            return None
        dist, stamp = m
        self.sign_stamp = stamp
        self.sign_at = self.travelled + max(0.0, dist)
        self._set_end()
        return dist

    def run(self):
        node = self.node
        cfg = node.bump

        while node.ok():
            if self.reset_requested():
                return 'aborted'

            elapsed = node.clock_sec() - self.entered_at
            node.dbg('bump/elapsed', elapsed)
            if elapsed >= cfg['max_duration']:
                node.get_logger().warn('[BUMP] %.0f초 넘음 - 서행 종료 (%.2fm 주행)'
                                       % (cfg['max_duration'], self.travelled))
                return 'done'

            pose = node.odom_pose()
            if pose is None:
                if elapsed >= cfg['duration_sec']:
                    node.get_logger().info('[BUMP] odom 없음, %.1f초 경과 - 서행 종료' % elapsed)
                    return 'done'
            else:
                if self.last_pose is not None:
                    step = math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
                    if step <= self.MAX_STEP:
                        self.travelled += step
                self.last_pose = pose
                self._update_sign()
                node.dbg('bump/travelled', self.travelled)
                if cfg['after_sign_sec'] > 0.0:
                    # (11/15) 표지판 옆을 지나면 그때부터 after_sign_sec 초
                    now = node.clock_sec()
                    if self.passed_at is None and self.travelled >= self.sign_at:
                        self.passed_at = now
                        node.get_logger().info('[BUMP] 표지판 지남 (%.2fm 주행) - %.1f초 뒤 서행 종료'
                                               % (self.travelled, cfg['after_sign_sec']))
                    if self.passed_at is None:
                        node.dbg('bump/remaining', self.sign_at - self.travelled)
                    else:
                        left = cfg['after_sign_sec'] - (now - self.passed_at)
                        node.dbg('bump/remaining_sec', left)
                        if left <= 0.0:
                            node.get_logger().info('[BUMP] 표지판 지나고 %.1f초 (%.2fm 주행, %.1f초) - 서행 종료'
                                                   % (cfg['after_sign_sec'], self.travelled, elapsed))
                            return 'done'
                else:
                    node.dbg('bump/remaining', self.end_at - self.travelled)
                    if self.travelled >= self.end_at:
                        node.get_logger().info('[BUMP] %.2fm 주행 (%.1f초) - 서행 종료'
                                               % (self.travelled, elapsed))
                        return 'done'

            self.driver.step(cfg['speed'])
            self.sleep()
        return 'aborted'
