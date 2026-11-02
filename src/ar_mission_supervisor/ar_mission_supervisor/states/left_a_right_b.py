"""미션 2 - 갈림길 경로 선택 (A = 좌, B = 우).

A/B 표지판을 인식하면 진입하고, 표지판까지 거리가 임계값 이하가 되는 시점에
지시 방향으로 횡방향 오프셋을 점진적으로 적용해 경로를 옮긴다.
인식 직후 즉시 조향하면 분기 지점과 어긋나 차선을 이탈할 수 있으므로
거리 기반 트리거를 사용한다.
규정: 표지판은 매 주행마다 무작위로 바뀜. 반대 경로 진입 또는 15초 내 미진입 시 실패.

3차선 이어받기 (odom 모드, 2026):
  AVOID_STATIC 에서 바로 넘어왔거나 끝난 지 context_timeout 안이면,
  AVOID_STATIC 의 기준 좌표와 차선 번호를 그대로 이어받는다.
  3차선 끝의 섬이 가운데 차선을 막고 있으므로 A = 왼쪽(a_lane), B = 오른쪽(b_lane) 차선으로
  필요한 만큼만 옮기고(이미 그 차선이면 그대로), odom_hold_distance 만큼 더 간 뒤 종료한다.
  목표 차선 앞에 아직 LIMO 가 있으면 지나갈 때까지 지금 차선을 유지한다.
"""

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.controllers import clamp
from ar_mission_supervisor.lane_driver import LaneDriver


class LeftARightB(SafeState):

    OUTCOMES = ['done']

    APPROACH = 0
    SHIFT = 1
    SETTLE = 2

    def __init__(self, node):
        SafeState.__init__(self, node, 'LEFT_A_RIGHT_B', self.OUTCOMES)
        self.driver = LaneDriver(node)
        self.phase = self.APPROACH
        self.entered_at = None
        self.shift_started = None
        self.settle_started = None
        self.direction = 0.0
        self.plan = node.three_lane
        self.odom_mode = False
        self.goal_lane = None
        self.reached_s = None

    def on_enter(self):
        node = self.node
        self.driver.reset()
        self.phase = self.APPROACH
        self.entered_at = node.clock_sec()
        self.shift_started = None
        self.settle_started = None
        self.direction = 0.0
        self.goal_lane = None
        self.reached_s = None
        self.odom_mode = False
        if node.left_a_right_b['use_lane_context'] and self.plan.fresh(self.entered_at):
            car = self.plan.update(self.entered_at)
            if car is not None:
                self.odom_mode = True
                self.plan.resume()
                if not self.plan.changing:
                    self.plan.lane = self.plan.target = self.plan.lane_of(car[2])
                node.get_logger().info('[LEFT_A_RIGHT_B] 3차선 이어받음 - 지금 %s 차선 (odom 모드)'
                                       % self.plan.name(self.plan.lane))

    def run(self):
        node = self.node
        cfg = node.left_a_right_b

        while node.ok():
            if self.reset_requested():
                return 'aborted'

            now = node.clock_sec()
            if now - self.entered_at > cfg['max_duration']:
                node.get_logger().warn('[LEFT_A_RIGHT_B] 제한시간 초과 - 복귀')
                self.plan.finish()
                return 'done'

            if self.odom_mode:
                if self._odom_step(now, cfg):
                    self.plan.finish()
                    return 'done'
                self.sleep()
                continue

            if self.phase == self.APPROACH:
                if self._approach(now, cfg):
                    return 'done'
            elif self.phase == self.SHIFT:
                self._shift(now, cfg)
            else:
                if self._settle(now, cfg):
                    return 'done'

            self.sleep()

        return 'aborted'

    # ------------------------------------------------------------------
    def _approach(self, now, cfg):
        """표지판까지 접근. 트리거 거리에 도달하면 SHIFT 로 전환.
        True 를 반환하면 미션을 포기하고 복귀한다."""
        node = self.node
        marker = node.marker_type()
        dist = node.marker_distance()

        if marker is not None:
            self.direction = 1.0 if marker == 'A' else -1.0     # A = 좌(+), B = 우(-)

        if dist is not None and dist <= cfg['trigger_distance']:
            if self.direction == 0.0:
                node.get_logger().warn('[LEFT_A_RIGHT_B] 표지판 종류 불명 - 복귀')
                return True
            node.get_logger().info(
                '[LEFT_A_RIGHT_B] %s 경로로 이동 시작 (표지판 %.2fm)'
                % ('A(좌)' if self.direction > 0 else 'B(우)', dist))
            self.phase = self.SHIFT
            self.shift_started = now
            return False

        if now - self.entered_at > cfg['approach_timeout']:
            node.get_logger().warn('[LEFT_A_RIGHT_B] 표지판 거리 미확보 - 복귀')
            return True

        self.driver.step(cfg['approach_speed'])
        return False

    def _shift(self, now, cfg):
        """오프셋을 점진적으로 키워 목표 차선으로 이동."""
        elapsed = now - self.shift_started
        progress = clamp(elapsed / cfg['shift_duration'], 0.0, 1.0)
        offset = self.direction * cfg['lane_width'] * progress

        self.driver.step(cfg['shift_speed'], offset)

        if progress >= 1.0:
            self.phase = self.SETTLE
            self.settle_started = now
            self.node.get_logger().info('[LEFT_A_RIGHT_B] 이동 완료 - 안정화')

    def _settle(self, now, cfg):
        """새 차선에서 잠시 유지한 뒤 종료."""
        offset = self.direction * cfg['lane_width']
        self.driver.step(cfg['shift_speed'], offset)
        return (now - self.settle_started) >= cfg['settle_sec']

    # ------------------------------------------------------------------
    # 3차선 이어받기 (odom 모드)
    # ------------------------------------------------------------------
    def _odom_step(self, now, cfg):
        """True 를 반환하면 종료. 장애물 피하기는 그대로 하면서 목표(A/B) 차선으로 옮긴다."""
        node = self.node
        plan = self.plan

        marker = node.marker_type()
        if self.goal_lane is None and marker is not None:
            self.goal_lane = cfg['a_lane'] if marker == 'A' else cfg['b_lane']
            node.get_logger().info('[LEFT_A_RIGHT_B] %s 표지판 - 목표 %s 차선 (지금 %s)'
                                   % ('A(좌)' if marker == 'A' else 'B(우)',
                                      plan.name(self.goal_lane), plan.name(plan.lane)))
        if self.goal_lane is None and now - self.entered_at > cfg['approach_timeout']:
            node.get_logger().warn('[LEFT_A_RIGHT_B] 표지판 종류 불명 - 복귀')
            return True

        res = plan.step(now, goal=self.goal_lane, who='LEFT_A_RIGHT_B')
        if res is None:
            node.get_logger().warn('[LEFT_A_RIGHT_B] odom 끊김 - 카메라 차선 방식으로 전환')
            self.odom_mode = False
            plan.finish()
            return False
        if res.blocked:
            return False                        # 길이 없음 - 정지한 채로 다시 계산

        arrived = (self.goal_lane is not None and plan.lane == self.goal_lane
                   and not plan.changing)
        if arrived and plan.free_ahead(plan.lane, res.s) > cfg['odom_hold_distance']:
            if self.reached_s is None:
                self.reached_s = res.s
                node.get_logger().info('[LEFT_A_RIGHT_B] %s 차선 도착 (d=%.2fm)'
                                       % (plan.name(plan.lane), res.d))
            if res.s - self.reached_s >= cfg['odom_hold_distance'] and not plan.obstacles_ahead(res.s):
                node.get_logger().info('[LEFT_A_RIGHT_B] %s 경로 진입 완료 - 복귀'
                                       % ('A(좌)' if self.goal_lane == cfg['a_lane'] else 'B(우)'))
                return True
        else:
            self.reached_s = None
        return False
