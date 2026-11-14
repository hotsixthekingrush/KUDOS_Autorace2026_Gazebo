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

카메라 방식에서 차선 노드 기준 차선이 바뀔 때 (11/13):
  차선 노드는 '차가 지금 들어가 있는 차선' 가운데를 준다 (팀원과 맞춘 약속). 그래서 옆으로 옮기다 점선을 넘으면
  lateral 이 차선 폭만큼 튄다 (왼쪽 차선으로 바뀌면 +폭). 그대로 두면 offset(처음 차선 기준)이 한 번 더 더해져
  한 차선 더 가려고 한다. lane_jump: 옮기는 동안(SHIFT, SETTLE) 연속 차선 값이 차선 폭의 반 넘게 튀면
  그 차선 수(lane_shift)를 세서 offset 에서 lane_shift x 차선 폭을 뺀다. 차선 노드가 처음 차선을 계속 줘도 그대로 동작.
  lane_shift 는 0(처음 차선) ~ 목표 차선 하나 사이만 (반대 방향으로 튀는 건 차선 노드 오류로 보고 무시).
  odom 모드(3차선 이어받음)는 카메라 차선을 안 쓰니 상관없음.
  (11/14) settle_sec 이 지나도 차선 노드 기준이 새 차선으로 안 바뀌었으면 최대 jump_wait_sec 더 옮기며 기다린다
  (점선을 못 넘은 채 끝나면 LANE_FOLLOWING 이 처음 차선으로 돌아가서).
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
        self.lane_shift = 0         # (11/13) 차선 노드 기준 차선이 옆으로 바뀐 수 (+1 = 왼쪽 차선 기준)
        self._cam_prev = None       # 직전 차선 값 (lateral, stamp)
        self.waiting_jump = False   # (11/14) settle 뒤 차선 노드 기준 바뀌기 기다리는 중

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
        self.lane_shift = 0
        self._cam_prev = None
        self.waiting_jump = False
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

    def _track_lane_jump(self, cfg):
        """(11/13) 연속 차선 값의 lateral 이 차선 폭의 반 넘게 튀면 차선 노드 기준 차선이 바뀐 것으로 센다."""
        if not cfg['lane_jump']:
            return
        lane = self.node.lane_error()
        if lane is None:
            return
        lat, stamp = float(lane[0]), lane[4]
        prev = self._cam_prev
        if prev is not None and stamp == prev[1]:
            return                              # 같은 차선 값
        self._cam_prev = (lat, stamp)
        if prev is None or stamp - prev[1] > cfg['jump_max_gap']:
            return
        n = int(round((lat - prev[0]) / cfg['lane_width']))
        if not n:
            return
        # 옮기는 차선은 하나뿐 -> 0 (처음 차선) 과 direction (목표 차선) 사이만 허용. 반대로 튀거나 두 차선 튀면 무시
        lo, hi = min(0, int(self.direction)), max(0, int(self.direction))
        new = self.lane_shift + n
        log = self.node.get_logger()
        if not lo <= new <= hi:
            log.warn('[LEFT_A_RIGHT_B] 차선 값이 %+.2f -> %+.2fm 로 튐 - 옮기는 방향과 안 맞아 무시'
                     % (prev[0], lat))
            return
        self.lane_shift = new
        log.info('[LEFT_A_RIGHT_B] 차선 노드 기준이 %s 차선으로 바뀜 (lateral %+.2f -> %+.2fm) - 목표 이동량에서 %+.2fm 뺌'
                 % ('왼쪽' if n > 0 else '오른쪽', prev[0], lat, self.lane_shift * cfg['lane_width']))

    def _offset(self, cfg, progress):
        """처음 차선 기준 이동량 - 차선 노드 기준이 바뀐 만큼."""
        return self.direction * cfg['lane_width'] * progress - self.lane_shift * cfg['lane_width']

    def _shift(self, now, cfg):
        """오프셋을 점진적으로 키워 목표 차선으로 이동."""
        elapsed = now - self.shift_started
        progress = clamp(elapsed / cfg['shift_duration'], 0.0, 1.0)
        self._track_lane_jump(cfg)
        offset = self._offset(cfg, progress)

        self.driver.step(cfg['shift_speed'], offset)

        if progress >= 1.0:
            self.phase = self.SETTLE
            self.settle_started = now
            self.node.get_logger().info('[LEFT_A_RIGHT_B] 이동 완료 - 안정화')

    def _settle(self, now, cfg):
        """새 차선에서 잠시 유지한 뒤 종료.
        (11/14) lane_jump 를 켜면 settle_sec 이 지나도 차선 노드 기준이 새 차선으로 바뀔 때까지 (최대 jump_wait_sec 더)
        기다린다. 점선을 못 넘은 채 끝나면 LANE_FOLLOWING 이 처음 차선으로 돌아가서 (기본값으로 끝날 때 옆 0.28 m,
        점선 0.26 m 를 겨우 넘음)."""
        self._track_lane_jump(cfg)
        offset = self._offset(cfg, 1.0)
        self.driver.step(cfg['shift_speed'], offset)
        held = now - self.settle_started
        if held < cfg['settle_sec']:
            return False
        if not cfg['lane_jump'] or self.lane_shift == int(self.direction):
            return True
        log = self.node.get_logger()
        if held < cfg['settle_sec'] + cfg['jump_wait_sec']:
            if not self.waiting_jump:
                self.waiting_jump = True
                log.info('[LEFT_A_RIGHT_B] 차선 노드가 아직 처음 차선 기준 - 새 차선으로 바뀔 때까지 최대 %.1f초 더 옮김'
                         % cfg['jump_wait_sec'])
            return False
        log.warn('[LEFT_A_RIGHT_B] %.1f초 더 기다려도 차선 노드 기준이 안 바뀜 - 그냥 끝냄 (처음 차선 기준을 계속 주는 노드일 수 있음)'
                 % cfg['jump_wait_sec'])
        return True

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
