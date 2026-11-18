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
        self._ab_why = ''

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
        self._ab_why = ''
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
            limit = cfg['max_duration']
            if self.odom_mode and self.plan.maneuvers > 0:
                limit = max(limit, cfg['odom_max_duration'])     # (11/16) 앞뒤 움직임은 30~60초 걸림
            if now - self.entered_at > limit:
                node.get_logger().warn('[LEFT_A_RIGHT_B] 제한시간 초과 (%.0f초) - 복귀' % limit)
                if self.odom_mode:
                    self.plan.release(now)      # (11/16) 곧 AVOID_STATIC 에 다시 들어오면 기준 좌표 이어 씀
                else:
                    self.plan.finish()
                return 'done'

            if self.odom_mode:
                if self._odom_step(now, cfg):
                    self.plan.finish()
                    if cfg['odom_end_clear'] > 0.0 and node.lockout.pop('AVOID_STATIC', None) is not None:
                        # (11/19) 목표 차선 1 m 앞만 비면 끝내니, 그 너머 LIMO 를 AVOID_STATIC 이 바로 다시 잡을 수 있게
                        #   (AVOID_STATIC -> 여기로 넘어온 뒤 8초(lockout_sec) 안이면 다시 못 들어가 차선 주행으로 박을 수 있음)
                        node.get_logger().info('[LEFT_A_RIGHT_B] AVOID_STATIC 재진입 막음 풀음 (앞에 LIMO 있으면 바로 다시 피함)')
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
                    self._ab_done()
                    return 'done'

            self.sleep()

        return 'aborted'

    # ------------------------------------------------------------------
    def _approach(self, now, cfg):
        """표지판까지 접근. 트리거 거리에 도달하면 SHIFT 로 전환.
        True 를 반환하면 미션을 포기하고 복귀한다."""
        node = self.node
        marker = self._ab_marker(now, fallback=self.direction == 0.0)
        dist = node.marker_distance()

        if marker is not None:
            new_dir = 1.0 if marker == 'A' else -1.0     # A = 좌(+), B = 우(-)
            if new_dir != self.direction:
                node.get_logger().info('[LEFT_A_RIGHT_B] %s 표지판%s' % ('A(좌)' if marker == 'A' else 'B(우)', self._ab_why))
            self.direction = new_dir

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

        v = cfg['approach_speed']
        if node.ab_vote['use'] and self.direction == 0.0:
            v = min(v, node.ab_vote['undecided_speed'])      # (11/19) 아직 못 정함 - 천천히 더 봄
        self.driver.step(v)
        return False

    def _ab_done(self):
        """(11/19) 갈림길을 끝까지 했으면 다음 AVOID_STATIC 전까지 A/B 표지판을 다시 안 받음 (ab_vote.after_static).
        1118 comp_lap11 142~145초: 신호등 근처에서 A 오인식. 시간 초과·종류 불명으로 끝난 건 그대로 둠 (다시 들어올 수 있게)."""
        node = self.node
        if node.ab_vote['after_static'] and node.static_started:
            node.static_started = False
            node.get_logger().info('[LEFT_A_RIGHT_B] 갈림길 끝 - 다음 AVOID_STATIC 전까지 A/B 표지판 안 받음')

    def _ab_marker(self, now, fallback=True):
        """(11/19 사용자·YOLO 팀원) 'A' / 'B' / None. 다수결로 정해지면 그쪽, 못 정한 채 표지판이 fallback_distance 안이거나
        들어온 지 fallback_sec 지나면 여기 들어온 뒤(조금 전부터) 받은 표 전부의 다수. ab_vote.use false 면 예전(마지막 표지판)."""
        node = self.node
        ab = node.ab_vote
        self._ab_why = ''
        if not ab['use']:
            return node.marker_type()
        d = node.ab_decision(now)
        na, nb = node.ab_counts(now)
        if d is not None:
            self._ab_why = ', 최근 %.1f초 A %d / B %d' % (ab['window'], na, nb)
            return d
        dist = node.marker_distance()
        late = now - self.entered_at > ab['fallback_sec']
        near = dist is not None and dist <= ab['fallback_distance']
        if fallback and (late or near):
            since = self.entered_at - ab['window']
            f = node.ab_fallback(since, now)
            if f is not None:
                fa, fb = node.ab_counts(now, since=since)
                self._ab_why = ', 다수결 못 함 - %s 받은 표 A %d / B %d 로 정함' % (
                    '표지판 %.2fm' % dist if near else '%.0f초 지나' % ab['fallback_sec'], fa, fb)
                node.get_logger().warn('[LEFT_A_RIGHT_B] A/B 다수결 못 함 (최근 A %d / B %d) - 받은 표 전부 A %d / B %d 로 %s'
                                       % (na, nb, fa, fb, f))
            return f
        return None

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

        marker = self._ab_marker(now) if self.goal_lane is None else None
        if self.goal_lane is None and marker is not None:
            self.goal_lane = cfg['a_lane'] if marker == 'A' else cfg['b_lane']
            plan.ab_goal = self.goal_lane       # (11/16) 시간 초과 뒤 AVOID_STATIC 에 다시 들어가도 이 차선으로
            node.get_logger().info('[LEFT_A_RIGHT_B] %s 표지판 - 목표 %s 차선 (지금 %s%s)'
                                   % ('A(좌)' if marker == 'A' else 'B(우)',
                                      plan.name(self.goal_lane), plan.name(plan.lane), self._ab_why))
        ab = node.ab_vote
        plan.speed_cap = ab['undecided_speed'] if ab['use'] and self.goal_lane is None else None
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
            if res.s - self.reached_s >= cfg['odom_hold_distance'] and self._end_clear(plan, res.s, cfg):
                node.get_logger().info('[LEFT_A_RIGHT_B] %s 경로 진입 완료 - 복귀'
                                       % ('A(좌)' if self.goal_lane == cfg['a_lane'] else 'B(우)'))
                self._ab_done()
                return True
        else:
            self.reached_s = None
        return False

    def _end_clear(self, plan, s_car, cfg):
        """(11/19 사용자) 끝내도 되는지. odom_end_clear > 0: 목표 차선(가운데 +-(차선 반폭 + 차 반폭)) 안, 차 앞 odom_end_clear m 안에
        장애물이 없으면 됨. 0 = 예전: 3차선 지도에 차보다 앞인 장애물이 하나라도 있으면 (차선·거리 상관없이) 안 끝냄
        -> 1118 comp_lap18: 도착하고 코너 벽 등이 계속 '앞'에 잡혀 20초 제한까지 카메라 안 보고 직진."""
        ahead = cfg['odom_end_clear']
        if ahead <= 0.0:
            return not plan.obstacles_ahead(s_car)
        c = plan.center(plan.lane)
        side = 0.5 * plan.width() + 0.5 * plan.cfg['car_width']
        front = s_car + 0.5 * plan.cfg['car_length']
        for it in plan.map.confirmed(plan.cfg['confirm_hits']):
            if abs(it['d'] - c) > side + it['r']:
                continue
            if it['s'] + it['r'] < front - 0.05 or it['s'] - it['r'] > front + ahead:
                continue
            self.node.dbg('ab/end_blocked', it['s'] - s_car)
            return False
        return True
