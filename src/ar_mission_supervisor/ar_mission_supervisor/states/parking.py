"""미션 4 - T자 주차 (2026: 진행방향 오른쪽 칸, 양옆 LIMO, 벽 없음, 전면 진입 P1).

2026 트랙: 주차 칸은 진행방향 오른쪽, 칸 0.5 m(진행방향) x 0.6 m(깊이), 2x3 배치.
"지정 위치" = 양옆에 LIMO 가 서 있는 칸 (미션 상세 문서). 벽(ㄷ자)은 없고 선만 있다.

단계
  SEARCH : 카메라 차선 추종으로 천천히 가면서 라이다 장애물을 기준 좌표(odom)에 쌓는다.
           오른쪽에서 칸 하나 건너(간격 2 x bay_pitch) 나란히 선 LIMO 2대를 찾으면
           그 가운데가 목표 칸. 뒷축이 (칸 중심 - turn_radius) 에 오면 TURN.
           (지나쳤으면 그 지점까지 천천히 후진한 뒤 TURN)
  TURN   : 반지름 turn_radius 원호를 따라 오른쪽으로 90도 (원호 오차를 피드백으로 보정).
           turn_radius 는 실제 최소 회전반경(실차 약 0.46 m)보다 조금 크게 잡아 보정 여유를 둔다.
  ALIGN  : 칸 깊이 가운데(park_depth)까지 전진/후진하며 방향(-90도)을 맞추고 정지.
  PARKED : 정지 유지 (마지막 미션이므로 기본 주행으로 돌아가지 않음).
실패(빈칸 못 찾음, 시간 초과) 시 정지 후 외부 리셋을 기다린다.
규정: 차체 네 모서리가 모두 칸 안이면 15점, 1곳 걸치면 7점, 2곳 이상이면 0점.
"""

import math

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.controllers import clamp, steering_to_omega
from ar_mission_supervisor.geometry import normalize_angle
from ar_mission_supervisor.lane_driver import LaneDriver
from ar_mission_supervisor.lane_frame import LaneFrame, ObstacleMap


class Parking(SafeState):

    OUTCOMES = ['done']

    SEARCH, TURN, ALIGN, PARKED, FAILED = range(5)
    PHASE_NAMES = ('SEARCH', 'TURN', 'ALIGN', 'PARKED', 'FAILED')

    def __init__(self, node):
        SafeState.__init__(self, node, 'PARKING', self.OUTCOMES)
        self.driver = LaneDriver(node)
        self.frame = LaneFrame()
        self.map = ObstacleMap()
        self.phase = self.SEARCH
        self.phase_started = 0.0
        self.gap_s = None
        self.gap_hits = 0
        self.center = None

    def on_enter(self):
        node = self.node
        cfg = node.parking
        self.driver.reset()
        self.map = ObstacleMap(merge=cfg['merge_distance'])
        self.gap_s = None
        self.gap_hits = 0
        self.center = None
        now = node.clock_sec()
        pose = node.odom_pose()
        if pose is None:
            self.phase, self.phase_started = self.SEARCH, now
            self._fail('odom 없음')
            return
        self.frame.begin(pose, node.lane_error())
        self._set_phase(self.SEARCH, now, '진입 - 오른쪽 빈칸(양옆 LIMO) 찾는 중')

    def run(self):
        node = self.node
        cfg = node.parking
        while node.ok():
            if self.reset_requested():
                return 'aborted'

            now = node.clock_sec()
            node.dbg('parking/phase', self.PHASE_NAMES[self.phase])
            pose = node.odom_pose()
            if self.phase in (self.PARKED, self.FAILED):
                node.publish_stop()
            elif pose is None:
                node.publish_stop()
                self._fail('odom 끊김')
            elif self.phase == self.SEARCH:
                self._search(now, cfg, pose)
            elif self.phase == self.TURN:
                self._turn(now, cfg, pose)
            else:
                self._align(now, cfg, pose)

            self.sleep()
        return 'aborted'

    # ------------------------------------------------------------------
    # SEARCH
    # ------------------------------------------------------------------
    def _search(self, now, cfg, pose):
        node = self.node
        s_, d, psi = self.frame.car(pose)
        if now - self.phase_started > cfg['search_timeout'] or s_ > cfg['search_distance']:
            node.publish_stop()
            self._fail('빈칸을 찾지 못함 (%.1fm 주행)' % s_)
            return

        def accept(so, do):
            return cfg['bay_d_min'] <= do <= cfg['bay_d_max'] and so > s_ - 1.5

        self.map.update(self.frame, pose, node.obstacles(), now, accept,
                        cfg['min_obstacle_radius'], cfg['max_obstacle_radius'])

        gap = self._find_gap(s_, cfg)
        if gap is not None:
            if self.gap_s is None or abs(gap - self.gap_s) > cfg['gap_match']:
                self.gap_s, self.gap_hits = gap, 1
            else:
                self.gap_s += 0.3 * (gap - self.gap_s)
                self.gap_hits += 1
        node.dbg('parking/gap_s', self.gap_s)

        wb = node.wheel_base
        rear_s = s_ - 0.5 * wb * math.cos(psi)
        if self.gap_s is not None and self.gap_hits >= cfg['gap_confirm']:
            start = self.gap_s - cfg['turn_radius']
            node.dbg('parking/to_turn', start - rear_s)
            if rear_s > start + cfg['overshoot_tolerance']:
                # 지나쳤음 -> 회전 시작점까지 천천히 후진 (헤딩 유지)
                v = -cfg['final_speed']
                steer = clamp(-cfg['k_heading'] * normalize_angle(0.0 - psi), -node.max_steer, node.max_steer)
                node.publish_cmd(v, steering_to_omega(v, steer, wb))
                return
            if rear_s >= start - 0.005:
                rear_d = d - 0.5 * wb * math.sin(psi)
                self.center = (rear_s, rear_d - cfg['turn_radius'])
                node.publish_stop()
                self._set_phase(self.TURN, now,
                                '빈칸 확인 (칸 중심 s=%.2fm, 차 s=%.2fm) - 오른쪽 원호 진입 시작'
                                % (self.gap_s, s_))
                return
            speed = cfg['final_speed'] if start - rear_s < cfg['slow_distance'] else cfg['search_speed']
        else:
            speed = cfg['search_speed']
        self.driver.step(speed)

    def _find_gap(self, s_car, cfg):
        """오른쪽 LIMO 들 중 칸 하나를 사이에 두고(2 x bay_pitch) 나란히 선 쌍의 가운데 s."""
        items = sorted(self.map.confirmed(cfg['confirm_hits']), key=lambda it: it['s'])
        want = 2.0 * cfg['bay_pitch']
        for a, b in zip(items, items[1:]):
            if abs((b['s'] - a['s']) - want) > cfg['pitch_tolerance']:
                continue
            if abs(b['d'] - a['d']) > cfg['pair_d_tolerance']:
                continue
            mid = 0.5 * (a['s'] + b['s'])
            if mid < s_car - cfg['turn_radius'] - 0.5:
                continue                        # 이미 한참 지나간 칸
            return mid
        return None

    # ------------------------------------------------------------------
    # TURN : 원호 추종 (오른쪽 90도)
    # ------------------------------------------------------------------
    def _turn(self, now, cfg, pose):
        node = self.node
        if now - self.phase_started > cfg['turn_timeout']:
            node.publish_stop()
            self._fail('회전 시간 초과')
            return
        s_, d, psi = self.frame.car(pose)
        wb = node.wheel_base
        rs = s_ - 0.5 * wb * math.cos(psi)
        rd = d - 0.5 * wb * math.sin(psi)
        radial = math.hypot(rs - self.center[0], rd - self.center[1]) - cfg['turn_radius']
        node.dbg('parking/radial_err', radial)

        if psi <= -0.5 * math.pi + cfg['heading_tolerance']:
            node.publish_stop()
            self._set_phase(self.ALIGN, now, '회전 완료 (%.0f도) - 칸 깊이 맞추기' % math.degrees(psi))
            return

        # 원 바깥(radial > 0)이면 더 꺾고, 안쪽이면 덜 꺾는다. 오른쪽 = 음수 조향
        steer = -math.atan(wb / cfg['turn_radius']) - cfg['k_radial'] * radial
        steer = clamp(steer, -node.max_steer, node.max_steer)
        v = cfg['turn_speed']
        node.publish_cmd(v, steering_to_omega(v, steer, wb))

    # ------------------------------------------------------------------
    # ALIGN : 칸 깊이 가운데로 전진/후진
    # ------------------------------------------------------------------
    def _align(self, now, cfg, pose):
        node = self.node
        s_, d, psi = self.frame.car(pose)
        ts, td = self.gap_s, -cfg['park_depth']
        c, sn = math.cos(psi), math.sin(psi)
        e_long = (ts - s_) * c + (td - d) * sn          # 차 앞 방향으로 남은 거리
        e_lat = -(ts - s_) * sn + (td - d) * c          # 차 왼쪽 방향으로 벗어난 거리
        node.dbg('parking/e_long', e_long)
        node.dbg('parking/e_lat', e_lat)

        timeout = now - self.phase_started > cfg['align_timeout']
        if abs(e_long) <= cfg['stop_tolerance'] or timeout:
            node.publish_stop()
            msg = '주차 완료' if not timeout else '정렬 시간 초과 - 현 위치에서 주차 종료'
            self._set_phase(self.PARKED, now,
                            '%s (칸 중심 기준 앞뒤 %+.3fm, 좌우 %+.3fm, 방향 %+.1f도)'
                            % (msg, e_long, e_lat, math.degrees(normalize_angle(psi + 0.5 * math.pi))))
            return

        v = clamp(cfg['k_long'] * e_long, -cfg['final_speed'], cfg['final_speed'])
        if abs(v) < cfg['min_speed']:
            v = math.copysign(cfg['min_speed'], e_long)
        # 전진이면 칸 중심선 쪽으로 머리를 살짝 틀고, 후진이면 반대로 튼다
        lat_gain = cfg['k_lateral'] if v > 0 else -cfg['k_lateral']
        psi_des = -0.5 * math.pi + clamp(math.atan(lat_gain * e_lat), -0.3, 0.3)
        omega_des = cfg['k_heading'] * normalize_angle(psi_des - psi)
        steer = clamp(math.atan(omega_des * node.wheel_base / v), -node.max_steer, node.max_steer)
        node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))

    # ------------------------------------------------------------------
    def _set_phase(self, phase, now, msg):
        self.phase = phase
        self.phase_started = now
        self.node.get_logger().info('[PARKING] %s' % msg)

    def _fail(self, reason):
        self.phase = self.FAILED
        self.node.publish_stop()
        self.node.get_logger().warn('[PARKING] 실패: %s - 정지 후 리셋 대기' % reason)
