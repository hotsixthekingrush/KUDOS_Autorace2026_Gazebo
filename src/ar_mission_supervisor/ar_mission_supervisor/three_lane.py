"""3차선 구간 공용 상태·주행 - AVOID_STATIC 과 LEFT_A_RIGHT_B 가 같이 쓴다.

차선 번호 (진행방향 기준): 0 = 왼쪽, 1 = 가운데, 2 = 오른쪽.
  2026 트랙에서는 3차선 끝의 A/B 섬이 가운데 차선을 막고 있어서
  A(좌) 경로는 0번, B(우) 경로는 2번 차선으로 이어진다.
좌표는 AVOID_STATIC 진입 순간의 차선 방향 기준(LaneFrame). 가운데 차선 중앙이 d = 0,
왼쪽 차선 중앙이 d = +lane_width, 오른쪽이 d = -lane_width.

차선 번호는 변수로 들고 다니되(시작 = 가운데), 차선 변경을 '끝냈을 때만' 바꾼다.
odom 으로 계산한 실제 위치(d)와 계속 어긋나면 odom 쪽을 따른다.

step(now, goal) 한 번 = 한 제어 주기:
  - 라이다 장애물 지도 갱신 (LIMO 는 차선 방향으로 놓인 0.322 x 0.22 m 박스 + 여유 clearance 로 본다)
    (11/04) 위치는 본 것의 평균(map_average), 이미 확실한 LIMO 와 겹치는 새 검출은 버림(ghost_*).
  - 후보 차선 3곳(차선 가운데)마다, 그 차선으로 가는 주행을 실제 조향 한계(max_steer, wheel_base)로
    앞 plan_horizon 만큼 미리 굴려 보고(rollout) 장애물과 닿는지 검사한다.
    (11/04) 보통(lookahead)으로 굴려서 막히는 차선은 급하게 꺾는 것(lookahead_short)도 굴려 보고
    더 멀리 가는 쪽으로 따라간다. 차 모양은 몸통 사각형 + (Gazebo 만) 바퀴를 감싼 사각형.
  - 지금 목표 차선이 끝까지(plan_horizon) 안 닿으면 유지한다 (goal 차선이 끝까지 비면 goal 로).
    아니면 '가장 멀리 갈 수 있는' 차선을 고른다 (지금 목표 +0.15 m, goal +0.3 m 가산).
  - 차선 사이(선 위)로 가는 경로는 쓰지 않는다. 고른 차선도 commit_distance 보다 짧게 닿으면
    감속하고, stop_distance 안에서 닿게 되면 선다.
  - (11/04 사용자 요청) 선 채로 backup_after 초가 지나면, 뒤가 비어 있을 때 backup_speed 로 backup_distance 만큼
    후진하고(차 방향을 차선 방향으로 펴면서) 다시 계산한다. 한 미션에 backup_max 번까지. LIMO 는 안 움직여서
    서서 기다리기만 하면 계속 막혀 있고, 규정상 진행 불가는 미션 처음부터 재출발이라서.
"""

import math

from ar_mission_supervisor.controllers import clamp, steering_to_omega
from ar_mission_supervisor.geometry import find_largest_gap
from ar_mission_supervisor.lane_frame import LaneFrame, ObstacleMap, pursue


class StepResult(object):
    def __init__(self, pose, s_, d, psi, blocked=False):
        self.pose, self.s, self.d, self.psi = pose, s_, d, psi
        self.blocked = blocked


class ThreeLanePlanner(object):

    LANES = (0, 1, 2)
    NAMES = ('왼쪽', '가운데', '오른쪽')

    def __init__(self, node):
        self.node = node
        self.frame = LaneFrame()
        self.map = ObstacleMap()
        self.active = False
        self.released_at = None
        self.lane = 1
        self.target = 1
        self.changing = False
        self.mismatch_since = None
        self.logged_blocked = False
        self.max_obstacle_s = None
        self.blocked_since = None
        self.backing = None         # 후진 중이면 시작 (x, y)
        self.backups = 0

    @property
    def cfg(self):
        return self.node.avoid_static

    # ------------------------------------------------------------------
    # 수명
    # ------------------------------------------------------------------
    def begin(self, lane):
        pose = self.node.odom_pose()
        if pose is None:
            self.active = False
            return False
        self.frame.begin(pose, self.node.lane_error())
        cfg = self.cfg
        ghost = None
        if cfg['ghost_min_hits'] > 0:
            ghost = (cfg['limo_length'] + cfg['ghost_margin'], cfg['limo_width'] + cfg['ghost_margin'],
                     cfg['ghost_min_hits'])
        self.map = ObstacleMap(merge=cfg['merge_distance'], average=cfg['map_average'], ghost=ghost)
        self.lane = self.target = int(lane)
        self.changing = False
        self.mismatch_since = None
        self.logged_blocked = False
        self.max_obstacle_s = None
        self.blocked_since = None
        self.backing = None
        self.backups = 0
        self.active = True
        self.released_at = None
        return True

    def release(self, now):
        """AVOID_STATIC 이 끝남. 잠깐 동안은 LEFT_A_RIGHT_B 가 이어받을 수 있다."""
        self.released_at = now

    def resume(self):
        self.released_at = None

    def fresh(self, now):
        if not self.active:
            return False
        if self.released_at is None:
            return True
        return (now - self.released_at) <= self.cfg['context_timeout']

    def finish(self):
        self.active = False

    # ------------------------------------------------------------------
    # 차선 기하
    # ------------------------------------------------------------------
    def width(self):
        return self.cfg['lane_width']

    def center(self, lane):
        return (1 - lane) * self.width()

    def lane_of(self, d):
        return int(max(0, min(2, round(1.0 - d / self.width()))))

    def name(self, lane):
        return self.NAMES[lane]

    @staticmethod
    def dist_str(x):
        return 'inf' if math.isinf(x) else '%.2fm' % x

    # ------------------------------------------------------------------
    # 장애물 지도
    # ------------------------------------------------------------------
    def update(self, now):
        """장애물 지도 갱신. 반환: (pose, s, d, psi) 또는 None(odom 없음)."""
        pose = self.node.odom_pose()
        if pose is None:
            return None
        s_, d, psi = self.frame.car(pose)
        cfg = self.cfg
        half = 1.5 * self.width() + 0.15

        def accept(so, do):
            return abs(do) <= half and so > s_ - 1.0

        self.map.update(self.frame, pose, self.node.obstacles(), now, accept,
                        cfg['min_obstacle_radius'], cfg['max_obstacle_radius'])
        self.map.prune(s_ - 1.5)
        items = self.map.confirmed(cfg['confirm_hits'])
        if items:
            last = max(it['s'] for it in items)
            self.max_obstacle_s = last if self.max_obstacle_s is None else max(self.max_obstacle_s, last)
        return pose, s_, d, psi

    def free_ahead(self, lane, s_car, back=0.0):
        """lane 에서 차(base_link) 앞쪽 가장 가까운 장애물 가장자리까지 거리 [m]. 없으면 inf."""
        best = float('inf')
        c = self.center(lane)
        for it in self.map.confirmed(self.cfg['confirm_hits']):
            if abs(it['d'] - c) > 0.5 * self.width():
                continue
            if it['s'] + it['r'] - s_car < -back:
                continue
            best = min(best, it['s'] - it['r'] - s_car)
        return best

    def obstacles_ahead(self, s_car):
        return [it for it in self.map.confirmed(self.cfg['confirm_hits'])
                if it['s'] + it['r'] > s_car]

    # ------------------------------------------------------------------
    # 미리 굴려보기 (rollout)
    # ------------------------------------------------------------------
    def _boxes(self, s_car):
        cfg = self.cfg
        hl = 0.5 * cfg['limo_length'] + cfg['clearance']
        hw = 0.5 * cfg['limo_width'] + cfg['clearance']
        return [(it['s'], it['d'], hl, hw)
                for it in self.map.confirmed(cfg['confirm_hits'])
                if it['s'] + hl > s_car - 0.5]

    def _shapes(self):
        """검사할 차 모양 [(반 길이, 반 폭), ...]. 실차는 몸통 하나.
        Gazebo 는 몸통 상자(앞뒤 0.19 x 좌우 0.31) + 바퀴 네 개를 감싼 사각형(0.33 x 0.258) 두 개 (11/04)."""
        cfg = self.cfg
        shapes = [(0.5 * cfg['car_length'], 0.5 * cfg['car_width'])]
        if cfg['wheel_box_length'] > 0.0 and cfg['wheel_box_width'] > 0.0:
            shapes.append((0.5 * cfg['wheel_box_length'], 0.5 * cfg['wheel_box_width']))
        return shapes

    @classmethod
    def _car_hits(cls, s_, d, psi, boxes, shapes):
        for half_l, half_w in shapes:
            if cls._rect_hits(s_, d, psi, boxes, half_l, half_w):
                return True
        return False

    @staticmethod
    def _rect_hits(s_, d, psi, boxes, half_l, half_w):
        c, sn = math.cos(psi), math.sin(psi)
        car = [(s_ + a * c - b * sn, d + a * sn + b * c)
               for a, b in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w))]
        cs = [p[0] for p in car]
        cd = [p[1] for p in car]
        for bs, bd, hl, hw in boxes:
            # 1) 박스 축으로 분리되는지
            if max(cs) < bs - hl or min(cs) > bs + hl or max(cd) < bd - hw or min(cd) > bd + hw:
                continue
            # 2) 차 축으로 분리되는지
            box = ((bs - hl, bd - hw), (bs + hl, bd - hw), (bs + hl, bd + hw), (bs - hl, bd + hw))
            separated = False
            for ax, ay in ((c, sn), (-sn, c)):
                pc = [x * ax + y * ay for x, y in car]
                pb = [x * ax + y * ay for x, y in box]
                if max(pc) < min(pb) or max(pb) < min(pc):
                    separated = True
                    break
            if not separated:
                return True
        return False

    def rollout(self, s0, d0, psi0, d_target, boxes, horizon, la=None):
        """d_target 차선으로 가는 주행을 horizon m 앞까지 굴려 보고, 닿기 전까지 간 거리 [m].
        la: Pure Pursuit 따라가는 거리 (None 이면 lookahead). 짧을수록 급하게 꺾는다."""
        node = self.node
        cfg = self.cfg
        wb, ms = node.wheel_base, node.max_steer
        if la is None:
            la = cfg['lookahead']
        shapes = self._shapes()
        ds = 0.04
        s_, d, psi = s0, d0, psi0
        if self._car_hits(s_, d, psi, boxes, shapes):
            return 0.0
        traveled = 0.0
        while traveled < horizon:
            c, sn = math.cos(psi), math.sin(psi)
            dx, dy = la, d_target - d
            bx, by = c * dx + sn * dy, -sn * dx + c * dy
            l2 = bx * bx + by * by
            kappa = 0.0 if l2 < 1e-9 else 2.0 * by / l2
            steer = clamp(math.atan(kappa * wb), -ms, ms)
            psi += ds * math.tan(steer) / wb
            s_ += ds * math.cos(psi)
            d += ds * math.sin(psi)
            traveled += ds
            if self._car_hits(s_, d, psi, boxes, shapes):
                return traveled
        return horizon

    def _all_boxes(self):
        cfg = self.cfg
        hl = 0.5 * cfg['limo_length'] + cfg['clearance']
        hw = 0.5 * cfg['limo_width'] + cfg['clearance']
        return [(it['s'], it['d'], hl, hw) for it in self.map.confirmed(cfg['confirm_hits'])]

    def back_steer(self, psi):
        """후진하면서 차 방향을 차선 방향(psi = 0)으로 펴는 조향각. 후진(v < 0)이면 psi 가 줄어든다."""
        return clamp(self.cfg['backup_k_heading'] * psi, -self.node.max_steer, self.node.max_steer)

    def rollout_back(self, s0, d0, psi0, boxes, dist):
        """후진하며 back_steer 로 dist 만큼 굴려 보고, 닿기 전까지 간 거리 [m]."""
        wb = self.node.wheel_base
        shapes = self._shapes()
        ds = 0.02
        s_, d, psi = s0, d0, psi0
        traveled = 0.0
        while traveled < dist:
            steer = self.back_steer(psi)
            psi -= ds * math.tan(steer) / wb
            s_ -= ds * math.cos(psi)
            d -= ds * math.sin(psi)
            traveled += ds
            if self._car_hits(s_, d, psi, boxes, shapes):
                return traveled - ds
        return dist

    def _back_step(self, pose, s_, d, psi, now, who):
        """후진 중 한 주기. 다 했으면 False (다시 계산으로)."""
        cfg = self.cfg
        node = self.node
        x0, y0 = self.backing
        done = math.hypot(pose[0] - x0, pose[1] - y0)
        if done >= cfg['backup_distance'] or self.rollout_back(s_, d, psi, self._all_boxes(), 0.04) < 0.04:
            node.get_logger().info('[%s] 후진 %.2fm 끝 - 다시 계산' % (who, done))
            self.backing = None
            self.blocked_since = None
            self.logged_blocked = False
            return False
        v = -cfg['backup_speed']
        node.publish_cmd(v, steering_to_omega(v, self.back_steer(psi), node.wheel_base))
        node.dbg('static/mode', 'backup')
        return True

    # ------------------------------------------------------------------
    # 한 제어 주기
    # ------------------------------------------------------------------
    def step(self, now, goal=None, who='3차선'):
        """반환: StepResult 또는 None(odom 없음). 길이 없어 서 있으면 result.blocked."""
        node = self.node
        cfg = self.cfg
        log = node.get_logger()
        car = self.update(now)
        if car is None:
            return None
        pose, s_, d, psi = car

        if self.changing and self.settled(d, psi):
            self.lane = self.target
            self.changing = False
            log.info('[%s] 차선 변경 완료 - %s 차선 (d=%.2fm)' % (who, self.name(self.lane), d))
        if not self.changing:
            self.check_lane(d, now, who)

        if self.backing is not None and self._back_step(pose, s_, d, psi, now, who):
            return StepResult(pose, s_, d, psi, blocked=True)

        horizon = cfg['plan_horizon']
        boxes = self._boxes(s_)
        # 차선마다 보통(lookahead)으로 굴려 보고, 막히면 급하게 꺾는 것(lookahead_short)도 굴려 봄 (11/04)
        las = [cfg['lookahead']]
        if cfg['lookahead_short'] > 0.0:
            las.append(cfg['lookahead_short'])
        free, best_la = {}, {}
        for lane in self.LANES:
            free[lane], best_la[lane] = -1.0, las[0]
            for la in las:
                f = self.rollout(s_, d, psi, self.center(lane), boxes, horizon, la)
                if f > free[lane] + 1e-6:
                    free[lane], best_la[lane] = f, la
                if f >= horizon - 1e-6:
                    break
        clear = [lane for lane in self.LANES if free[lane] >= horizon - 1e-6]
        node.dbg('static/free_target', free[self.target])

        def score(ln):
            sc = free[ln] - 0.02 * abs(ln - self.lane)
            if ln == self.target:
                sc += 0.15                      # 자꾸 바꾸지 않게
            if goal is not None and ln == goal:
                sc += 0.3
            return sc

        old = self.target
        if goal is not None and goal in clear:
            new = goal
        elif self.target in clear:
            new = self.target
        else:
            new = max(self.LANES, key=score)

        if new != old:
            log.info('[%s] 목표 차선 %s -> %s (지금 %s / 앞 %.1fm 굴려본 결과 %s)'
                     % (who, self.name(old), self.name(new), self.name(self.lane), horizon,
                        ' '.join('%s=%.2f' % (self.name(ln), free[ln]) for ln in self.LANES)))
            self.target = new
        if self.target != self.lane:
            self.changing = True

        v = cfg['speed']
        if free[self.target] < cfg['commit_distance']:
            if not self.logged_blocked:
                log.warn('[%s] 갈 수 있는 거리가 짧음 (최대 %.2fm) - 감속, %.2fm 안이면 정지'
                         % (who, free[self.target], cfg['stop_distance']))
                self.logged_blocked = True
            if free[self.target] <= cfg['stop_distance']:
                node.publish_stop()
                node.dbg('static/mode', 'stop')
                if self.blocked_since is None:
                    self.blocked_since = now
                elif (now - self.blocked_since >= cfg['backup_after'] and self.backups < cfg['backup_max']):
                    room = self.rollout_back(s_, d, psi, self._all_boxes(), cfg['backup_distance'])
                    if room >= 0.05:
                        self.backups += 1
                        self.backing = (pose[0], pose[1])
                        log.warn('[%s] %.1f초 막힘 - %.2fm 후진 후 다시 계산 (%d/%d번째, 차선별 갈 거리 %s)'
                                 % (who, now - self.blocked_since, min(room, cfg['backup_distance']),
                                    self.backups, cfg['backup_max'],
                                    ' '.join('%s=%.2f' % (self.name(ln), free[ln]) for ln in self.LANES)))
                return StepResult(pose, s_, d, psi, blocked=True)
            v = cfg['slow_speed']
            self.blocked_since = None
        else:
            self.logged_blocked = False
            self.blocked_since = None

        node.dbg('static/lane', self.lane)
        node.dbg('static/target', self.target)
        node.dbg('static/d', d)
        node.dbg('static/s', s_)
        node.dbg('static/lookahead', best_la[self.target])
        node.publish_cmd(v, self.steer(pose, s_, self.center(self.target), v, best_la[self.target]))
        return StepResult(pose, s_, d, psi)

    def settled(self, d, psi):
        return (abs(d - self.center(self.target)) <= self.cfg['settle_tolerance']
                and abs(psi) <= self.cfg['settle_heading'])

    def check_lane(self, d, now, who='3차선'):
        """차선 변경 중이 아닐 때 odom 위치와 차선 번호가 계속 다르면 odom 을 따른다."""
        actual = self.lane_of(d)
        if actual == self.lane:
            self.mismatch_since = None
            return
        if self.mismatch_since is None:
            self.mismatch_since = now
        elif now - self.mismatch_since >= 0.5:
            self.node.get_logger().warn(
                '[%s] 차선 번호 %d 와 odom 위치(%s, d=%.2fm)가 달라 odom 을 따름'
                % (who, self.lane, self.name(actual), d))
            self.lane = self.target = actual
            self.mismatch_since = None

    def steer(self, pose, s_car, d_target, v, la=None):
        omega, _ = pursue(self.node, self.frame, pose,
                          s_car + (self.cfg['lookahead'] if la is None else la), d_target, v)
        return omega

    def gap_step(self):
        """odom 이 없을 때만 쓰는 비상 주행: 전방 스캔에서 가장 넓은 빈 공간 쪽으로 저속."""
        node = self.node
        cfg = self.cfg
        v = cfg['gap_speed']
        points = node.forward_scan_points(cfg['gap_range'])
        gap = find_largest_gap(points, cfg['robot_width'], cfg['gap_range'])
        if gap is None:
            node.publish_cmd(v, 0.0)
            return
        steer = clamp(cfg['gap_gain'] * gap[0], -node.max_steer, node.max_steer)
        node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))
