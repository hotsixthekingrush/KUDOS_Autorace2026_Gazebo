"""3차선 구간 공용 상태·주행 - AVOID_STATIC 과 LEFT_A_RIGHT_B 가 같이 쓴다.

차선 번호 (진행방향 기준): 0 = 왼쪽, 1 = 가운데, 2 = 오른쪽.
  2026 트랙에서는 3차선 끝의 A/B 섬이 가운데 차선을 막고 있어서
  A(좌) 경로는 0번, B(우) 경로는 2번 차선으로 이어진다.
좌표는 AVOID_STATIC 진입 순간의 차선 방향 기준(LaneFrame). 가운데 차선 중앙이 d = 0,
왼쪽 차선 중앙이 d = +lane_width, 오른쪽이 d = -lane_width.
(11/12) 카메라 차선을 못 믿어서 차 방향을 기준으로 쓸 때는, 차 방향이 트랙 축(odom 기준 axis_ref_deg + 90도 단위)에서
  axis_snap_max_deg 안이면 그 축으로 맞춘다 (axis_snap). Gazebo pd_lane_run1~3: 두 번째 코너를 16~20도 덜 돈 채
  (카메라는 3차선 입구에서 옆 0, 방향 +20도로 틀림) 진입 -> 차 방향을 기준으로 잡아 바깥 노란 선 위까지 감.
  트랙이 직각으로만 되어 있고, odom yaw 0 이 출발 직선 방향이라는 가정 (출발선에 똑바로 놓고 odom 을 켤 것).

(11/12) 차 방향을 기준으로 쓸 때는 옆 위치(d = 0)도 차 위치라서, 차가 차선 가운데에서 비켜 있으면 세 차선이 다 같이
  밀린다 (pd_lane_run4: 11 cm). 가까이(lat_snap_range) 확실히 본 LIMO 가 lat_snap_min_items 대 이상 모이면,
  LIMO 가 차선 가운데(d = 0, +-lane_width)에 서 있다고 보고 옆 위치를 한 번 맞춘다 (lat_snap).

차선 번호는 변수로 들고 다니되(시작 = 가운데), 차선 변경을 '끝냈을 때만' 바꾼다.
odom 으로 계산한 실제 위치(d)와 계속 어긋나면 odom 쪽을 따른다.
(11/12, lane_from_odom) 차선 바꾸는 중에도 차선 번호는 odom 위치로 고친다 (pd_lane_run4: 가운데~오른쪽인데 계속 '왼쪽').
(11/12, lane_commit, 기본 끔) 켜면 목표 차선에 들어가기 전에는 목표를 안 바꾼다 (그 목표가 곧 막히고 switch_hold 가
  지났을 때만 예외). pd_lane_run4 에서 목표가 0.15초 만에 뒤집혀서 넣어 봤지만, 0.55 배치는 계속 다시 골라야 빠져나가는
  경우가 있어 run.py 4개가 막힘 -> 끔. '바로 옆 차선만 목표' 도 시험했지만 0.55 배치는 오른쪽 -> 왼쪽을 가운데 LIMO 사이로
  대각선으로 가로질러야 해서 static_noise 40번 중 2번만 성공 -> 코드에 안 넣음.

step(now, goal) 한 번 = 한 제어 주기:
  - 라이다 장애물 지도 갱신 (LIMO 는 차선 방향으로 놓인 0.322 x 0.22 m 박스 + 여유 clearance 로 본다)
    (11/04) 위치는 본 것의 평균(map_average), 이미 확실한 LIMO 와 겹치는 새 검출은 버림(ghost_*).
  - 후보 차선 3곳(차선 가운데)마다, 그 차선으로 가는 주행을 실제 조향 한계(max_steer, wheel_base)로
    앞 plan_horizon 만큼 미리 굴려 보고(rollout) 장애물과 닿는지 검사한다.
    (11/04) 보통(lookahead)으로 굴려서 막히는 차선은 급하게 꺾는 것(lookahead_short)도 굴려 보고
    더 멀리 가는 쪽으로 따라간다. 차 모양은 몸통 사각형 + (Gazebo 만) 바퀴를 감싼 사각형.
  - (11/05) plan_depth 2: 차선마다 '그 차선으로 가다가 switch_step 마다 옆 차선으로 한 번 더 옮기면' 까지
    plan_horizon_long 만큼 굴려 보고 가장 멀리 가는 거리(reach)로 비교한다 (막다른 차선 피하기, static_run3).
    굴릴 때 3차선 바깥 선을 넘어도 막힌 것으로 본다 (edge_check).
  - 지금 목표 차선이 끝까지 안 닿으면 유지한다 (goal 차선이 끝까지 비면 goal 로).
    아니면 reach 가 switch_margin 이상 긴 차선으로 바꾸되, 바꾼 뒤 switch_hold 초는 유지한다
    (지금 목표가 commit_distance 안에서 막히면 바로 바꿈). goal 은 +0.3 m 가산.
    (11/12, lane_commit 켰을 때만) 목표 차선으로 가는 중(아직 목표 차선 밖)에는 목표를 고정, 목표가 commit_distance
    안에서 막히고 switch_hold 가 지났을 때만 다시 고른다.
  - 차선 사이(선 위)로 가는 경로는 쓰지 않는다. 고른 차선도 commit_distance 보다 짧게 닿으면
    감속하고, stop_distance 안에서 닿게 되면 선다.
  - (11/12) 지금 자리에서 이미 여유 박스(LIMO + clearance)와 겹치는 LIMO 는, 지금 안 겹칠 만큼만 여유를 줄여서 본다
    (overlap_relax, 진짜 크기보다 overlap_extra 까지). 그보다 더 가까워지는 길은 그대로 막힘.
    static_run6: 진짜 틈 2.6 cm 인데 여유 1.5 cm + 지도 오차 2 cm 로 '이미 닿음' -> 어느 길도 0 m, 후진도 못 해서
    제한시간까지 22초 정지 (제한시간 뒤 차선 주행으로 그냥 빠져나감).
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
        self.target_since = -1e9
        self.lat_snap_pending = False   # 옆 위치를 LIMO 로 맞춰야 하는지 (차 위치 기준일 때, 11/12)
        self.relax_logged = False       # 겹친 LIMO 여유 줄임 로그 (11/12)
        self.lat_offset = 0.0

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
        lane_ref, why = self.node.lane_for_frame()
        self.frame.begin(pose, lane_ref)
        cfg = self.cfg
        self.lat_offset = 0.0
        self.relax_logged = False
        if lane_ref is not None:
            self.lat_snap_pending = False
            self.node.get_logger().info('[3차선] 기준 좌표: %s' % why)
        else:
            self.lat_snap_pending = bool(cfg['lat_snap'])
            self.node.get_logger().info('[3차선] 기준 좌표: 차 방향 (%s)%s%s' % (
                why, self._snap_axis(pose), ' - 옆 위치는 LIMO 보이면 맞춤' if self.lat_snap_pending else ''))
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
        self.target_since = -1e9
        self.active = True
        self.released_at = None
        return True

    def _snap_axis(self, pose):
        """(11/12) 차 방향 기준일 때 가까운 트랙 축으로 맞춤. 로그에 붙일 설명을 돌려준다."""
        cfg = self.cfg
        if not cfg['axis_snap']:
            return ''
        x, y, yaw = pose
        ref = math.radians(cfg['axis_ref_deg'])
        rel = math.atan2(math.sin(yaw - ref), math.cos(yaw - ref))
        axis = ref + round(rel / (math.pi / 2.0)) * (math.pi / 2.0)
        err = math.atan2(math.sin(yaw - axis), math.cos(yaw - axis))
        if abs(err) > math.radians(cfg['axis_snap_max_deg']):
            return ' - 트랙 축에서 %.0f도 벗어나 그대로 씀' % math.degrees(abs(err))
        ox, oy, _ = self.frame.origin
        self.frame.origin = (ox, oy, math.atan2(math.sin(axis), math.cos(axis)))
        return ' -> 트랙 축 %.0f도로 맞춤 (차 방향 %.0f도, %+.0f도 차이)' % (
            math.degrees(math.atan2(math.sin(axis), math.cos(axis))), math.degrees(yaw), math.degrees(err))

    def _snap_lateral(self, s_car, who):
        """(11/12) 차 위치 기준일 때, 가까이 확실히 본 LIMO 가 차선 가운데(d = 0, +-lane_width)에 오도록 옆으로 한 번 옮긴다.
        LIMO 마다 '가까운 차선 가운데에서 얼마나 비켜 있나'를 구해, lat_snap_spread 안에 가장 많이 모인 값들
        (lat_snap_min_items 대 이상이고 절반 넘게)의 평균만큼 옮김. 유령·잘못 찍힌 LIMO 는 묶음에서 빠짐.
        lat_snap_max 보다 크면 어느 차선인지 애매해서 안 옮김. 맞췄거나 안 맞추기로 했으면 True."""
        cfg = self.cfg
        w = self.width()
        items = [it for it in self.map.confirmed(cfg['confirm_hits'])
                 if s_car - 0.3 <= it['s'] <= s_car + cfg['lat_snap_range'] and abs(it['d']) <= 1.5 * w]
        need = max(1, int(cfg['lat_snap_min_items']))
        if len(items) < need:
            return False
        res = sorted(it['d'] - w * round(it['d'] / w) for it in items)
        group = []
        for i in range(len(res)):
            g = [r for r in res[i:] if r - res[i] <= cfg['lat_snap_spread']]
            if len(g) > len(group):
                group = g
        if len(group) < need or 2 * len(group) <= len(res):
            return False                    # 아직 모자라거나 제각각 - 더 보고 판단
        off = sum(group) / len(group)
        log = self.node.get_logger()
        if abs(off) > cfg['lat_snap_max']:
            log.warn('[%s] LIMO %d대가 차선 가운데에서 %+.2fm 비켜 있음 - %.2fm 넘어 어느 차선인지 애매해서 옆 위치 그대로'
                     % (who, len(group), off, cfg['lat_snap_max']))
            return True
        nx, ny = self.frame.frame_to_odom(0.0, off)
        self.frame.origin = (nx, ny, self.frame.origin[2])
        for it in self.map.items:
            it['d'] -= off
        self.lat_offset = off
        log.info('[%s] 기준 좌표 옆 위치를 LIMO %d대(전체 %d대)로 맞춤: %+.2fm (LIMO 가 차선 가운데에 있다는 가정)'
                 % (who, len(group), len(res), off))
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
        return self._roll(s0, d0, psi0, d_target, boxes, horizon, la, 0.0)[0]

    def _roll(self, s0, d0, psi0, d_target, boxes, horizon, la, mark_step):
        """rollout + mark_step [m] 마다 지나간 상태 [(간 거리, s, d, psi)] (0 이면 기록 안 함)."""
        node = self.node
        cfg = self.cfg
        wb, ms = node.wheel_base, node.max_steer
        if la is None:
            la = cfg['lookahead']
        shapes = self._shapes()
        ds = 0.04
        s_, d, psi = s0, d0, psi0
        marks = []
        next_mark = mark_step if mark_step > 0.0 else float('inf')
        if self._car_hits(s_, d, psi, boxes, shapes):
            return 0.0, marks
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
            if self._car_hits(s_, d, psi, boxes, shapes) or self._off_edge(d, psi):
                return traveled, marks
            if traveled >= next_mark - 1e-9:
                marks.append((traveled, s_, d, psi))
                next_mark += mark_step
        return horizon, marks

    def _off_edge(self, d, psi):
        """바퀴(Gazebo 는 바퀴 사각형, 실차는 몸통)가 3차선 바깥 선을 넘는지 (edge_check, 11/05)."""
        cfg = self.cfg
        if not cfg['edge_check']:
            return False
        if cfg['wheel_box_length'] > 0.0 and cfg['wheel_box_width'] > 0.0:
            hl, hw = 0.5 * cfg['wheel_box_length'], 0.5 * cfg['wheel_box_width']
        else:
            hl, hw = 0.5 * cfg['car_length'], 0.5 * cfg['car_width']
        reach = abs(d) + hl * abs(math.sin(psi)) + hw * abs(math.cos(psi))
        return reach > 1.5 * self.width() + cfg['edge_margin']

    def _las(self):
        cfg = self.cfg
        las = [cfg['lookahead']]
        if cfg['lookahead_short'] > 0.0:
            las.append(cfg['lookahead_short'])
        return las

    def plan(self, s_, d, psi, boxes):
        """차선마다 {'reach', 'la', 'free'} 와 굴려 보는 거리 H.
        reach: 그 차선으로 가다가 switch_step 마다 '여기서 옆 차선으로 한 번 더 옮기면' 까지 굴려 보고
               가장 멀리 가는 거리 (H 까지). 막다른 차선을 피하려고 (11/05, static_run3). plan_depth 1 이면 그 차선만.
        free:  그 차선으로만 갈 때 닿기 전까지 거리 (plan_horizon 까지, 감속·정지 판단에 씀).
        la:    그 차선으로 갈 때 따라갈 거리 (lookahead / lookahead_short 중 reach 가 긴 것)."""
        cfg = self.cfg
        depth = int(cfg['plan_depth'])
        H = max(cfg['plan_horizon_long'], cfg['plan_horizon']) if depth >= 2 else cfg['plan_horizon']
        step = cfg['switch_step'] if depth >= 2 else 0.0
        out = {}
        for l1 in self.LANES:
            best = None
            for la in self._las():
                f1, marks = self._roll(s_, d, psi, self.center(l1), boxes, H, la, step)
                r = f1
                if depth >= 2 and f1 < H - 1e-6:
                    for a, ss, dd, pp in marks:
                        if a >= f1 - 1e-6 or r >= H - 1e-6:
                            break
                        for l2 in self.LANES:
                            if l2 != l1:
                                r = max(r, a + self.rollout(ss, dd, pp, self.center(l2), boxes, H - a))
                if best is None or r > best['reach'] + 1e-6:
                    best = {'reach': r, 'la': la, 'free': min(f1, cfg['plan_horizon'])}
                if best['reach'] >= H - 1e-6:
                    break
            out[l1] = best
        return out, H

    def _all_boxes(self):
        cfg = self.cfg
        hl = 0.5 * cfg['limo_length'] + cfg['clearance']
        hw = 0.5 * cfg['limo_width'] + cfg['clearance']
        return [(it['s'], it['d'], hl, hw) for it in self.map.confirmed(cfg['confirm_hits'])]

    def _relax(self, boxes, s_, d, psi, who=None):
        """(11/12) 지금 자리에서 이미 겹친 박스는 지금 안 겹칠 만큼(2 mm 씩) + overlap_slack 만큼 줄인다
        (clearance + overlap_extra 까지). overlap_slack 은 방향 틀 때 차 뒤쪽이 옆으로 도는 몫 (없으면 멀어지는 길도 4 cm 에서 막힘).
        그만큼 줄여도 겹치면(진짜로 닿았다고 봄) 그대로 둔다. 그 LIMO 에 지금보다 overlap_slack 넘게 가까워지는 길은 막힘."""
        cfg = self.cfg
        if not cfg['overlap_relax']:
            return boxes
        shapes = self._shapes()
        limit = cfg['clearance'] + cfg['overlap_extra']
        out, relaxed = [], []
        for b in boxes:
            if not self._car_hits(s_, d, psi, [b], shapes):
                out.append(b)
                continue
            bs, bd, hl, hw = b
            nb, k = b, 0.0
            while k < limit - 1e-9:
                k = min(k + 0.002, limit)
                if not self._car_hits(s_, d, psi, [(bs, bd, hl - k, hw - k)], shapes):
                    k = min(k + cfg['overlap_slack'], limit)
                    nb = (bs, bd, hl - k, hw - k)
                    break
            out.append(nb)
            if nb is not b:
                relaxed.append((bs, bd, k))
        if who is not None:
            self.node.dbg('static/relaxed', float(len(relaxed)))
            if relaxed and not self.relax_logged:
                self.node.get_logger().warn(
                    '[%s] 지금 자리에서 LIMO 여유 박스와 겹침 - 그 LIMO 는 여유를 줄여서 봄 (지금보다 크게 가까워지는 길은 막힘) (%s)'
                    % (who, ', '.join('s=%.2f d=%+.2f %.1fcm' % (a, b_, 100.0 * k_) for a, b_, k_ in relaxed)))
            self.relax_logged = bool(relaxed)
        return out

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
        if (done >= cfg['backup_distance']
                or self.rollout_back(s_, d, psi, self._relax(self._all_boxes(), s_, d, psi), 0.04) < 0.04):
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
        if self.lat_snap_pending and self._snap_lateral(s_, who):
            self.lat_snap_pending = False
            s_, d, psi = self.frame.car(pose)
        node.dbg('static/lat_offset', self.lat_offset)

        if self.changing and self.settled(d, psi):
            self.lane = self.target
            self.changing = False
            log.info('[%s] 차선 변경 완료 - %s 차선 (d=%.2fm)' % (who, self.name(self.lane), d))
        if not self.changing:
            self.check_lane(d, now, who)
        elif cfg['lane_from_odom']:
            self.lane = self.lane_of(d)     # 바꾸는 중에도 지금 있는 차선 (11/12)

        if self.backing is not None and self._back_step(pose, s_, d, psi, now, who):
            return StepResult(pose, s_, d, psi, blocked=True)

        boxes = self._relax(self._boxes(s_), s_, d, psi, who)
        # 차선마다 굴려 봄: 보통(lookahead) / 급하게(lookahead_short, 11/04) x 한 번 더 옆 차선으로 옮기기(11/05)
        info, horizon = self.plan(s_, d, psi, boxes)
        reach = dict((ln, info[ln]['reach']) for ln in self.LANES)
        free = dict((ln, info[ln]['free']) for ln in self.LANES)
        full = [ln for ln in self.LANES if reach[ln] >= horizon - 1e-6]
        node.dbg('static/free_target', free[self.target])
        node.dbg('static/reach_target', reach[self.target])

        def score(ln):
            sc = reach[ln] - 0.02 * abs(ln - self.lane)
            if goal is not None and ln == goal:
                sc += 0.3
            return sc

        old = self.target
        held = now - self.target_since
        if cfg['lane_commit'] and self.changing and self.lane != old:
            # (11/12) 목표 차선으로 가는 중: 고정. 목표가 곧 막히고 switch_hold 가 지났을 때만 다시 고름
            new = old
            if free[old] < cfg['commit_distance'] and held >= cfg['switch_hold']:
                best = max(self.LANES, key=score)
                if best != old and score(best) > score(old) + cfg['switch_margin']:
                    new = best
        elif goal is not None and goal in full:
            new = goal
        elif old in full:
            new = old
        else:
            # 자꾸 바꾸지 않게: switch_margin 이상 나을 때만, 바꾼 뒤 switch_hold 초는 유지
            # (지금 목표가 commit_distance 안에서 막히면 바로 바꿈)
            best = max(self.LANES, key=score)
            new = old
            if best != old and score(best) > score(old) + cfg['switch_margin']:
                if held >= cfg['switch_hold'] or free[old] < cfg['commit_distance']:
                    new = best

        if new != old:
            log.info('[%s] 목표 차선 %s -> %s (지금 %s / 앞 %.1fm 굴려본 결과 %s)'
                     % (who, self.name(old), self.name(new), self.name(self.lane), horizon,
                        ' '.join('%s=%.2f' % (self.name(ln), reach[ln]) for ln in self.LANES)))
            self.target = new
            self.target_since = now
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
                    room = self.rollout_back(s_, d, psi, self._relax(self._all_boxes(), s_, d, psi),
                                             cfg['backup_distance'])
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
        node.dbg('static/lookahead', info[self.target]['la'])
        node.publish_cmd(v, self.steer(pose, s_, self.center(self.target), v, info[self.target]['la']))
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
