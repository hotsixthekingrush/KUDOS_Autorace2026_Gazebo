"""미션 8 - 라바콘 회피.

차선 인식이 아닌 라이다 기반으로 주행한다.
좌우 라바콘을 짝지어 게이트 중점을 만들고, 그 중점들을 이은 선을 "가고 싶은 길" 로 쓴다.

주행 방식 (cone.use_rollout)
  true  (11/02 기본) 미리 굴려보기:
        게이트 길을 옆으로 0, ±offset_step, ... ±offset_max 만큼 옮긴 후보 길들을
        실제 조향 한계·축간거리로 Pure Pursuit 하며 horizon m 굴려보고,
        차 몸통(car_length x car_width)이 콘(반지름 + clearance)에 안 닿는 것 중
        가운데(옮긴 양이 작은)에 가깝고 콘과 여유가 큰 것을 고른다.
        차 옆·뒤로 지나간 콘은 라이다 시야(앞쪽) 밖이라, odom 기준으로 기억해 두고 같이 검사한다.
        조향이 느린 차는 steer_rate_deg 로 흉내 내서 굴려본다 (0 = 바로 바뀜).
  false 예전 방식: 게이트 중점 선 위 lookahead 점을 Pure Pursuit.
게이트를 못 만들면(한쪽 콘만 보이거나 짝이 안 맞으면) 한쪽 콘 줄 따라가기 (11/02 사용자 제안):
  한 줄로 이어진 콘들(다음 콘 = 진행 방향 side_max_turn_deg 안, side_max_step 안에서 가장 가까운 콘)을
  선으로 잇고, 차가 그 선의 어느 쪽에 있는지로 왼쪽/오른쪽 줄을 정한다 (차 기준 y 부호로 나누면
  곡선·S자에서 같은 줄 콘이 양쪽으로 갈림 - 11/02 시험).
  side_use_rollout true  (기본) 그 선을 통로 반폭만큼 안쪽으로 옮긴 "가상 가운데 길"을 위 미리 굴려보기로 따라감
                         (콘에 안 닿는지 차 몸통으로 검사됨).
                   false 차선 따라가기처럼 조향 = side_kp_heading x 선과 틀어진 각도 + side_kp_lateral x 옆 오차.
  통로 반폭은 마지막으로 본 게이트 폭의 절반 (없으면 side_half_width).
끝내는 때: 라이다에서 콘이 다 빠져도, 마지막으로 본 게이트(또는 콘 줄 끝) 선을 앞바퀴가 pass_margin 넘게
  지날 때까지는 계속 간다 (한쪽 콘만 있으면 마지막 콘이 일찍 시야에서 빠지기 때문).
그 쪽 콘이 1개뿐이면 그 콘에서 옆으로 고정 거리, 콘이 아예 없으면 Follow the Gap.
odom 이 없으면 예전 방식(게이트 Pure Pursuit)으로 간다.
"""

import math

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.controllers import steering_to_omega, clamp
from ar_mission_supervisor.geometry import (
    distance, pure_pursuit_curvature, point_along_path, find_largest_gap)


class AvoidCone(SafeState):

    OUTCOMES = ['done']

    def __init__(self, node):
        SafeState.__init__(self, node, 'AVOID_CONE', self.OUTCOMES)
        self.miss = 0
        self.entered_at = None
        self.last_steer = 0.0
        self.memory = []          # [[ox, oy, r]] odom 기준 콘 기억
        self.steer_model = 0.0    # 실제 바퀴 조향 추정 (steer_rate_deg > 0 일 때)
        self.last_time = None
        self.warned_at = None
        self.half_width = None    # 마지막으로 본 게이트 폭의 절반 (한쪽 선 따라가기용)
        self.side = None          # 한쪽 선 따라가기에서 따라가는 쪽 (+1 왼쪽, -1 오른쪽)
        self.last_pair = None     # 가장 먼 게이트의 (왼쪽 콘, 오른쪽 콘) base_link
        self.final_gate = None    # 지금까지 본 가장 먼 게이트 선 (odom 점 x, y, 앞쪽 방향 nx, ny)

    def on_enter(self):
        self.miss = 0
        self.entered_at = self.node.clock_sec()
        self.last_steer = 0.0
        self.memory = []
        self.steer_model = 0.0
        self.last_time = None
        self.warned_at = None
        self.half_width = None
        self.side = None
        self.last_pair = None
        self.final_gate = None

    def run(self):
        node = self.node
        cfg = node.cone

        while node.ok():
            if self.reset_requested():
                return 'aborted'

            now = node.clock_sec()
            if now - self.entered_at > cfg['max_duration']:
                node.get_logger().warn('[CONE] 제한시간 초과 - 복귀')
                return 'done'

            cones = self._collect_cones()
            node.dbg('cone/count', len(cones))

            if not cones:
                if self._gate_not_passed(cfg):
                    # 라이다 시야(앞쪽)에선 빠졌지만 마지막 게이트 선을 앞바퀴가 아직 안 넘음 - 넘을 때까지 계속
                    # (한쪽 콘만 있으면 마지막 콘이 일찍 시야에서 빠져, 앞바퀴가 마지막 줄 전에 끝나던 문제)
                    self._publish(cfg['speed'], self.last_steer, now)
                    self.sleep()
                    continue
                # 라바콘이 안 보임 - 구간 종료로 판단 (몇 회 유예)
                self.miss += 1
                if self.miss >= node.perception['exit_miss_count']:
                    node.get_logger().info('[CONE] 라바콘 구간 종료')
                    return 'done'
                self._publish(cfg['speed'], self.last_steer, now)
                self.sleep()
                continue

            self.miss = 0

            gates = self._gates(cones, cfg)
            pose = node.odom_pose()
            if pose is not None:
                self._remember(pose, cfg)
            if gates:
                self.side = None                     # 게이트가 다시 보이면 따라갈 쪽은 다음에 새로 정함
                if pose is not None:
                    (lx, ly), (rx, ry) = self.last_pair
                    self._note_gate(pose, (0.5 * (lx + rx), 0.5 * (ly + ry)), (ly - ry, rx - lx))
            if cfg['use_rollout'] and pose is not None and gates:
                self._drive_rollout(pose, gates, cfg, now)
                self.sleep()
                continue

            if not gates and self._drive_side_line(cones, pose, cfg, now):
                self.sleep()
                continue

            target = self._make_target(cones, cfg)

            if target is None:
                node.get_logger().warn('[CONE] 경로 없음 - 감속 직진')
                self._publish(cfg['creep_speed'], 0.0, now)
                self.sleep()
                continue

            self._drive_to(target, cfg, now)
            self.sleep()

        return 'aborted'

    # ------------------------------------------------------------------
    def _collect_cones(self):
        """전방 ROI 안의 정적 클러스터만 라바콘 후보로 추린다."""
        node = self.node
        cfg = node.cone
        obstacles = node.obstacles()
        if obstacles is None:
            return []

        cones = []
        for o in obstacles:
            if o.is_dynamic:
                continue
            if o.x < cfg['roi_min_x'] or o.x > cfg['roi_max_x']:
                continue
            if abs(o.y) > cfg['roi_max_y']:
                continue
            if o.radius > cfg['max_radius']:
                continue
            cones.append(o)
        return cones

    def _gates(self, cones, cfg):
        """좌우 콘을 앞에서부터 짝지어 게이트 중점 목록 (base_link)."""
        left = sorted([c for c in cones if c.y > 0.0], key=lambda c: c.x)
        right = sorted([c for c in cones if c.y <= 0.0], key=lambda c: c.x)

        gates = []
        for l, r in zip(left, right):
            # 전후 거리가 크게 어긋나면 잘못된 짝
            if abs(l.x - r.x) > cfg['pair_max_dx']:
                break
            # 두 콘을 이은 선이 차 진행 방향과 거의 나란하면 게이트가 아니라 같은 줄의 콘 2개
            # (곡선에서 한쪽 줄만 보일 때 차가 틀어지면 같은 줄 콘이 왼쪽/오른쪽으로 갈려 짝지어지던 문제)
            if math.degrees(math.atan2(abs(l.x - r.x), l.y - r.y)) > cfg['pair_max_angle_deg']:
                break
            width = distance(l.x, l.y, r.x, r.y) - (l.radius + r.radius)
            if width < cfg['min_gate_width']:
                break
            gates.append((0.5 * (l.x + r.x), 0.5 * (l.y + r.y)))
            self.last_pair = ((l.x, l.y), (r.x, r.y))
            if len(gates) == 1:
                self.half_width = 0.5 * distance(l.x, l.y, r.x, r.y)
        return gates

    def _note_gate(self, pose, m, n):
        """base_link 의 게이트 선 (점 m, 앞쪽 방향 n) 을 odom 으로 바꿔, 지금까지 본 것보다 더 앞이면 기억."""
        k = math.hypot(n[0], n[1])
        if k < 1e-6:
            return
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my = x + c * m[0] - s * m[1], y + s * m[0] + c * m[1]
        nx, ny = (c * n[0] - s * n[1]) / k, (s * n[0] + c * n[1]) / k
        g = self.final_gate
        if g is None or (mx - g[0]) * g[2] + (my - g[1]) * g[3] > 0.05:
            self.final_gate = (mx, my, nx, ny)

    def _gate_not_passed(self, cfg):
        """마지막 게이트 선을 앞바퀴가 아직 pass_margin 넘게 안 지났나 (그 선에서 0.8 m 안일 때만)."""
        node = self.node
        g = self.final_gate
        pose = node.odom_pose()
        if g is None or pose is None:
            return False
        x, y, yaw = pose
        fx = x + node.front_wheel_x * math.cos(yaw)
        fy = y + node.front_wheel_x * math.sin(yaw)
        along = (fx - g[0]) * g[2] + (fy - g[1]) * g[3]
        return along < cfg['pass_margin'] and math.hypot(fx - g[0], fy - g[1]) < 0.8

    def _drive_side_line(self, cones, pose, cfg, now):
        """한쪽 콘 줄을 따라간다. 줄을 못 찾으면 False."""
        node = self.node
        ahead = sorted((c.x, c.y) for c in cones)
        if not ahead:
            return False
        beside = []                       # 라이다 시야 밖으로 지나간 콘 (odom 기억)
        if pose is not None:
            x, y, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            for m in self.memory:
                bx = c * (m[0] - x) + s * (m[1] - y)
                by = -s * (m[0] - x) + c * (m[1] - y)
                if bx < ahead[0][0] - 0.05 and abs(by) < 1.0:
                    beside.append((bx, by))
        found = self._side_line(beside, ahead, cfg)
        if found is None:
            return False
        side, chain = found
        self.side = side
        half = self.half_width if self.half_width is not None else cfg['side_half_width']
        if pose is not None:
            (px, py), (qx, qy) = chain[-2], chain[-1]
            self._note_gate(pose, (qx, qy), (qx - px, qy - py))   # 줄 끝 콘에서 줄에 수직인 선

        if cfg['side_use_rollout'] and pose is not None:
            mids = [m for m in self._line_to_center(chain, side, half) if m[0] > 0.05]
            if mids:
                self._drive_rollout(pose, mids, cfg, now)
                node.dbg('cone/mode', 'side_roll_left' if side > 0 else 'side_roll_right')
                return True

        # 차선 따라가기처럼 조향: 차 옆(또는 가장 가까운) 콘 a 와 그 다음 콘 b 를 이은 선
        k = 0
        while k + 2 < len(chain) and chain[k + 1][0] <= 0.0:
            k += 1
        a, b = chain[k], chain[k + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)                       # 선 방향 = 차가 틀어진 각도 (왼쪽 +)
        dist = (dx * (0.0 - a[1]) - dy * (0.0 - a[0])) / n  # 차 중심이 선 왼쪽이면 +
        want = -half if side > 0 else half                 # 왼쪽 줄이면 선 오른쪽에, 오른쪽 줄이면 왼쪽에
        lateral = want - dist                              # + 면 왼쪽으로 가야 함
        steer = clamp(cfg['side_kp_heading'] * heading + cfg['side_kp_lateral'] * lateral,
                      -node.max_steer, node.max_steer)
        speed = cfg['speed'] / (1.0 + cfg['curve_gain'] * abs(math.tan(steer) / node.wheel_base))
        speed = max(cfg['creep_speed'], speed)
        self._publish(speed, steer, now)
        node.dbg('cone/mode', 'side_left' if side > 0 else 'side_right')
        node.dbg('cone/side_heading', heading)
        node.dbg('cone/side_lateral', lateral)
        return True

    def _side_line(self, beside, ahead, cfg):
        """한 줄로 이어진 콘 묶음 하나를 고른다 -> (쪽 +1 왼쪽/-1 오른쪽, [콘...] 뒤에서 앞 순) 또는 None.

        차에 가까운 콘 몇 개에서 각각 줄을 이어 보고, 가장 멀리까지 이어지는 줄을 고른다
        (가운데 콘이 빠진 줄보다 끝까지 이어지는 반대쪽 줄이 낫다). 같으면 전에 따라가던 쪽.
        """
        pool = beside + ahead
        seeds = sorted(pool, key=lambda p: math.hypot(p[0], p[1]))[:4]
        best = None
        for seed in seeds:
            chain = [seed]
            ref = (1.0, 0.0)                   # 첫 연결은 차 진행 방향 기준
            while True:
                p = self._link(chain[-1], ref, pool, chain, cfg)
                if p is None:
                    break
                ref = (p[0] - chain[-1][0], p[1] - chain[-1][1])
                chain.append(p)
            if len(chain) < 2 or chain[-1][0] <= 0.0:
                continue
            side = self._which_side(chain)
            key = chain[-1][0] + (0.05 if side == self.side else 0.0)
            if best is None or key > best[0]:
                best = (key, side, chain)
        return None if best is None else (best[1], best[2])

    @staticmethod
    def _link(cur, ref, pool, used, cfg):
        """cur 다음 콘: ref 방향에서 side_max_turn_deg 안, side_max_step 안에서 가장 가까운 콘."""
        lim = math.radians(cfg['side_max_turn_deg'])
        best = None
        for p in pool:
            if p in used:
                continue
            dx, dy = p[0] - cur[0], p[1] - cur[1]
            d = math.hypot(dx, dy)
            if d < 0.05 or d > cfg['side_max_step']:
                continue
            if abs(math.atan2(ref[0] * dy - ref[1] * dx, ref[0] * dx + ref[1] * dy)) > lim:
                continue
            if best is None or d < best[0]:
                best = (d, p)
        return None if best is None else best[1]

    @staticmethod
    def _which_side(chain):
        """차(원점)가 줄의 오른쪽에 있으면 왼쪽 줄(+1), 왼쪽에 있으면 오른쪽 줄(-1).

        차 옆을 지나는 선분(없으면 첫 선분)으로 판단.
        """
        k = 0
        while k + 2 < len(chain) and chain[k + 1][0] <= 0.0:
            k += 1
        (ax, ay), (bx, by) = chain[k], chain[k + 1]
        cross = (bx - ax) * (0.0 - ay) - (by - ay) * (0.0 - ax)
        return -1.0 if cross > 0.0 else 1.0

    @staticmethod
    def _line_to_center(chain, side, half):
        """콘 줄을 줄에 수직으로 half 만큼 통로 안쪽으로 옮긴 점들 (가상 게이트 중점)."""
        out = []
        for k, p in enumerate(chain):
            q0 = chain[k - 1] if k > 0 else p
            q1 = chain[k + 1] if k + 1 < len(chain) else p
            tx, ty = q1[0] - q0[0], q1[1] - q0[1]
            n = math.hypot(tx, ty)
            if n < 1e-6:
                continue
            # 줄 왼쪽 방향 (-ty, tx). 왼쪽 줄이면 통로는 줄 오른쪽
            out.append((p[0] + side * half * ty / n, p[1] - side * half * tx / n))
        return out

    def _make_target(self, cones, cfg):
        """게이트 중점 시퀀스 -> lookahead 목표점 (예전 방식).

        게이트 중점들을 차에서부터 선으로 이어, 그 선 위에서 딱 lookahead 만큼 간 점을 목표로 한다.
        """
        gates = self._gates(cones, cfg)
        if gates:
            self.node.dbg('cone/mode', 'gate')
            return point_along_path(gates, cfg['lookahead'])

        left = [c for c in cones if c.y > 0.0]
        right = [c for c in cones if c.y <= 0.0]
        # 한쪽만 보이는 경우 - 고정 오프셋으로 가상 목표
        single = sorted(left if left else right, key=lambda c: c.x)
        if single:
            self.node.dbg('cone/mode', 'single')
            c = single[0]
            sign = -1.0 if c.y > 0.0 else 1.0
            offset = c.radius + cfg['side_offset']
            return (c.x, c.y + sign * offset)

        self.node.dbg('cone/mode', 'gap')
        return self._follow_the_gap(cfg)

    def _follow_the_gap(self, cfg):
        """페어링 실패 시 폴백."""
        node = self.node
        points = node.forward_scan_points(cfg['roi_max_x'])
        result = find_largest_gap(points, cfg['robot_width'], cfg['lookahead'])
        if result is None:
            return None
        angle, rng = result
        return (rng * math.cos(angle), rng * math.sin(angle))

    def _drive_to(self, target, cfg, now):
        node = self.node
        tx, ty = target
        kappa = pure_pursuit_curvature(tx, ty)

        # 곡률 -> 조향각 (애커만 기구학 역변환)
        steer = math.atan(kappa * node.wheel_base)
        steer = clamp(steer, -node.max_steer, node.max_steer)

        # 곡률이 클수록 감속
        speed = cfg['speed'] / (1.0 + cfg['curve_gain'] * abs(kappa))
        speed = max(cfg['creep_speed'], speed)

        self._publish(speed, steer, now)

        node.dbg('cone/target_x', tx)
        node.dbg('cone/target_y', ty)
        node.dbg('cone/kappa', kappa)

    def _publish(self, speed, steer, now):
        """조향각으로 명령하고, 느린 조향 모델(steer_model)을 갱신한다."""
        node = self.node
        cfg = node.cone
        self.last_steer = steer
        rate = math.radians(cfg['steer_rate_deg'])
        if rate > 0.0 and self.last_time is not None:
            step = rate * max(0.0, now - self.last_time)
            self.steer_model += clamp(steer - self.steer_model, -step, step)
        else:
            self.steer_model = steer
        self.last_time = now
        node.publish_cmd(speed, steering_to_omega(speed, steer, node.wheel_base))

    # ------------------------------------------------------------------
    # 미리 굴려보기
    # ------------------------------------------------------------------
    def _remember(self, pose, cfg):
        """라이다 콘을 odom 기준으로 기억 (차 옆으로 지나간 콘도 몸통 검사에 쓰려고)."""
        node = self.node
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        merge = cfg['memory_merge']
        for o in node.obstacles() or ():
            if o.is_dynamic or o.radius > cfg['max_radius']:
                continue
            if o.x > cfg['roi_max_x'] or abs(o.y) > cfg['roi_max_y']:
                continue
            ox = x + c * o.x - s * o.y
            oy = y + s * o.x + c * o.y
            best, best_d = None, merge
            for m in self.memory:
                d = math.hypot(m[0] - ox, m[1] - oy)
                if d <= best_d:
                    best, best_d = m, d
            if best is None:
                self.memory.append([ox, oy, float(o.radius)])
            else:
                best[0] += 0.3 * (ox - best[0])
                best[1] += 0.3 * (oy - best[1])
                best[2] += 0.3 * (float(o.radius) - best[2])
        # 차 뒤로 충분히 지나간 콘은 지움
        keep = []
        for m in self.memory:
            bx = c * (m[0] - x) + s * (m[1] - y)
            if bx > -cfg['memory_behind']:
                keep.append(m)
        self.memory = keep

    def _drive_rollout(self, pose, gates, cfg, now):
        """게이트 길을 옆으로 옮긴 후보들을 Pure Pursuit 로 따라가 보고, 안 닿는 것 중 가운데에 가까운 것."""
        node = self.node
        wb, ms = node.wheel_base, node.max_steer
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        reach = cfg['horizon'] + 0.6
        obs = []
        for m in self.memory:
            dx, dy = m[0] - x, m[1] - y
            bx, by = c * dx + s * dy, -s * dx + c * dy
            if -0.8 < bx < reach and abs(by) < 1.2:
                obs.append((bx, by, m[2] + cfg['clearance']))

        rate = math.radians(cfg['steer_rate_deg'])
        per_m = rate / max(cfg['speed'], 0.05) if rate > 0.0 else 0.0   # 1 m 갈 때 바뀔 수 있는 조향
        step = max(0.01, cfg['offset_step'])
        n = int(round(cfg['offset_max'] / step))
        offsets = [0.0] + [k * step * sgn for k in range(1, n + 1) for sgn in (1.0, -1.0)]

        best = None          # (score, offset, first_steer, end, margin)
        fallback = None      # 다 닿으면: 여유가 가장 큰 것
        for off in offsets:
            path = self._shift(gates, off)
            first, end, margin = self._roll(path, self.steer_model, per_m, wb, ms, obs, cfg)
            cand = (off, first, end, margin)
            if fallback is None or margin > fallback[3]:
                fallback = cand
            if margin < 0.0:
                continue
            score = -cfg['w_offset'] * abs(off) + cfg['w_margin'] * min(margin, cfg['margin_cap'])
            if best is None or score > best[0]:
                best = (score,) + cand

        if best is not None:
            _, off, steer, end, margin = best
            node.dbg('cone/mode', 'roll')
            v = cfg['speed'] / (1.0 + cfg['curve_gain'] * abs(math.tan(steer) / wb))
            v = max(cfg['creep_speed'], v)
        else:
            off, steer, end, margin = fallback
            node.dbg('cone/mode', 'roll_tight')
            v = cfg['creep_speed']
            if self.warned_at is None or now - self.warned_at > 1.0:
                node.get_logger().warn('[CONE] 안 닿는 길을 못 찾음 - 여유가 가장 큰 쪽으로 천천히 (여유 %.3fm)'
                                       % margin)
                self.warned_at = now

        self._publish(v, steer, now)
        node.dbg('cone/target_x', end[0])
        node.dbg('cone/target_y', end[1])
        node.dbg('cone/kappa', math.tan(steer) / wb)
        node.dbg('cone/margin', margin)
        node.dbg('cone/offset', off)

    @staticmethod
    def _shift(gates, off):
        """게이트 중점들을 길의 왼쪽(+)/오른쪽(-) 수직 방향으로 off 만큼 옮긴 꺾은선 (차 위치에서 시작)."""
        if abs(off) < 1e-9:
            return [(0.0, 0.0)] + list(gates)
        pts = [(0.0, 0.0)] + list(gates)
        out = [(0.0, 0.0)]
        for k in range(1, len(pts)):
            ax, ay = pts[k - 1]
            bx, by = pts[k + 1] if k + 1 < len(pts) else pts[k]   # 앞뒤 점으로 그 자리 길 방향
            dx, dy = bx - ax, by - ay
            n = math.hypot(dx, dy)
            if n < 1e-6:
                out.append(pts[k])
                continue
            out.append((pts[k][0] - dy / n * off, pts[k][1] + dx / n * off))
        return out

    def _roll(self, path, steer, per_m, wb, ms, obs, cfg):
        """path 를 Pure Pursuit 로 horizon 만큼 따라가 본다 (조향 한계, 조향 속도 per_m 반영).

        반환: (첫 조향 명령, 끝 위치, 가장 작은 여유[m]). 여유 < 0 이면 콘에 닿음.
        """
        x = y = yaw = 0.0
        ds = 0.05
        la = cfg['lookahead']
        hl, hw = 0.5 * cfg['car_length'], 0.5 * cfg['car_width']
        reach = math.hypot(hl, hw)
        margin = 9.0
        first = None
        for _ in range(max(1, int(round(cfg['horizon'] / ds)))):
            sp, _ = self._project(path, x, y)
            tx, ty = self._point_at(path, sp + la)
            c, s = math.cos(yaw), math.sin(yaw)
            lx = c * (tx - x) + s * (ty - y)
            ly = -s * (tx - x) + c * (ty - y)
            target = clamp(math.atan(wb * pure_pursuit_curvature(lx, ly)), -ms, ms)
            if first is None:
                first = target
            if per_m > 0.0:
                d = per_m * ds
                steer += clamp(target - steer, -d, d)
            else:
                steer = target
            yaw += ds * math.tan(steer) / wb
            x += ds * math.cos(yaw)
            y += ds * math.sin(yaw)
            c, s = math.cos(yaw), math.sin(yaw)
            for ox, oy, rr in obs:
                dx, dy = ox - x, oy - y
                if abs(dx) > reach + rr + 0.1 or abs(dy) > reach + rr + 0.1:
                    continue
                qx = c * dx + s * dy
                qy = -s * dx + c * dy
                d = math.hypot(max(abs(qx) - hl, 0.0), max(abs(qy) - hw, 0.0)) - rr
                if d < margin:
                    margin = d
        return first, (x, y), margin

    @staticmethod
    def _point_at(path, s_target):
        """꺾은선 위 길이 s_target 지점 (끝을 넘으면 마지막 선분을 연장)."""
        acc = 0.0
        for (ax, ay), (bx, by) in zip(path, path[1:]):
            seg = math.hypot(bx - ax, by - ay)
            if seg < 1e-6:
                continue
            if acc + seg >= s_target:
                t = (s_target - acc) / seg
                return ax + t * (bx - ax), ay + t * (by - ay)
            acc += seg
        if len(path) >= 2:
            (ax, ay), (bx, by) = path[-2], path[-1]
            seg = math.hypot(bx - ax, by - ay)
            if seg > 1e-6:
                t = (s_target - acc) / seg
                return bx + t * (bx - ax), by + t * (by - ay)
        return path[-1]

    @staticmethod
    def _project(path, px, py):
        """꺾은선 path 위로 (px, py) 를 내렸을 때 (따라간 길이, 떨어진 거리)."""
        best_d, best_s, acc = None, 0.0, 0.0
        for (ax, ay), (bx, by) in zip(path, path[1:]):
            vx, vy = bx - ax, by - ay
            seg = math.hypot(vx, vy)
            if seg < 1e-6:
                continue
            t = clamp(((px - ax) * vx + (py - ay) * vy) / (seg * seg), 0.0, 1.0)
            qx, qy = ax + t * vx, ay + t * vy
            d = math.hypot(px - qx, py - qy)
            if best_d is None or d < best_d:
                best_d, best_s = d, acc + t * seg
            acc += seg
        if best_d is None:
            return 0.0, math.hypot(px, py)
        return best_s, best_d
