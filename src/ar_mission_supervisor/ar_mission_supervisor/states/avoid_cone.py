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
odom 이 없거나 게이트를 못 만들면 예전 방식(한쪽 콘 오프셋 / Follow the Gap)으로 간다.
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

    def on_enter(self):
        self.miss = 0
        self.entered_at = self.node.clock_sec()
        self.last_steer = 0.0
        self.memory = []
        self.steer_model = 0.0
        self.last_time = None
        self.warned_at = None

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
                # 라바콘이 안 보임 - 구간 종료로 판단 (몇 회 유예)
                self.miss += 1
                if self.miss >= node.perception['exit_miss_count']:
                    node.get_logger().info('[CONE] 라바콘 구간 종료')
                    return 'done'
                self._publish(cfg['speed'], self.last_steer, now)
                self.sleep()
                continue

            self.miss = 0

            if cfg['use_rollout']:
                pose = node.odom_pose()
                gates = self._gates(cones, cfg)
                if pose is not None and gates:
                    self._remember(pose, cfg)
                    self._drive_rollout(pose, gates, cfg, now)
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
            width = distance(l.x, l.y, r.x, r.y) - (l.radius + r.radius)
            if width < cfg['min_gate_width']:
                break
            gates.append((0.5 * (l.x + r.x), 0.5 * (l.y + r.y)))
        return gates

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
