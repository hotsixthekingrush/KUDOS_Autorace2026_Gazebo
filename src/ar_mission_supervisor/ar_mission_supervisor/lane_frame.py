"""미션용 주행 방향 좌표계 (odom 기반) + 라이다 장애물 지도.

미션에 들어가는 순간의 차선 방향을 s 축(앞 +), 그 왼쪽을 d 축(왼쪽 +)으로 잡는다.
원점은 그 순간 차가 따라가던 차선의 중앙이다 (카메라 차선 오차로 보정, 없으면 차 위치).
odom 은 몇 m 정도의 짧은 구간에서는 오차가 작아서, 차선 변경·주차처럼
카메라 차선이 흔들리거나 다른 차선에 다시 붙을 수 있는 동작을 이 좌표로 계산한다.
"""

import math

from ar_mission_supervisor.controllers import clamp, steering_to_omega
from ar_mission_supervisor.geometry import normalize_angle


class LaneFrame(object):

    def __init__(self):
        self.origin = None          # (x, y, yaw) odom 좌표

    @property
    def ready(self):
        return self.origin is not None

    def begin(self, pose, lane=None):
        """pose = (x, y, yaw) odom. lane = node.lane_error() 값(없으면 None).

        lane 의 차선 중앙 점은 차 앞 x_ref(lane[3], 없으면 0), 옆 lateral 위치.
        그 점에서 차선 방향으로 거슬러 올라가 차 옆(s = 0)에 원점을 둔다 (직선 차선이면 정확).
        """
        x, y, yaw = pose
        ox, oy, oyaw = x, y, yaw
        if lane is not None:
            lateral, heading = lane[0], lane[1]
            x_ref = lane[3] if len(lane) > 3 else 0.0
            # lateral: 로봇 기준 차선 중앙의 위치 (왼쪽 +), heading: 차선 방향 - 차 방향
            oyaw = yaw + heading
            px = x + math.cos(yaw) * x_ref - math.sin(yaw) * lateral
            py = y + math.sin(yaw) * x_ref + math.cos(yaw) * lateral
            back = (px - x) * math.cos(oyaw) + (py - y) * math.sin(oyaw)
            ox = px - back * math.cos(oyaw)
            oy = py - back * math.sin(oyaw)
        self.origin = (ox, oy, oyaw)

    def odom_to_frame(self, x, y):
        ox, oy, oyaw = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (c * dx + s * dy, -s * dx + c * dy)

    def frame_to_odom(self, s_, d):
        ox, oy, oyaw = self.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (ox + c * s_ - s * d, oy + s * s_ + c * d)

    def car(self, pose):
        """차(base_link) 위치를 (s, d, psi) 로. psi = 차 방향 - 기준 방향."""
        x, y, yaw = pose
        s_, d = self.odom_to_frame(x, y)
        return s_, d, normalize_angle(yaw - self.origin[2])

    def base_to_frame(self, pose, bx, by):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return self.odom_to_frame(x + c * bx - s * by, y + s * bx + c * by)

    def frame_to_base(self, pose, s_, d):
        x, y, yaw = pose
        gx, gy = self.frame_to_odom(s_, d)
        dx, dy = gx - x, gy - y
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy, -s * dx + c * dy)


class ObstacleMap(object):
    """라이다 장애물을 기준 좌표(s, d)에 쌓아 둔다.

    LIMO 는 서로 가리거나 시야(3 m) 밖으로 나가므로, 한 번 본 것을 기억해 두고
    같은 자리(merge 거리 안)에서 다시 보이면 위치를 갱신한다.
      average=False: 부드럽게 (새 값 쪽으로 alpha 만큼). average=True: 지금까지 본 것의 평균 (안 움직이는 LIMO).
      ghost=(길이, 폭, 최소 횟수): 이미 그 횟수 이상 본 장애물 박스(길이 x 폭)와 겹치는 새 검출은 버린다.
        LIMO 끼리는 겹칠 수 없어서. (11/04 녹화 static_run2: 라이다가 옆 LIMO 뒷면을 보고 중심을 반대쪽에 찍은 유령)
    같은 라이다 메시지는 한 번만 반영한다 (제어 20 Hz, 라이다 약 6 Hz 라 예전엔 한 메시지를 3번쯤 반영했음 - 11/04).
    """

    def __init__(self, merge=0.2, alpha=0.3, average=False, ghost=None):
        self.merge = merge
        self.alpha = alpha
        self.average = average
        self.ghost = ghost
        self.items = []
        self._last = None

    def clear(self):
        self.items = []
        self._last = None

    def _ghost(self, s_, d):
        if not self.ghost:
            return False
        length, width, hits = self.ghost
        for it in self.items:
            if it['n'] >= hits and abs(it['s'] - s_) < length and abs(it['d'] - d) < width:
                return True
        return False

    def update(self, frame, pose, obstacles, now, accept=None,
               min_radius=0.0, max_radius=1.0):
        if obstacles is not None and obstacles is self._last:
            return                          # 이미 반영한 라이다 메시지
        self._last = obstacles
        for o in obstacles or ():
            r = float(o.radius)
            if r < min_radius or r > max_radius:
                continue
            s_, d = frame.base_to_frame(pose, float(o.x), float(o.y))
            if accept is not None and not accept(s_, d):
                continue
            best, best_dist = None, self.merge
            for it in self.items:
                dist = math.hypot(it['s'] - s_, it['d'] - d)
                if dist <= best_dist:
                    best, best_dist = it, dist
            if best is None:
                if self._ghost(s_, d):
                    continue
                self.items.append({'s': s_, 'd': d, 'r': r, 'n': 1, 'last': now})
            else:
                a = 1.0 / (best['n'] + 1) if self.average else self.alpha
                best['s'] += a * (s_ - best['s'])
                best['d'] += a * (d - best['d'])
                best['r'] += a * (r - best['r'])
                best['n'] += 1
                best['last'] = now

    def prune(self, s_min):
        self.items = [it for it in self.items if it['s'] >= s_min]

    def confirmed(self, min_hits=2):
        return [it for it in self.items if it['n'] >= min_hits]


def pursue(node, frame, pose, s_target, d_target, v):
    """기준 좌표의 목표점을 향하는 Pure Pursuit. 반환: (omega, steer)."""
    bx, by = frame.frame_to_base(pose, s_target, d_target)
    l2 = bx * bx + by * by
    kappa = 0.0 if l2 < 1e-6 else 2.0 * by / l2
    steer = clamp(math.atan(kappa * node.wheel_base), -node.max_steer, node.max_steer)
    return steering_to_omega(v, steer, node.wheel_base), steer
