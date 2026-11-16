"""(11/16) 차선 팀원 흰 점선 노드 값을 모아 '차가 실제로 몇 번 차선에 있나' 를 따진다.

토픽 /perception/white_status (std_msgs/String, JSON):
  {"t": 영상 시각, "left": {"seen", "offset_m", "heading_rad", "conf", "n_dash"}, "right": {...}}
  - left / right = 차 왼쪽 / 오른쪽에서 대표 점선 한 줄. offset_m 은 둘 다 양수 (차 가운데에서 옆으로 몇 m).
  - heading_rad = 차 앞 방향 기준 점선 방향 (왼쪽 +). seen=false 면 값 없음. 못 봐도 메시지는 옴.
  - 한쪽을 못 봤다고 그쪽에 점선이 없는 건 아님 (가림·미검출) -> 여러 번 모아서, 그것도 한쪽만으로는 확정 안 함.

3차선 (차선 폭 w): 흰 점선은 가운데 차선 양옆 d = +w/2, -w/2 두 줄뿐 (바깥은 노란 실선).
  가운데 차선: 양쪽에 w/2 씩 (왼쪽 + 오른쪽 = w).   왼쪽 차선: 오른쪽에만 (w/2, 3w/2).   오른쪽 차선: 왼쪽에만.
관측 하나마다 '차의 진짜 옆 위치' 후보를 구해 (양쪽 다 보면 1개, 한쪽만 보면 2개), 판단이 쓰는 기준 좌표의 차 위치와 비교한다.
  어긋난 양 = k x w + 조금 (k = -1 / 0 / +1). k = 0 이면 기준 좌표가 맞음, -1 이면 차가 실제로는 한 차선 더 왼쪽,
  +1 이면 한 차선 더 오른쪽. votes() 가 k 마다 '이 k 로 설명되는 관측 수' 와 '이 k 로만 설명되는 관측 수' 를 센다.
쓰는 쪽: three_lane.ThreeLanePlanner._ident_step.
"""

import collections
import json
import math


class WhiteLines(object):

    KS = (-1, 0, 1)

    def __init__(self, node):
        self.node = node
        self.hist = collections.deque(maxlen=400)   # (받은 시각, (x, y, yaw) 또는 None, 왼쪽 거리 또는 None, 오른쪽 거리 또는 None)
        self.last_rx = -1e9
        self.dropped_old = 0

    @property
    def cfg(self):
        return self.node.white

    def alive(self, now):
        return (now - self.last_rx) <= self.cfg['alive_sec']

    # ------------------------------------------------------------------
    def on_message(self, text, now, pose):
        """JSON 글 하나. 쓸 수 있으면 (왼쪽 거리 또는 None, 오른쪽 거리 또는 None), 못 읽거나 오래됐으면 None."""
        cfg = self.cfg
        try:
            data = json.loads(text)
        except (ValueError, TypeError):
            return None
        if not isinstance(data, dict):
            return None
        self.last_rx = now
        t = data.get('t')
        if isinstance(t, (int, float)) and not isinstance(t, bool):
            age = now - float(t)
            if cfg['max_age'] < age < 60.0:          # 오래된 영상. (60초 넘게 다르면 시계가 다른 것 - 받은 시각만 믿음)
                self.dropped_old += 1
                return None
        expect = self._expected_heading(pose)
        if expect is False:
            return None                               # 3차선 방향이 아님 (코너 도는 중 등) - 안 모음
        left = self._side(data.get('left'), expect)
        right = self._side(data.get('right'), expect)
        self.hist.append((now, pose, left, right))
        return left, right

    def _expected_heading(self, pose):
        """차 앞 방향 기준 3차선 점선 방향 [rad]. 모르면 None, 3차선 방향에서 많이 벗어났으면 False."""
        node = self.node
        if pose is None:
            return None
        if not node.avoid_static['three_lane_axis_use']:
            return None
        known = node.known_lane_axis(pose[2])
        if known is None or abs(known[1]) > math.radians(self.cfg['axis_tol_deg']):
            return False
        return -known[1]

    def _side(self, sd, expect):
        cfg = self.cfg
        if not isinstance(sd, dict) or not sd.get('seen'):
            return None
        off = sd.get('offset_m')
        if not isinstance(off, (int, float)) or isinstance(off, bool) or off != off:
            return None
        off = abs(float(off))
        if not (cfg['min_offset'] <= off <= cfg['max_offset']):
            return None
        conf = sd.get('conf')
        if isinstance(conf, (int, float)) and float(conf) < cfg['min_conf']:
            return None
        nd = sd.get('n_dash')
        if isinstance(nd, (int, float)) and nd < cfg['min_dash']:
            return None
        hd = sd.get('heading_rad')
        if isinstance(hd, (int, float)) and hd == hd:
            ref = 0.0 if expect is None else expect
            if abs(math.atan2(math.sin(hd - ref), math.cos(hd - ref))) > math.radians(cfg['heading_tol_deg']):
                return None
        return off

    # ------------------------------------------------------------------
    def votes(self, now, pose, frame, width):
        """최근 관측을 기준 좌표(frame)에 놓고 센다. 쓸 관측이 없으면 None.
        {'n', 'supp': {k: 수}, 'excl': {k: 수}, 'fine': {k: 양쪽 다 본 관측의 잔차 중간값 또는 None}, 'kinds': (양쪽, 왼쪽만, 오른쪽만)}"""
        cfg = self.cfg
        half = 0.5 * width
        tol = cfg['lane_tol']
        supp = dict((k, 0) for k in self.KS)
        excl = dict((k, 0) for k in self.KS)
        fine = dict((k, []) for k in self.KS)
        kinds = [0, 0, 0]
        n = 0
        for t, p, left, right in list(self.hist):
            if now - t > cfg['window_sec'] or p is None:
                continue
            if math.hypot(p[0] - pose[0], p[1] - pose[1]) > cfg['window_dist']:
                continue
            if left is not None and right is not None:
                if abs(left + right - width) > cfg['pair_tol']:
                    continue                          # 두 줄 사이가 차선 폭이 아님 - 잘못 본 것
                cands, kind = [(0.5 * (right - left), True)], 0
            elif left is not None:
                cands, kind = [(half - left, False), (-half - left, False)], 1
            elif right is not None:
                cands, kind = [(half + right, False), (-half + right, False)], 2
            else:
                continue
            d_obs = frame.odom_to_frame(p[0], p[1])[1]
            ks = {}
            for d_true, both in cands:
                if abs(d_true) > width + tol:
                    continue                          # 3차선 밖 - 있을 수 없는 후보
                res = d_obs - d_true
                k = int(math.floor(res / width + 0.5))
                if abs(k) > 1 or abs(res - k * width) > tol:
                    continue
                ks[k] = (res - k * width, both)
            if not ks:
                continue
            n += 1
            kinds[kind] += 1
            for k, (f, both) in ks.items():
                supp[k] += 1
                if both:
                    fine[k].append(f)
            if len(ks) == 1:
                excl[list(ks)[0]] += 1
        if n == 0:
            return None
        med = {}
        for k in self.KS:
            vals = sorted(fine[k])
            med[k] = vals[len(vals) // 2] if vals else None
        return {'n': n, 'supp': supp, 'excl': excl, 'fine': med, 'kinds': tuple(kinds)}
