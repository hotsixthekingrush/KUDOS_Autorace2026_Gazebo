"""차선 검출과 lane_error 계산 (ROS 의존 없음).

limo_ros2_application/detect_line.py의 선 추적 로직을 그대로 옮기고,
옆 도로 선 오검출 가드를 더했다. camera_perception_node가 이 클래스를 감싼다.
좌표계: base_link 지면, x 전방 / y 왼쪽 +.
"""
import numpy as np
import cv2

DEFAULTS = {
    # 색 임계값 (HLS)
    'lane_h_l': 0, 'lane_l_l': 90, 'lane_s_l': 100,
    'lane_h_h': 60, 'lane_l_h': 220, 'lane_s_h': 255,
    # 선 추적 (영상 px)
    'side': 'both',           # both | left | right
    'center_x': 320,          # 처음 시작할 때만 좌/우 선을 가르는 기준
    'y_top': 320,
    'y_bot': 470,
    'n_rows': 8,
    'horizon_y': 260,
    'lane_w_bottom': 560,
    'smooth': 0.7,            # distance_y 필터: 클수록 빠르게 반응
    'max_step': 100.0,        # 프레임당 distance_y 최대 변화
    'track_tol': 90,          # 위/아래 줄에서 같은 선으로 인정하는 거리(px)
    'track_predict': False,   # True: 아래 두 줄의 기울기로 다음 줄 위치 예측
    'lost_hold': 5,
    # lane_error (지면 좌표)
    'lane_width_m': 0.55,     # 선 중심 간 차선 폭 (실차 미확인). 시뮬은 0.52
    'reference_distance_m': 0.5,
    'cam_height_m': 0.21,
    'cam_x_m': 0.10,
    'cam_pitch_rad': 0.0,
    'focal_px': 381.0,
    # 옆 도로 선 가드
    'guard_enable': True,
    'guard_width_tol_m': 0.10,   # 두 선 간격이 lane_width_m에서 이만큼 넘게 벗어나면 잘못 잡은 것
    'guard_jump_m': 0.10,        # 직전 정상값 대비 lateral이 한 번에 이만큼 넘게 바뀌면 의심
    'guard_memory_sec': 1.0,     # 직전 정상값을 이 시간까지만 비교 기준으로 씀
}


class LaneCore:
    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        if params:
            self.p.update(params)
        self.wrow, self.prevL, self.prevR = {}, {}, {}
        self.gap_f = 0.0
        self.lost = 0
        # 가드 상태: 마지막으로 내보낸 정상(valid) lateral과 그 시각
        self.ref_lat = None
        self.ref_t = None

    # ------------------------------------------------------------------
    # 선 추적 (detect_line.py와 동일)
    # ------------------------------------------------------------------
    @staticmethod
    def nearest(cands, p, tol, skip=None):
        best, bd = None, None
        for m in cands:
            if skip is not None and abs(m - skip) < 1e-6:
                continue
            dd = abs(m - p)
            if dd <= tol and (bd is None or dd < bd):
                best, bd = m, dd
        return best

    def find_row(self, means, cx0, side, exp, tol, pl, pr):
        """한 줄에서 왼쪽선 L, 오른쪽선 R 찾기. 화면 가운데를 넘어가도 같은 선을 유지."""
        L = R = None
        if pl is None and pr is None:
            left = [m for m in means if m < cx0]
            right = [m for m in means if m >= cx0]
            L = max(left) if left else None
            R = min(right) if right else None
        else:
            if pl is not None:
                L = self.nearest(means, pl, tol)
            if pr is not None:
                R = self.nearest(means, pr, tol, skip=L)
            if L is None and pl is None and R is not None:
                c = [m for m in means if m < R - 0.4 * exp]
                L = max(c) if c else None
            if R is None and pr is None and L is not None:
                c = [m for m in means if m > L + 0.4 * exp]
                R = min(c) if c else None
            if L is not None and R is not None and L >= R:
                R = None
        if side == 'right':
            L = None
        elif side == 'left':
            R = None
        return L, R

    def track(self, img, want_debug=False):
        """반환: gap(px, 필터 후), newL, newR, pairs, debug 영상(또는 None).

        pairs: 한 줄에서 L, R이 둘 다 잡힌 경우의 (y, L, R). 픽셀 폭 검사로 한쪽을 버리기 전 값.
        """
        g = self.p.get
        h, w = img.shape[:2]
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hls,
                           np.array([g('lane_h_l'), g('lane_l_l'), g('lane_s_l')]),
                           np.array([g('lane_h_h'), g('lane_l_h'), g('lane_s_h')]))
        cx0 = int(g('center_x'))
        side = g('side')
        hz = g('horizon_y')
        tol = g('track_tol')
        ys = np.linspace(g('y_bot'), g('y_top'), int(g('n_rows'))).astype(int)

        dbg = None
        if want_debug:
            dbg = img.copy()
            dbg[mask > 0] = (0, 0, 255)
            cv2.line(dbg, (cx0, 0), (cx0, h), (0, 255, 0), 2)

        centers, weights, modes = [], [], ''
        newL, newR, pairs = {}, {}, []
        lastL = lastR = None
        l2 = r2 = None
        predict = bool(g('track_predict'))
        for y in ys:
            y = int(y)
            band = mask[max(y - 2, 0):y + 3, :]
            cols = np.nonzero(band.any(axis=0))[0]
            if len(cols) == 0:
                continue
            groups = np.split(cols, np.where(np.diff(cols) > 8)[0] + 1)
            means = [gp.mean() for gp in groups]

            exp = self.wrow.get(y, g('lane_w_bottom') * max(y - hz, 1) / max(h - hz, 1))
            pl = lastL if lastL is not None else self.prevL.get(y)
            pr = lastR if lastR is not None else self.prevR.get(y)
            if predict:
                if lastL is not None and l2 is not None:
                    pl = 2 * lastL - l2
                if lastR is not None and r2 is not None:
                    pr = 2 * lastR - r2
            L, R = self.find_row(means, cx0, side, exp, tol, pl, pr)

            c = None
            if L is not None and R is not None:
                pairs.append((y, L, R))
                wd = R - L
                if 0.6 * exp <= wd <= 1.4 * exp:
                    c = (L + R) / 2.0
                    modes += 'D'
                    self.wrow[y] = 0.8 * exp + 0.2 * wd
                else:
                    dl = abs(L - pl) if pl is not None else 1e9
                    dr = abs(R - pr) if pr is not None else 1e9
                    if dl <= dr:
                        c = L + exp / 2.0
                        R = None
                        modes += 'L'
                    else:
                        c = R - exp / 2.0
                        L = None
                        modes += 'R'
            elif L is not None:
                c = L + exp / 2.0
                modes += 'L'
            elif R is not None:
                c = R - exp / 2.0
                modes += 'R'
            if c is None:
                continue

            if L is not None:
                newL[y] = L
                l2, lastL = lastL, L
            if R is not None:
                newR[y] = R
                r2, lastR = lastR, R
            centers.append(c)
            weights.append(1.0 + (g('y_bot') - y) / max(g('y_bot') - g('y_top'), 1))
            if dbg is not None:
                cv2.line(dbg, (0, y), (w, y), (90, 90, 90), 1)
                if L is not None:
                    cv2.circle(dbg, (int(L), y), 4, (255, 255, 0), -1)
                if R is not None:
                    cv2.circle(dbg, (int(R), y), 4, (0, 165, 255), -1)
                cv2.circle(dbg, (int(c), y), 4, (255, 0, 0), -1)

        if centers:
            self.prevL, self.prevR = newL, newR
            tgt = float(np.average(centers, weights=weights))
            gap = cx0 - tgt
            step = g('smooth') * (gap - self.gap_f)
            ms = g('max_step')
            self.gap_f += float(np.clip(step, -ms, ms))
            self.lost = 0
            text, color = '%s  gap %+d' % (modes, round(self.gap_f)), (0, 255, 0)
        else:
            self.lost += 1
            if self.lost > g('lost_hold'):
                self.gap_f = 0.0
                self.prevL, self.prevR = {}, {}
            text, color = 'NO LANE (lost %d)' % self.lost, (0, 0, 255)
        if dbg is not None:
            cv2.putText(dbg, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return int(round(self.gap_f)), newL, newR, pairs, dbg

    # ------------------------------------------------------------------
    # 지면 좌표 lane_error
    # ------------------------------------------------------------------
    def pix_to_base(self, u, v, w, h):
        """영상 (u, v) -> base_link 지면 좌표 (x 전방, y 왼쪽). 지평선 위면 None."""
        p = self.p
        f, hc, th = p['focal_px'], p['cam_height_m'], p['cam_pitch_rad']
        xc, yc = (u - w / 2.0) / f, (v - h / 2.0) / f
        down = yc * np.cos(th) + np.sin(th)
        fwd = np.cos(th) - yc * np.sin(th)
        if down <= 1e-6:
            return None
        t = hc / down
        return np.array([t * fwd + p['cam_x_m'], -t * xc])

    def lane_error(self, newL, newR, w, h):
        """반환: lateral, heading, valid, confidence, n_both(두 선 줄 수), n_one(한쪽 선으로 추정한 줄 수)."""
        half = self.p['lane_width_m'] / 2.0
        n_rows = max(int(self.p['n_rows']), 1)
        G = {}
        for side, pts in (('L', newL), ('R', newR)):
            G[side] = {y: self.pix_to_base(u, y, w, h) for y, u in pts.items()}
            G[side] = {y: q for y, q in G[side].items() if q is not None}
        C, W = [], []
        n_both = n_one = 0
        for y in sorted(set(G['L']) | set(G['R']), reverse=True):
            if y in G['L'] and y in G['R']:
                C.append((G['L'][y] + G['R'][y]) / 2.0)
                W.append(1.0)
                n_both += 1
                continue
            side = 'L' if y in G['L'] else 'R'
            ys = sorted(G[side], reverse=True)
            k = ys.index(y)
            a = G[side][ys[k - 1]] if k > 0 else G[side][y]
            b = G[side][ys[k + 1]] if k + 1 < len(ys) else G[side][y]
            t = b - a
            nt = np.hypot(*t)
            t = np.array([1.0, 0.0]) if nt < 1e-6 else t / nt
            left = np.array([-t[1], t[0]])
            C.append(G[side][y] + (-left if side == 'L' else left) * half)
            W.append(0.5)
            n_one += 1
        if len(C) < 2:
            return 0.0, 0.0, False, 0.0, n_both, n_one
        C, W = np.array(C), np.array(W)
        span = C[:, 0].max() - C[:, 0].min()
        if span < 0.10:
            return 0.0, 0.0, False, 0.0, n_both, n_one
        A = np.c_[np.ones(len(C)), C[:, 0]]
        sw = np.sqrt(W)
        coef = np.linalg.lstsq(A * sw[:, None], C[:, 1] * sw, rcond=None)[0]
        rms = float(np.sqrt(np.average((A @ coef - C[:, 1]) ** 2, weights=W)))
        heading = float(np.arctan(coef[1]))
        x_ref = float(self.p['reference_distance_m'])
        lateral = float((coef[0] + coef[1] * x_ref) / np.hypot(1.0, coef[1]))
        coverage = (n_both + 0.5 * n_one) / n_rows
        conf = coverage * np.exp(-(rms / 0.03) ** 2) * min(span / 0.4, 1.0)
        if len(C) >= 4:
            near = np.argsort(C[:, 0])[:3]
            bn = np.linalg.lstsq(A[near] * sw[near, None], C[near, 1] * sw[near],
                                 rcond=None)[0][1]
            conf *= np.exp(-(np.degrees(abs(np.arctan(bn) - np.arctan(coef[1]))) / 10.0) ** 2)
        return lateral, heading, True, float(np.clip(conf, 0.0, 1.0)), n_both, n_one

    def pair_width_error(self, pairs, w, h, heading):
        """L, R이 같이 잡힌 줄들의 차선 폭 - lane_width_m 중 절댓값 최대. 없으면 None.

        영상 가로줄은 차선을 비스듬히 자르므로 base_link y 간격에 cos(heading)을 곱해 수직 폭으로 바꾼다.
        """
        c = abs(np.cos(heading))
        worst = None
        for y, L, R in pairs:
            a, b = self.pix_to_base(L, y, w, h), self.pix_to_base(R, y, w, h)
            if a is None or b is None:
                continue
            e = abs(abs(a[1] - b[1]) * c - self.p['lane_width_m'])
            worst = e if worst is None else max(worst, e)
        return worst

    def guard(self, lat, valid, n_one, width_err, t):
        """옆 도로 선 오검출 가드. 반환: (valid, 사유 문자열 또는 '').

        애매한 프레임 = 한 줄에서 잡힌 두 선의 지면 간격이 lane_width_m과 크게 다르고
        (옆 도로 선이 섞였다는 뜻), 그 때문에 한쪽 선만으로 중심을 추정한 줄이 있는 경우.
        이때 남긴 선이 자기 차선 선인지 옆 도로 선인지는 한 프레임만으로는 대칭이라 알 수 없으므로,
        직전 정상값과 이어질 때만 믿는다. 직전 값이 없거나 오래됐으면 valid=false.
        기준값을 언제 갱신하는지는 process() 참고.
        """
        if not valid or not self.p['guard_enable']:
            return valid, ''
        ambiguous = (n_one > 0 and width_err is not None
                     and width_err > self.p['guard_width_tol_m'])
        if not ambiguous:
            return True, ''
        fresh = self.ref_t is not None and 0.0 <= t - self.ref_t <= self.p['guard_memory_sec']
        if not fresh:
            return False, 'ambiguous_no_history'
        if abs(lat - self.ref_lat) > self.p['guard_jump_m']:
            return False, 'ambiguous_jump'
        return True, ''

    def process(self, img, t, want_debug=False):
        """한 프레임 처리. t: 촬영 시각 [s]. 반환 dict."""
        h, w = img.shape[:2]
        gap, newL, newR, pairs, dbg = self.track(img, want_debug)
        lat, head, valid, conf, n_both, n_one = self.lane_error(newL, newR, w, h)
        werr = self.pair_width_error(pairs, w, h, head)
        ok, why = self.guard(lat, valid, n_one, werr, t)
        if ok:
            # 가드 기준값 갱신. 한쪽 선만 보인 프레임은 옆 도로 선 하나만 잡은 것일 수도 있으므로
            # 새 기준을 세우지 못하고, 이미 있는 기준을 이어 갈 때만 쓴다.
            fresh = self.ref_t is not None and 0.0 <= t - self.ref_t <= self.p['guard_memory_sec']
            if n_both > 0 or (fresh and abs(lat - self.ref_lat) <= self.p['guard_jump_m']):
                self.ref_lat, self.ref_t = lat, t
        return dict(gap=gap, lateral=lat, heading=head, valid=ok, raw_valid=valid,
                    confidence=conf, single=n_one > 0, guard=why, width_err=werr,
                    n_one=n_one, debug=dbg)
