"""(11/18 사용자) 주차 칸 격자 찾기. 주최측 안내 (11/17): 2행 x 3열 6칸 중 한 칸만 비우고 나머지는 LIMO 로 막음,
빈칸 위치는 경기 중 바뀔 수 있음 -> 1행 2열 고정이 아니라 어느 칸이 비었든 찾아야 함.

라이다 원본(/scan) 점을 주차 기준 좌표(LaneFrame: s 진행, d 왼쪽 +, 차선 가운데 0)에 쌓고, '6칸 중 1칸만 빈 격자' 가설로 빈칸을 고른다.
도면 (2026Track): 차선 폭 0.5, 선 0.02, 1행 d -0.27 ~ -0.87, 2행 d -0.89 ~ -1.49, 열 간격 0.52 (칸 0.5 + 선 0.02).
쌓기: 오른쪽 빔마다 맞은 곳 = 맞음 +1, 빔이 지나간 곳 = 지나감 +1 (2 cm 칸). 안 돌아온 빔은 안 셈.
  2 cm 칸은 맞음 >= 2 이고 맞음 >= 지나감 x 0.5 면 '찬 점', 지나감 >= 3 이고 맞음 0 이면 '빈 점'.
가설 = (격자 시작 s G, 빈칸 = (행, 열)). 점수:
  LIMO 가 있어야 할 칸 : 찬 점이 많을수록 +, 찬 점 없이 칸 가운데가 비어 보이면 -
  빈칸이어야 할 칸     : 칸 가운데 넓은 곳(±0.15 x ±0.22)에 빔이 지나간 비율이 높을수록 +, 찬 점이 있으면 -
  격자 앞·뒤 바깥, 칸 사이 선 근처 : 찬 점이 있으면 - (얇은 기둥 정도는 봐줌)
G 는 0.01 m 간격으로 다 해보고, 가장 좋은 가설과 '다른 칸을 고르는 가설' 의 점수 차가 decide_margin 넘고,
  목표 칸이 비어 보이고, 옆 칸 1행 LIMO 와 2행 1열 LIMO(격자 시작 표시)를 봤고, 보이는 LIMO 가 다 칸 가운데 근처면,
  stable 번 연속 같을 때 확정. 확정 뒤에도 회전 시작 전까지 가까이서 본 LIMO 로 위치를 다듬는다.
1행 3칸이 다 찬 것으로 확정되면 kind='row1_full' (빈칸이 2행 - 차선에서 못 들어감. 주최측 안내상 없어야 하는 경우).
시뮬 (scratchpad parksim, 실차 녹화 스펙): 1,740번 중 잘못 고른 칸 0, 결정 못 함 9, 칸 가운데 오차 최대 12 cm (여유 14 cm).
"""

import math

import numpy as np

LINE, CELL_S, CELL_D, PITCH_S = 0.02, 0.50, 0.60, 0.52
ROW_TOP = (-0.27, -0.89)


class GridFinder(object):

    def __init__(self, s_lo, s_hi, res=0.02, d_lo=-1.70, d_hi=-0.10, cfg=None):
        c = dict(max_range=2.6, right_from_deg=-110.0, right_to_deg=-3.0, margin=0.03, dead=4, occ_full=8.0,
                 core_s=0.15, core_d=0.22, free_lo=0.3, free_hi=0.7, need_seen=0.6, w_occ_free=1.0, w_empty_occ=2.0, w_empty_free=1.0, w_out=1.5,
                 decide_margin=1.0, stable=3, target_free=0.7, near_min=6, limo_half_w=0.11, near_range=1.0, line_band=0.05, w_line=1.0, depth_fit=True, front_min=8, limo_len=0.322, align_tol=0.13)
        if cfg:
            c.update(cfg)
        self.cfg = c
        self.res = res
        self.s_lo, self.d_lo = s_lo, d_lo
        self.ns = int(math.ceil((s_hi - s_lo) / res))
        self.nd = int(math.ceil((d_hi - d_lo) / res))
        self.H = np.zeros((self.ns, self.nd), np.float32)
        self.Hn = np.zeros((self.ns, self.nd), np.float32)   # 가까이(near_range 안)에서 맞은 것만 - 확정 뒤 위치 다듬기용
        self.F = np.zeros((self.ns, self.nd), np.float32)
        self.hist = []
        self.decided = None
        self.best = None
        self.dd = 0.0
        self.dd_ok = False

    # ------------------------------------------------------------------
    def add_scan(self, lx, ly, lyaw, a0, ainc, ranges):
        c = self.cfg
        n = len(ranges)
        rel = a0 + ainc * np.arange(n)
        keep = (rel >= math.radians(c['right_from_deg'])) & (rel <= math.radians(c['right_to_deg']))
        r = np.asarray(ranges, float)[keep]
        ang = lyaw + rel[keep]
        ok = np.isfinite(r) & (r > 0.05)
        r, ang = r[ok], ang[ok]
        dx, dy = np.cos(ang), np.sin(ang)
        hit = r <= c['max_range']
        hx, hy = lx + r[hit] * dx[hit], ly + r[hit] * dy[hit]
        self._add(self.H, hx, hy)
        nr = r[hit] <= c['near_range']
        self._add(self.Hn, hx[nr], hy[nr])
        # 빔 지나간 곳 (맞은 곳 4 cm 앞까지, 너무 멀면 max_range 까지)
        tmax = np.minimum(r, c['max_range']) - 0.04
        steps = np.arange(0.04, c['max_range'], self.res)
        T = steps[None, :]
        m = T < tmax[:, None]
        fx = (lx + T * dx[:, None])[m]
        fy = (ly + T * dy[:, None])[m]
        self._add(self.F, fx, fy)

    def _add(self, A, x, y):
        i = np.floor((x - self.s_lo) / self.res).astype(int)
        j = np.floor((y - self.d_lo) / self.res).astype(int)
        ok = (i >= 0) & (i < self.ns) & (j >= 0) & (j < self.nd)
        np.add.at(A, (i[ok], j[ok]), 1.0)

    # ------------------------------------------------------------------
    def _integrals(self):
        occ = ((self.H >= 2) & (self.H >= 0.5 * self.F)).astype(np.float32)
        free = ((self.F >= 3) & (self.H == 0)).astype(np.float32)
        def integ(A):
            I = np.zeros((self.ns + 1, self.nd + 1), np.float64)
            I[1:, 1:] = A.cumsum(0).cumsum(1)
            return I
        return integ(occ), integ(free), occ

    def _box(self, I, s0, s1, d0, d1):
        """[s0, s1) x [d0, d1) 합. s0, s1 은 배열 가능."""
        i0 = np.clip(np.round((np.asarray(s0) - self.s_lo) / self.res).astype(int), 0, self.ns)
        i1 = np.clip(np.round((np.asarray(s1) - self.s_lo) / self.res).astype(int), 0, self.ns)
        j0 = int(np.clip(round((d0 - self.d_lo) / self.res), 0, self.nd))
        j1 = int(np.clip(round((d1 - self.d_lo) / self.res), 0, self.nd))
        return I[i1, j1] - I[i0, j1] - I[i1, j0] + I[i0, j0]

    def evaluate(self, G_lo, G_hi):
        """모든 (G, 빈칸) 점수. 반환 Gs, scores[6, nG], 칸별 (찬 점, 빈 점) [6, nG, 2]."""
        c = self.cfg
        Io, If, occ = self._integrals()
        Gs = np.arange(G_lo, G_hi, 0.01)
        if c['depth_fit']:
            dd = self._depth_shift(occ, G_lo, G_hi)
        else:
            dd, self.dd_ok = 0.0, True
        self.dd = dd
        RT = (ROW_TOP[0] + dd, ROW_TOP[1] + dd)
        m = c['margin']
        stats = np.zeros((6, len(Gs), 2))
        for row in (0, 1):
            top = RT[row]
            for col in (0, 1, 2):
                s0 = Gs + LINE + PITCH_S * col + m
                s1 = Gs + LINE + PITCH_S * col + CELL_S - m
                k = row * 3 + col
                stats[k, :, 0] = self._box(Io, s0, s1, top - CELL_D + m, top - m)
                # 빈 점 = 칸 가운데 넓은 곳(진행 ±core_s, 깊이 ±core_d)에서 빔이 지나간 2 cm 칸의 '비율'.
                # LIMO 가 있으면(±0.1 m 어긋나도) 몸통 + 그 뒤 그늘이 이곳 절반 넘게 가려서 비율이 낮음.
                # (예전: 좁은 가운데 칸의 '개수' -> LIMO 가 0.1 m 깊이 들어가 있으면 그 앞만 비어 보여 빈칸으로 잘못 봄)
                cs = Gs + LINE + PITCH_S * col + 0.5 * CELL_S
                cd = top - 0.5 * CELL_D
                area = (2 * c['core_s'] / self.res) * (2 * c['core_d'] / self.res)
                stats[k, :, 1] = self._box(If, cs - c['core_s'], cs + c['core_s'], cd - c['core_d'], cd + c['core_d']) / area
        dead, of = c['dead'], c['occ_full']
        a = np.clip((stats[:, :, 0] - dead) / of, 0, 1)
        f = np.clip((stats[:, :, 1] - c['free_lo']) / (c['free_hi'] - c['free_lo']), 0, 1)
        occ_score = a - c['w_occ_free'] * (1 - a) * f
        empty_score = -c['w_empty_occ'] * a + c['w_empty_free'] * f
        # 격자 바깥 (앞 0.45 m, 뒤 0.45 m) 1·2행 깊이
        out_pen = np.zeros(len(Gs))
        for s0, s1 in ((Gs - 0.45, Gs - 0.01), (Gs + 3 * PITCH_S + LINE + 0.01, Gs + 3 * PITCH_S + 0.47)):
            n = self._box(Io, s0, s1, RT[1] - CELL_D, RT[0] - 0.02)
            out_pen += c['w_out'] * np.clip((n - dead) / of, 0, 1)
        # 칸 사이 선 근처(±line_band)에 찬 점 = 격자가 LIMO 에 비해 어긋남 (LIMO 0.22 가 칸 0.5 안이라 맞으면 거의 없음)
        for k in range(4):
            sl = Gs + PITCH_S * k + 0.5 * LINE
            n = self._box(Io, sl - c['line_band'], sl + c['line_band'], RT[1] - CELL_D, RT[0] - 0.02)
            out_pen += c['w_line'] * np.clip((n - dead) / of, 0, 1)
        total_occ = occ_score.sum(0)
        scores = np.zeros((6, len(Gs)))
        for k in range(6):
            scores[k] = total_occ - occ_score[k] + empty_score[k] - out_pen
        return Gs, scores, stats, occ

    def _depth_shift(self, occ, G_lo, G_hi):
        """기준 좌표가 옆으로 어긋난 양 (기준 방향이 틀어지면 칸 쪽에서 옆으로 밀려 보임 - 시뮬 4도면 0.2 m).
        진행 방향 2 cm 줄마다 차선 쪽에서 처음 찬 점(-0.10 ~ -0.70 m) = 대개 1행 LIMO 앞면. 그 75% 값 - 도면상 앞면(-0.41)."""
        c = self.cfg
        i0 = max(0, int((G_lo - self.s_lo) / self.res))
        i1 = min(self.ns, int((G_hi + 1.6 - self.s_lo) / self.res))
        j0 = int(round((-0.70 - self.d_lo) / self.res))
        j1 = int(round((-0.10 - self.d_lo) / self.res))
        blk = occ[i0:i1, j0:j1]
        fronts = []
        for i in range(blk.shape[0]):
            jj = np.where(blk[i] > 0)[0]
            if len(jj):
                fronts.append(self.d_lo + (j0 + jj[-1] + 0.5) * self.res)
        self.dd_ok = len(fronts) >= c['front_min']
        if not self.dd_ok:
            return 0.0
        front_expect = ROW_TOP[0] - 0.5 * CELL_D + 0.5 * c['limo_len']
        # 중간값이 아니라 75% 값: 1행 빈칸 자리에선 그 뒤 2행 LIMO 앞면이 처음 찬 점이 돼서 중간값을 깊게 끌어내렸음
        # (시뮬: 기준 방향 +2도에서 -0.25 로 잘못 재서 2행 LIMO 를 1행으로 봄). 1행 앞면이 가장 얕음.
        return float(np.clip(np.percentile(fronts, 75) - front_expect, -0.25, 0.25))

    def update_target(self):
        """확정 뒤 (회전 시작 전까지) 칸은 그대로 두고 위치만 가까이서 본 LIMO 옆면으로 다시 다듬음.
        odom 거리 배율이 틀리면 멀리서 본 LIMO 일수록 위치가 (배율 오차 x 본 거리) 만큼 틀어져서."""
        d = self.decided
        if d is None or d['kind'] != 'row1':
            return d
        occn = ((self.Hn >= 2) & (self.Hn >= 0.5 * self.F)).astype(np.float32)
        G_new = self._refine(occn, d['G'], d['row'], d['col'])
        if G_new is not None and abs(G_new - d['G']) <= 0.15:
            d['G_near'] = G_new
            d['target_s'] = G_new + LINE + 0.5 * CELL_S + PITCH_S * d['col']
        return d

    def decide(self, G_lo, G_hi):
        """확정 전 None. 확정되면 dict(kind='row1'|'row1_full', row, col, G, target_s, ...)."""
        if self.decided is not None:
            return self.update_target()
        c = self.cfg
        Gs, scores, stats, occ = self.evaluate(G_lo, G_hi)
        k_best, i_best = np.unravel_index(np.argmax(scores), scores.shape)
        best = scores[k_best, i_best]
        G_best = Gs[i_best]
        # 다른 목표를 고르는 가설: 목표 칸 가운데가 0.4 m 넘게 다르거나 (칸 간격 0.52 - 같은 칸 안에서 G 가 조금 다른 건 같은 목표) 행이 다른 것
        # (G 를 한 칸 옮기고 빈칸 열도 한 칸 옮기면 같은 자리 - 같은 목표로 봄)
        row, col = divmod(int(k_best), 3)
        self.best = (float(G_best), row, col, float(best))
        t_best = G_best + LINE + 0.5 * CELL_S + PITCH_S * col
        alt = scores.copy()
        for k in range(6):
            r_, c_ = divmod(k, 3)
            if r_ != row:
                continue
            tk = Gs + LINE + 0.5 * CELL_S + PITCH_S * c_
            alt[k, np.abs(tk - t_best) <= 0.40] = -1e9
        second = alt.max()
        f_target = stats[k_best, i_best, 1]
        ok = best - second >= c['decide_margin'] and getattr(self, 'dd_ok', True)
        a_row1 = [min(max(stats[q, i_best, 0] - c['dead'], 0) / c['occ_full'], 1.0) for q in (0, 1, 2)]
        if row == 0:
            # 목표 칸 가운데 넓은 곳에 빔이 많이 지나갔어야 함
            # 1행: 바로 옆 칸 LIMO 는 봤어야 함. 1행 셋 다 요구하면 1열 빈칸일 때 3열 LIMO 가 2열 LIMO 에 가려
            #   늦게 보여 회전 시작점을 0.5 m 지나서 정했음 (시뮬 LIMO ±0.1 m).
            # 2행 1열 LIMO(격자 시작 표시, 칸 앞쪽이 트여 멀리서도 잘 보임)도 봤어야 함 - 이게 없으면 격자를 한 칸 앞으로
            #   잘못 맞춰 칸 앞 빈 바닥을 1열 빈칸으로 고르는 일이 있었음.
            a_r2c1 = min(max(stats[3, i_best, 0] - c['dead'], 0) / c['occ_full'], 1.0)
            ok = ok and f_target >= c['target_free'] and a_row1[col] == 0.0 and a_r2c1 >= c['need_seen'] and \
                all(a_row1[q] >= c['need_seen'] for q in (col - 1, col + 1) if 0 <= q <= 2)
        else:
            ok = ok and all(v >= c['need_seen'] for v in a_row1)
        info = dict(best=float(best), second=float(second), row=row, col=col, G=float(G_best))
        if not ok:
            self.hist = []
            return None
        # G 다듬기: 같은 가설 점수 거의 같은 구간의 가운데 -> 1행 LIMO 진행 방향 가운데로
        same = np.zeros(len(Gs), bool)
        for k in range(6):
            r_, c_ = divmod(k, 3)
            if r_ == row:
                tk = Gs + LINE + 0.5 * CELL_S + PITCH_S * c_
                same |= (np.abs(tk - t_best) <= 0.40) & (scores[k] >= best - 0.02) & (k == k_best)
        plateau = np.where(same)[0]
        G_mid = float(Gs[plateau].mean())
        G_ref, aligned = self._refine(occ, G_mid, row, col, check=True)
        G_use = G_ref if G_ref is not None else G_mid
        if row == 0 and not aligned:
            self.hist = []
            return None
        self.hist.append((row, col, G_use))
        if len(self.hist) >= c['stable'] and all(h[:2] == (row, col) and abs(h[2] - G_use) < 0.05
                                                 for h in self.hist[-c['stable']:]):
            target = G_use + LINE + 0.5 * CELL_S + PITCH_S * col
            kind = 'row1' if row == 0 else 'row1_full'
            self.decided = dict(info, kind=kind, G=G_use, G_mid=G_mid, G_ref=G_ref, target_s=target)
            return self.decided
        return None

    def _blob_centres(self, occ, row):
        """그 행 깊이의 찬 점을 진행 방향 덩어리로 나눠 덩어리마다 LIMO 진행 방향 가운데 [s].
        가운데 = '다가오는 쪽 옆면'(깊이 줄마다 가장 앞에 찬 점)의 중간값 + LIMO 반폭.
        옆면은 다가오면서 늘 잘 보이고, 반대쪽 끝(앞면 먼 모서리)은 늦게 보여서 양끝 범위로 재면 차 쪽으로 쏠렸음
        (시뮬 1열 빈칸에서 -0.34 m). 덩어리로 나누는 건 G 가 아직 0.3 m 까지 틀릴 수 있어 칸 창으로 자르면
        옆 칸 LIMO 가 섞였기 때문 (시뮬 -0.22 m). LIMO 사이 틈 0.28 m 쯤 - 6 cm 넘게 비면 다른 덩어리."""
        c = self.cfg
        top = ROW_TOP[row] + getattr(self, 'dd', 0.0)
        j0 = max(0, int(round((top - CELL_D - self.d_lo) / self.res)))
        j1 = int(round((top - self.d_lo) / self.res))
        band = occ[:, j0:j1]
        idx = np.where(band.sum(1) > 0)[0]
        blobs = []
        if len(idx):
            start = prev = idx[0]
            for i in idx[1:]:
                if i - prev > 3:
                    blobs.append((start, prev))
                    start = i
                prev = i
            blobs.append((start, prev))
        out = []
        for b0, b1 in blobs:
            if (b1 - b0 + 1) * self.res > 0.40:
                continue                                   # 두 대가 붙어 보임 - 안 씀
            near = []
            for j in range(band.shape[1]):
                ii = np.where(band[b0:b1 + 1, j] > 0)[0]
                if len(ii):
                    near.append(self.s_lo + (b0 + ii[0] + 0.5) * self.res)
            if len(near) < c['near_min']:
                continue
            out.append(float(np.median(near)) + c['limo_half_w'])
        return out

    def _refine(self, occ, G, erow, ecol, check=False):
        """1행 LIMO 가운데 - 칸 가운데 = G 보정량의 평균으로 G 를 다듬음.
        check=True 면 (G, 문제 없음) 을 돌려줌: 1행 LIMO 와 2행 1열 LIMO 가 다 다듬은 격자의 칸 가운데에서
        align_tol 안이어야 함. 아니면 격자가 칸 반쯤 어긋난 것 (시뮬: 이른 판단에서 0.5 m 앞으로 맞춰 칸 앞 바닥을 고름)."""
        c = self.cfg
        offs = []
        for centre in self._blob_centres(occ, 0):
            col = int(round((centre - (G + LINE + 0.5 * CELL_S)) / PITCH_S))
            if col < 0 or col > 2 or (erow == 0 and col == ecol):
                continue
            off = centre - (G + LINE + 0.5 * CELL_S + PITCH_S * col)
            if abs(off) <= 0.30:
                offs.append(off)
        if not offs:
            return (None, False) if check else None
        G2 = G + float(np.mean(offs))
        if not check:
            return G2
        good = all(abs(o - (G2 - G)) <= c['align_tol'] for o in offs)
        r2 = [x - (G2 + LINE + 0.5 * CELL_S) for x in self._blob_centres(occ, 1)]
        good = good and any(abs(x) <= c['align_tol'] for x in r2)        # 2행 1열 LIMO 가 격자 1열 가운데
        return G2, good
