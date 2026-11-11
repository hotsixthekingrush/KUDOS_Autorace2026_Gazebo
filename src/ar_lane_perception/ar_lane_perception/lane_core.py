"""차선 검출과 lane_error 계산 (ROS 의존 없음).

limo_ros2_application/detect_line.py의 선 추적 로직을 그대로 옮기고,
옆 도로 선 오검출 가드를 더했다. camera_perception_node가 이 클래스를 감싼다.
좌표계: base_link 지면, x 전방 / y 왼쪽 +.
"""
import numpy as np
import cv2

DEFAULTS = {
    # 노란 선 색 필터: 'hls'(기본, 기존 동작) 또는 'lab'
    'color_space': 'hls',
    # 색 임계값 (HLS)
    'lane_h_l': 0, 'lane_l_l': 90, 'lane_s_l': 100,
    'lane_h_h': 60, 'lane_l_h': 220, 'lane_s_h': 255,
    # 색 임계값 (LAB, OpenCV 8bit: L 0~255, a/b 128이 무채색).
    # 시뮬 렌더 영상의 노란 선 픽셀 분포로 정한 임시값. 선배 임계값 아님, 실차에서 다시 맞출 것.
    'lab_l_min': 100, 'lab_l_max': 255,
    'lab_a_min': 90, 'lab_a_max': 140,
    'lab_b_min': 175, 'lab_b_max': 255,
    # 흰 선 (3차선 흰 점선). color_space를 따라 HLS / LAB 임계값을 씀.
    # 시뮬 흰 선 실측 LAB (255,128,128), HLS (0,255,0) 기준 임시값. 실차 미확인.
    'white_enable': True,
    'white_lab_l_min': 170, 'white_lab_l_max': 255,
    'white_lab_a_min': 118, 'white_lab_a_max': 138,
    'white_lab_b_min': 118, 'white_lab_b_max': 138,
    'white_h_l': 0, 'white_l_l': 170, 'white_s_l': 0,
    'white_h_h': 180, 'white_l_h': 255, 'white_s_h': 60,
    # 가로선 / 가로 줄무늬 제거 (노랑, 흰색 마스크 각각에 적용)
    'clean_enable': True,
    'blob_angle_max_deg': 30.0,  # (a) 지면 덩어리 장축이 현재 헤딩과 이 각도 넘게 다르면 제거
    'blob_min_len_m': 0.08,      # (a) 장축 길이가 이보다 짧으면 방향 검사 안 함 (방향이 불안정)
    'blob_max_width_m': 0.08,    # (a) 덩어리 평균 폭(지면 면적/장축 길이)이 이보다 넓으면 제거
    'row_width_ratio': 0.6,      # (b) 한 행의 한 구간 지면 폭 >= 차선 폭(실측 기억) * 이 값이면 그 행 제외
    'row_max_runs': 3,           # (b) 한 행에서 같은 색 구간이 차선 폭 하나(실측 기억 + guard_width_tol_m)
                                 #     안에 이 개수 이상 들어 있으면 그 행 제외. 옆 도로 선(0.2 m 바깥)은 세지 않음
    # 선 추적 (영상 px)
    'side': 'both',           # both | left | right
    'center_x': 320,          # 처음 시작할 때만 좌/우 선을 가르는 기준
    'y_top': 320,
    'y_bot': 470,
    'n_rows': 8,
    'horizon_y': 260,         # (미사용, 호환용) 예상 픽셀 폭은 실측 폭을 지면 투영으로 환산한다
    'lane_w_bottom': 560,     # (미사용, 호환용)
    'smooth': 0.7,            # distance_y 필터: 클수록 빠르게 반응
    'max_step': 100.0,        # 프레임당 distance_y 최대 변화
    'track_tol': 90,          # 위/아래 줄에서 같은 선으로 인정하는 거리(px)
    'track_predict': False,   # True: 아래 두 줄의 기울기로 다음 줄 위치 예측
    'lost_hold': 5,
    # lane_error (지면 좌표)
    'lane_width_m': 0.55,     # (미사용, 호환용) 차선 폭은 실시간 실측값을 쓴다. 아래 '차선 폭 실측' 참고
    # 차선 폭 실측: 두 선이 같은 줄에서 함께 보이는 프레임에서만 재서 기억한다.
    'width_min_m': 0.40,          # 프레임 폭(두 선 줄 폭의 중앙값)이 이 범위를 벗어나면 기억을 갱신하지 않고
    'width_max_m': 0.95,          #   그 프레임은 추적 기록과 이어지는 선 하나만 남겨 한 선 프레임으로 처리
    'width_max_step_m': 0.03,     # 기억값에서 한 프레임에 이만큼 넘게 바뀌어도 같은 처리
    'width_alpha': 0.3,           # 기억 갱신 비율 (완만하게): 기억 = (1-a)*기억 + a*이번 폭
    'width_stale_sec': 3.0,       # 마지막 실측 뒤 이 시간이 지나면 기억을 버림 (고정 폭 경우 '가')
    'pair_ratio_tol': 0.4,        # 한 줄의 두 선 픽셀 간격이 기억한 폭의 (1 +- 이 값) 배 밖이면 한 선만 남김
    # 고정 폭: 두 선 프레임에서는 쓰지 않는다. 한 선 프레임에서 아래 두 경우에만.
    'lane_width_default_m': 0.50,  # (가) 실측이 한 번도 없었거나 width_stale_sec 넘게 지났을 때
    'lane_width_corner_m': 0.75,   # (나) 보이는 선이 휘어 있을 때: 휜 정도에 비례해 max(기억 폭, 이 값) 쪽으로 서서히
    'corner_curv_min': 0.3,        # (나) 보이는 선의 곡률 [1/m] 이 이 값 이하면 직선으로 봄 (기억 폭 그대로)
    'corner_curv_full': 1.5,       # (나) 곡률이 이 값 이상이면 목표 비율 1
    'corner_blend_per_sec': 1.0,   # (나) 비율이 1초에 최대 이만큼만 움직임 (한 번에 바꾸지 않음)
    'single_confidence': 0.5,      # 한 선 프레임의 confidence (고정)
    'pair_return_rate_mps': 0.3,   # 한 선 -> 두 선 전환 때만 lateral 변화 속도 제한 [m/s]. 0이면 끔
    'reference_distance_m': 0.5,
    'cam_height_m': 0.21,
    'cam_x_m': 0.10,
    'cam_pitch_rad': 0.0,
    'focal_px': 381.0,
    # 옆 도로 선 가드
    'guard_enable': True,
    'guard_width_tol_m': 0.10,   # 이력 없이 선 쌍을 고를 때: 간격이 기억한 실측 폭에서 이 안이어야 함
    'guard_jump_m': 0.10,        # 직전 정상값 대비 lateral이 한 번에 이만큼 넘게 바뀌면 의심
    'guard_memory_sec': 1.0,     # 직전 정상값을 이 시간까지만 비교 기준으로 씀
    'guard_head_jump_rad': 0.35,  # 애매한 프레임은 헤딩도 직전 정상값과 이 안(약 20도)으로 이어져야 valid
    # 한 선 프레임은 최근(width_stale_sec 안) 실측 폭이 있을 때만 valid (없으면 single_no_width)
    'guard_need_pair': True,
    # 판단팀은 confidence를 보지 않고 valid만 본다. 그래서 confidence가 이보다 낮으면 valid=false로 내린다
    # (선 점들이 직선에 안 맞거나 가까운/먼 헤딩이 크게 다름: 지그재그, 줄무늬, 잘못 이은 선). 0이면 끔
    'min_confidence': 0.05,
    # 차선 갈라짐 (3차선 서쪽 섬 끝 접근): valid=false.
    # 흰 선을 차선 선으로 쓴 프레임에서, 직전 정상 차선(중심 ± 실측 폭/2 - fork_margin_m) 안쪽에
    # 노란 구간(제거 전 마스크)이 있는 줄이 fork_min_rows 이상이면 갈라짐. (b)로 뺀 줄(방지턱 줄무늬)은 세지 않는다.
    # 갈라짐을 본 뒤에는 갈라짐 직전 차선과 lateral guard_jump_m 안으로 다시 이어질 때까지,
    # 또는 마지막 갈라짐 뒤 fork_timeout_sec가 지날 때까지 valid=false (섬을 지나며 옆 차선을 잡는 것 방지).
    'single_tol_m': 0.05,       # 두 선 줄의 중심선에서 이만큼 넘게 벗어난 한쪽 선 추정 줄 = 충돌. 0이면 끔
    'single_conflict': 'invalid',  # 충돌 처리: 'invalid'(그 프레임 valid=false) | 'reject'(그 줄만 버림)
    # 시작 확인: 직전 프레임이 invalid였으면, 이번 추정이 직전 프레임의 추정(raw)과 heading confirm_head_rad,
    # lateral confirm_lat_m 안으로 맞을 때만 valid (invalid 구간 뒤 첫 프레임 하나의 튀는 값 차단). 0이면 끔
    'confirm_head_rad': 0.035,
    'confirm_lat_m': 0.03,
    'fork_enable': True,
    'fork_margin_m': 0.08,
    'fork_min_rows': 2,
    'fork_timeout_sec': 3.0,
    'fork_min_white_rows': 2,   # 흰 선을 차선 선으로 쓴 줄이 이만큼 있을 때만 갈라짐 검사 (3차선 구간에서만)
    # 한쪽 선 추정만 있는 프레임(폭 맞는 쌍 0줄)도 기존 기준을 이어 가며 기준값을 갱신할지.
    # True(예전 동작)면 틀린 한쪽 선 프레임이 0.10 m씩 기준을 끌고 가서 방지턱 위에서 25 cm 틀린 값이 valid로 나갔다
    # (가제보 base_hls_1). False면 기준은 쌍이 있는 프레임에서만 갱신하고, 그 사이는 odom으로 옮긴다(motion 설정 시).
    'single_updates_ref': True,
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
        self.ref_head = None      # 마지막 valid 헤딩 (덩어리 방향 검사 기준)
        self.ref_head_t = None
        self.invalid_run = 0
        self.prev_raw, self.last_ok = None, False
        self.fork_active, self.fork_t, self.fork_ref = False, None, 0.0
        self.stats = dict(blobs_removed=0, rows_dropped=0, fork_rows=0, white_rows=0, single_rejected=0)
        self._gmap_key = None
        self._gmap = None
        self.prev_raw_t = None
        # motion(lat, head, t_from, t_to) -> (lat, head) 또는 None: t_from에 잰 차선 중심선을 t_to의 차 자세 기준으로 옮김.
        # 노드가 odom으로 설정한다. None이면 예전처럼 옮기지 않고 비교한다.
        self.motion = None
        # 차선 폭 실측 기억
        self.w_mem, self.w_t = None, None    # 기억한 미터 폭, 마지막 실측 시각
        self.half = self.p['lane_width_default_m'] / 2.0   # 이번 프레임에 쓰는 반폭 (process가 정함)
        self.corner_b = 0.0                  # (나) 코너 폭 쪽으로 옮긴 비율 0~1
        self.last_t = None
        self.last_kind = None                # 마지막 valid 프레임 종류 'pair' | 'single'
        self.out_lat = self.out_head = self.out_t = None
        self.slew_on = False
        self.counts = dict(frames=0, pair=0, single=0, fixed_a=0, fixed_b=0, width_reject=0, slew=0)

    def width_fresh(self, t):
        return (self.w_mem is not None and self.w_t is not None
                and 0.0 <= t - self.w_t <= self.p['width_stale_sec'])

    def cur_width(self):
        """판정에 쓰는 차선 폭: 기억한 실측 폭, 없으면 lane_width_default_m."""
        return self.w_mem if self.w_mem is not None else self.p['lane_width_default_m']

    def exp_px(self, y, w, h):
        """줄 y의 예상 픽셀 폭: 줄별 실측 픽셀 폭, 없으면 cur_width()를 지면 투영으로 환산."""
        if y in self.wrow:
            return self.wrow[y]
        p = self.p
        th = p['cam_pitch_rad']
        down = (y - h / 2.0) / p['focal_px'] * np.cos(th) + np.sin(th)
        return max(self.cur_width() * p['focal_px'] * max(down, 1e-3) / p['cam_height_m'], 1.0)

    def _moved(self, lat, head, t_from, t_to):
        """motion이 있으면 (lat, head)를 t_to 자세 기준으로 옮긴 값, 없거나 실패하면 그대로."""
        if self.motion is None or t_from is None:
            return lat, head
        r = self.motion(lat, head, t_from, t_to)
        return (lat, head) if r is None else r

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

    def lane_mask(self, img):
        """노란 선 마스크 (uint8, 0/255). color_space 파라미터로 HLS / LAB 선택."""
        g = self.p.get
        if g('color_space') == 'lab':
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            return cv2.inRange(lab,
                               np.array([g('lab_l_min'), g('lab_a_min'), g('lab_b_min')]),
                               np.array([g('lab_l_max'), g('lab_a_max'), g('lab_b_max')]))
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        return cv2.inRange(hls,
                           np.array([g('lane_h_l'), g('lane_l_l'), g('lane_s_l')]),
                           np.array([g('lane_h_h'), g('lane_l_h'), g('lane_s_h')]))

    def white_mask(self, img):
        """흰 선 마스크 (uint8, 0/255). color_space를 따른다."""
        g = self.p.get
        if g('color_space') == 'lab':
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            return cv2.inRange(lab,
                               np.array([g('white_lab_l_min'), g('white_lab_a_min'), g('white_lab_b_min')]),
                               np.array([g('white_lab_l_max'), g('white_lab_a_max'), g('white_lab_b_max')]))
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        return cv2.inRange(hls,
                           np.array([g('white_h_l'), g('white_l_l'), g('white_s_l')]),
                           np.array([g('white_h_h'), g('white_l_h'), g('white_s_h')]))

    def ground_maps(self, w, h):
        """픽셀별 지면 좌표 (X 전방, Y 왼쪽)와 지면 면적 [m^2]. 지평선 위는 nan. 파라미터가 같으면 캐시."""
        p = self.p
        key = (w, h, p['focal_px'], p['cam_height_m'], p['cam_pitch_rad'], p['cam_x_m'])
        if key != self._gmap_key:
            f, hc, th = p['focal_px'], p['cam_height_m'], p['cam_pitch_rad']
            uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
            xc, yc = (uu - w / 2.0) / f, (vv - h / 2.0) / f
            down = yc * np.cos(th) + np.sin(th)
            fwd = np.cos(th) - yc * np.sin(th)
            ok = down > 1e-6
            t = np.where(ok, hc / np.where(ok, down, 1.0), np.nan)
            X = t * fwd + p['cam_x_m']
            Y = -t * xc
            # 한 픽셀이 덮는 지면 면적 = |dX/dv| * |dY/du|
            dxdv = np.abs(np.gradient(X, axis=0))
            dydu = t / f
            self._gmap = (X, Y, dxdv * dydu)
            self._gmap_key = key
        return self._gmap

    def clean_mask(self, mask, y0, y1):
        """(a) 가로 방향이거나 너무 넓은 지면 덩어리 제거. mask[y0:y1]만 본다. 반환 (마스크, 제거 개수)."""
        g = self.p.get
        h, w = mask.shape[:2]
        X, Y, A = self.ground_maps(w, h)
        sub = mask[y0:y1]
        n, lab, stats, _ = cv2.connectedComponentsWithStats((sub > 0).astype(np.uint8), connectivity=8)
        if n <= 1:
            return mask, 0
        ref = self.ref_head if self.ref_head is not None else 0.0
        amax = np.radians(g('blob_angle_max_deg'))
        out = mask.copy()
        removed = 0
        idx = lab.ravel()
        order = np.argsort(idx, kind='stable')
        bounds = np.searchsorted(idx[order], np.arange(n + 1))
        Xs, Ys, As = X[y0:y1].ravel(), Y[y0:y1].ravel(), A[y0:y1].ravel()
        for k in range(1, n):
            pix = order[bounds[k]:bounds[k + 1]]
            px, py = Xs[pix], Ys[pix]
            ok = np.isfinite(px)
            if ok.sum() < 3:
                continue
            px, py, area = px[ok], py[ok], As[pix][ok].sum()
            cx, cy = px.mean(), py.mean()
            cov = np.cov(np.vstack([px - cx, py - cy]))
            ev, evec = np.linalg.eigh(cov)
            v = evec[:, 1]
            proj = (px - cx) * v[0] + (py - cy) * v[1]
            length = max(proj.max() - proj.min(), 1e-3)
            width = area / length
            ang = np.arctan2(v[1], v[0])
            d = (ang - ref + np.pi / 2) % np.pi - np.pi / 2
            bad = width > g('blob_max_width_m') or (length >= g('blob_min_len_m') and abs(d) > amax)
            if bad:
                sl = lab == k
                out[y0:y1][sl] = 0
                removed += 1
        return out, removed

    def row_groups(self, mask, y, w, h):
        """한 추적 줄의 구간 중심 목록. (b) 넓은 구간이나 구간 수 초과면 None (그 줄 제외)."""
        band = mask[max(y - 2, 0):y + 3, :]
        cols = np.nonzero(band.any(axis=0))[0]
        if len(cols) == 0:
            return []
        groups = np.split(cols, np.where(np.diff(cols) > 8)[0] + 1)
        if self.p['clean_enable']:
            _, Y, _ = self.ground_maps(w, h)
            lim = self.p['row_width_ratio'] * self.cur_width()
            for gp in groups:
                if abs(Y[y, gp[0]] - Y[y, gp[-1]]) >= lim:
                    return None
            k = int(self.p['row_max_runs'])
            if k >= 2 and len(groups) >= k:
                span = self.cur_width() + self.p['guard_width_tol_m']
                yc = sorted(Y[y, int(round(gp.mean()))] for gp in groups)
                if any(yc[i + k - 1] - yc[i] <= span for i in range(len(yc) - k + 1)):
                    return None
        return [gp.mean() for gp in groups]

    @staticmethod
    def _line_at(pts, y):
        """점 (idx, y, u) 마지막 두 개를 이은 직선의 줄 y 위치. 점이 2개 미만이면 None."""
        if len(pts) < 2:
            return None
        (_, ya, ua), (_, yb, ub) = pts[-2], pts[-1]
        if ya == yb:
            return None
        return ub + (ub - ua) / (yb - ya) * (y - yb)

    def _predict(self, pts, other, y, cx0, vh):
        """이번 프레임 점들로 줄 y의 선 위치 예측. 점이 2개 이상이면 직선 연장,
        1개면 소실점(다른 쪽 선을 지평선까지 연장한 곳, 없으면 center_x)을 향해 원근 축소."""
        u = self._line_at(pts, y)
        if u is not None:
            return u
        _, y1, u1 = pts[-1]
        if y1 - vh < 1.0:
            return u1
        vp = self._line_at(other, vh)
        vp = cx0 if vp is None else vp
        return vp + (u1 - vp) * (y - vh) / (y1 - vh)

    def _has_pair(self, means, y, w, h):
        return self._pick_pair(means, y, w, h) is not None

    def _pick_pair(self, means, y, w, h):
        """이력이 없을 때: 지면 간격이 기억한 실측 폭에 맞는(기억이 없으면 width_min_m~width_max_m 안인)
        이웃 선 쌍 중 차 중심에 가장 가까운 쌍."""
        _, Y, _ = self.ground_maps(w, h)
        if self.w_mem is not None:
            lw, tol = self.w_mem, self.p['guard_width_tol_m']
        else:
            lw = (self.p['width_min_m'] + self.p['width_max_m']) / 2.0
            tol = (self.p['width_max_m'] - self.p['width_min_m']) / 2.0
        best, bd = None, None
        ms = sorted(means)
        for a, b in zip(ms, ms[1:]):
            ya, yb = Y[y, int(round(a))], Y[y, int(round(b))]
            if abs(abs(ya - yb) - lw) <= tol:
                d = abs((ya + yb) / 2.0)
                if bd is None or d < bd:
                    best, bd = (a, b), d
        return best

    def track(self, img, want_debug=False):
        """반환: gap(px, 필터 후), newL, newR, pairs, debug 영상(또는 None).

        pairs: 한 줄에서 L, R이 둘 다 잡힌 경우의 (y, L, R). 픽셀 폭 검사로 한쪽을 버리기 전 값.
        """
        g = self.p.get
        h, w = img.shape[:2]
        y0 = max(int(min(g('y_top'), g('y_bot'))) - 2, 0)
        y1 = min(int(max(g('y_top'), g('y_bot'))) + 3, h)
        mask = self.lane_mask(img)
        self._ymask_raw = mask
        self._dropped_rows = set()
        self._wrow_new = {}      # 이번 프레임 줄별 픽셀 폭 후보 (process가 폭 검사 통과 때만 wrow에 반영)
        wmask = self.white_mask(img) if g('white_enable') else None
        self.stats = dict(blobs_removed=0, rows_dropped=0, fork_rows=0, white_rows=0, single_rejected=0)
        if g('clean_enable'):
            mask, k = self.clean_mask(mask, y0, y1)
            self.stats['blobs_removed'] += k
            if wmask is not None:
                wmask, k = self.clean_mask(wmask, y0, y1)
                self.stats['blobs_removed'] += k
        cx0 = int(g('center_x'))
        side = g('side')
        tol = g('track_tol')
        rt = g('pair_ratio_tol')
        ys = np.linspace(g('y_bot'), g('y_top'), int(g('n_rows'))).astype(int)

        dbg = None
        if want_debug:
            dbg = img.copy()
            dbg[mask > 0] = (0, 0, 255)
            if wmask is not None:
                dbg[wmask > 0] = (255, 0, 255)
            cv2.line(dbg, (cx0, 0), (cx0, h), (0, 255, 0), 2)

        centers, weights, modes = [], [], ''
        newL, newR, pairs = {}, {}, []
        lastL = lastR = None
        # 한 선을 바로 아래 줄에서 못 찾았으면(점선 끊김, (b)로 뺀 줄) 멀리 떨어진 아래 줄 위치를 그대로 쓰지 않고
        # 이번 프레임에서 찾은 점들을 이은 직선(차선은 영상에서도 직선)으로 연장해 찾는다. _predict 참고.
        ptsL, ptsR = [], []
        vh = h / 2.0 - g('focal_px') * np.tan(g('cam_pitch_rad'))   # 지평선 줄
        l2 = r2 = None
        predict = bool(g('track_predict'))
        for idx, y in enumerate(ys):
            y = int(y)
            means = self.row_groups(mask, y, w, h)
            dropped = means is None
            wset = set()
            if wmask is not None and not (means and self._has_pair(means, y, w, h)):
                # 흰 선은 노란 선만으로 폭이 맞는 쌍을 못 만드는 줄에서만 쓴다 (3차선 가운데 차선 등).
                # 일반 구간의 정지선 조각, 글자, 마름모가 선 후보로 끼어드는 것을 막는다.
                wm = self.row_groups(wmask, y, w, h)
                wset = set(wm or [])
                dropped = dropped or wm is None
                means = None if means is None and wm is None else sorted((means or []) + (wm or []))
            if dropped:
                # 한 색이라도 (b)로 빠진 줄: 남은 색 구간은 쓰되, 다음 줄 예측은 직선 연장으로
                self.stats['rows_dropped'] += 1
                self._dropped_rows.add(y)
            if means is None:
                continue
            if len(means) == 0:
                continue

            exp = self.exp_px(y, w, h)
            known = y in self.wrow or self.w_mem is not None
            pl = lastL if lastL is not None else self.prevL.get(y)
            pr = lastR if lastR is not None else self.prevR.get(y)
            if ptsL and idx - ptsL[-1][0] >= 2:
                pl = self._predict(ptsL, ptsR, y, cx0, vh)
            if ptsR and idx - ptsR[-1][0] >= 2:
                pr = self._predict(ptsR, ptsL, y, cx0, vh)
            if predict:
                if lastL is not None and l2 is not None:
                    pl = 2 * lastL - l2
                if lastR is not None and r2 is not None:
                    pr = 2 * lastR - r2
            pair = None
            if pl is None and pr is None and side == 'both':
                pair = self._pick_pair(means, y, w, h)
            if pair is not None:
                L, R = pair
            else:
                L, R = self.find_row(means, cx0, side, exp, tol, pl, pr)

            c = None
            if L is not None and R is not None:
                pairs.append((y, L, R))
                wd = R - L
                # 기억한 실측 폭이 있으면 그 폭과 비교, 없으면 줄 단위로는 거르지 않는다 (process의 범위 검사가 판정)
                if not known or (1.0 - rt) * exp <= wd <= (1.0 + rt) * exp:
                    c = (L + R) / 2.0
                    modes += 'D'
                    self._wrow_new[y] = (0.8 * exp + 0.2 * wd) if known else wd
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

            if (L is not None and L in wset) or (R is not None and R in wset):
                self.stats['white_rows'] += 1
            if L is not None:
                newL[y] = L
                l2, lastL = lastL, L
                ptsL.append((idx, y, L))
            if R is not None:
                newR[y] = R
                r2, lastR = lastR, R
                ptsR.append((idx, y, R))
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
        half = self.half
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
        C, W, n_rej = self._reject_singles(C, W)
        if self.p['single_conflict'] == 'reject':
            n_one -= n_rej
        self.stats['single_rejected'] = n_rej
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

    def _reject_singles(self, C, W):
        """두 선 줄(W=1) 중심선과 single_tol_m 넘게 어긋나는 한쪽 선 추정 줄(W=0.5)을 찾는다.
        중심선: 두 선 줄이 2개 이상이고 전방 0.10 m 넘게 퍼져 있으면 그 줄들의 직선,
        아니면 두 선 줄 중심을 지나고 직전 valid 헤딩(없으면 0) 방향인 직선.
        넓어지는 구간의 비스듬한 노란 선 하나가 맞는 쌍과 섞여 헤딩을 끌어당기는 것을 막는다.
        single_conflict='reject'면 그 줄을 버리고, 'invalid'면 줄은 두고 개수만 돌려준다(process에서 valid=false)."""
        tol = self.p['single_tol_m']
        P = np.array([c for c, wt in zip(C, W) if wt == 1.0])
        # 흰 선을 쓴 프레임(3차선 구간)에서만. 일반 코너에서는 굽은 선 때문에 한쪽 선 줄이 직선에서 벗어나는 게 정상
        if self.stats['white_rows'] < 1:
            return C, W, 0
        if tol <= 0 or len(P) == 0 or len(P) == len(C):
            return C, W, 0
        if len(P) >= 2 and np.ptp(P[:, 0]) > 0.10:
            b, a = np.polyfit(P[:, 0], P[:, 1], 1)
        else:
            b = np.tan(self.ref_head) if self.ref_head is not None else 0.0
            a = P[:, 1].mean() - b * P[:, 0].mean()
        cs = np.cos(np.arctan(b))
        keep = [wt == 1.0 or abs(c[1] - (a + b * c[0])) * cs <= tol for c, wt in zip(C, W)]
        n_bad = len(keep) - sum(keep)
        if self.p['single_conflict'] != 'reject':
            return C, W, n_bad
        return [c for c, k in zip(C, keep) if k], [wt for wt, k in zip(W, keep) if k], n_bad

    def frame_width(self, pairs, newL, newR, w, h, heading):
        """두 선을 다 쓴 줄들의 차선 폭 [m] 중앙값 (없으면 None).

        영상 가로줄은 차선을 비스듬히 자르므로 base_link y 간격에 cos(heading)을 곱해 수직 폭으로 바꾼다.
        """
        c = abs(np.cos(heading))
        ws = []
        for y, L, R in pairs:
            if newL.get(y) != L or newR.get(y) != R:
                continue
            a, b = self.pix_to_base(L, y, w, h), self.pix_to_base(R, y, w, h)
            if a is not None and b is not None:
                ws.append(abs(a[1] - b[1]) * c)
        return float(np.median(ws)) if ws else None

    def _history_side(self, newL, newR, oldL, oldR):
        """추적 기록(직전 프레임 선 위치)과 더 잘 이어지는 쪽 'L' | 'R'. 기록이 없으면 None."""
        def dist(new, old):
            d = [abs(new[y] - old[y]) for y in new if y in old]
            return float(np.mean(d)) if d else None
        dl, dr = dist(newL, oldL), dist(newR, oldR)
        if dl is None and dr is None:
            return None
        if dr is None or (dl is not None and dl <= dr):
            return 'L'
        return 'R'

    def line_curvature(self, pts, w, h):
        """한 선의 지면 곡률 크기 [1/m]. 점이 3개 미만이거나 전방 0.15 m 미만으로 퍼져 있으면 0."""
        G = [self.pix_to_base(u, y, w, h) for y, u in pts.items()]
        G = np.array([q for q in G if q is not None])
        if len(G) < 3 or np.ptp(G[:, 0]) < 0.15:
            return 0.0
        c2, c1, _ = np.polyfit(G[:, 0], G[:, 1], 2)
        slope = c1 + 2.0 * c2 * G[:, 0].mean()
        return float(abs(2.0 * c2) / (1.0 + slope * slope) ** 1.5)

    def fork_rows(self, lat, head, w, h):
        """차선 띠(중심선 lat/head 기준, 양쪽에서 fork_margin_m 안쪽) 안에 노란 구간이 있는 추적 줄 수."""
        X, Y, _ = self.ground_maps(w, h)
        g = self.p.get
        c1 = np.tan(head)
        c0 = lat * np.hypot(1.0, c1) - c1 * g('reference_distance_m')
        half = self.cur_width() / 2.0 - g('fork_margin_m')
        ys = np.linspace(g('y_bot'), g('y_top'), int(g('n_rows'))).astype(int)
        n = 0
        for y in ys:
            y = int(y)
            if y in self._dropped_rows:
                continue
            band = self._ymask_raw[max(y - 2, 0):y + 3, :]
            cols = np.nonzero(band.any(axis=0))[0]
            if len(cols) == 0:
                continue
            for gp in np.split(cols, np.where(np.diff(cols) > 8)[0] + 1):
                u = int(round(gp.mean()))
                d = (Y[y, u] - (c0 + c1 * X[y, u])) * np.cos(head)
                if abs(d) < half:
                    n += 1
                    break
        return n

    def _fork_check(self, ok, why, lat, head, t, w, h):
        """차선 갈라짐 검사와 갈라짐 뒤 유지. 반환 (valid, 사유)."""
        g = self.p.get
        fresh = self.ref_t is not None and 0.0 <= t - self.ref_t <= g('guard_memory_sec')
        n = 0
        if self.stats['white_rows'] >= g('fork_min_white_rows'):
            # 직전 정상 차선 기준 (없으면 이번 추정)
            bl, bh = (self.ref_lat, self.ref_head if self.ref_head is not None else head) if fresh else (lat, head)
            n = self.fork_rows(bl, bh, w, h)
        self.stats['fork_rows'] = n
        if n >= g('fork_min_rows'):
            if not self.fork_active:
                self.fork_ref = self.ref_lat if fresh else lat
            self.fork_active, self.fork_t = True, t
            return False, 'lane_fork'
        if self.fork_active:
            if not 0.0 <= t - self.fork_t <= g('fork_timeout_sec'):
                self.fork_active = False
            elif ok and abs(lat - self.fork_ref) <= g('guard_jump_m'):
                self.fork_active = False
            else:
                return False, 'fork_hold'
        return ok, why

    def guard(self, valid, single, t):
        """한 선 프레임 가드. 반환: (valid, 사유 문자열 또는 '').

        guard_need_pair가 켜져 있으면 한 선 프레임은 최근(width_stale_sec 안)에 잰 실측 폭이 있을 때만 믿는다.
        (예전의 lane_width_m 대비 폭 오차 판정 ambiguous_* / single_* 는 없앴다. 폭이 안 맞는 프레임은
        process()에서 추적 기록과 이어지는 한 선만 남긴다.)
        """
        if not valid or not self.p['guard_enable']:
            return valid, ''
        if single and self.p['guard_need_pair'] and not self.width_fresh(t):
            return False, 'single_no_width'
        return True, ''

    def process(self, img, t, want_debug=False):
        """한 프레임 처리. t: 촬영 시각 [s]. 반환 dict."""
        h, w = img.shape[:2]
        if self.ref_head_t is not None and not 0.0 <= t - self.ref_head_t <= self.p['guard_memory_sec']:
            self.ref_head = None      # 오래된 헤딩은 덩어리 방향 검사 기준으로 쓰지 않음 (0으로 봄)
        g = self.p.get
        dt = 0.0 if self.last_t is None else float(np.clip(t - self.last_t, 0.0, 0.5))
        self.last_t = t
        if self.w_mem is not None and not self.width_fresh(t):
            self.w_mem, self.w_t, self.wrow = None, None, {}     # 오래된 실측은 버림 -> 경우 (가)
        oldL, oldR = dict(self.prevL), dict(self.prevR)
        gap, newL, newR, pairs, dbg = self.track(img, want_debug)
        tracked = bool(newL or newR)
        self.half = self.cur_width() / 2.0
        lat, head, valid, conf, n_both, n_one = self.lane_error(newL, newR, w, h)
        # --- 차선 폭 실측과 범위 검사
        wf = self.frame_width(pairs, newL, newR, w, h, head) if valid else None
        werr, note, no_hist = None, '', False
        if wf is not None:
            werr = abs(wf - self.w_mem) if self.w_mem is not None else None
            if not g('width_min_m') <= wf <= g('width_max_m') or (werr is not None and werr > g('width_max_step_m')):
                # 기억 갱신 안 함. 추적 기록과 이어지는 선 하나만 믿고 한 선 프레임으로
                note = 'width_reject'
                self.counts['width_reject'] += 1
                keep = self._history_side(newL, newR, oldL, oldR)
                if keep is None:
                    no_hist = True
                elif keep == 'L':
                    newR = {}
                else:
                    newL = {}
                if tracked:
                    self.prevL, self.prevR = newL, newR
            else:
                a = g('width_alpha')
                self.w_mem = wf if self.w_mem is None else (1.0 - a) * self.w_mem + a * wf
                self.w_t = t
                self.wrow.update(self._wrow_new)
        # --- 반폭 결정. 두 선 프레임: 기억한 실측 폭만. 한 선 프레임: 기억 폭, (가) 기본 폭, (나) 코너 폭 쪽으로
        single = bool(newL or newR) and not (set(newL) & set(newR))
        fixed = ''
        W = self.cur_width()
        b_tgt = 0.0
        if single:
            if self.w_mem is None:
                fixed += 'a'
            k = self.line_curvature(newL or newR, w, h)
            k0, k1 = g('corner_curv_min'), g('corner_curv_full')
            b_tgt = float(np.clip((k - k0) / max(k1 - k0, 1e-6), 0.0, 1.0))
        step = g('corner_blend_per_sec') * dt
        self.corner_b += float(np.clip(b_tgt - self.corner_b, -step, step))
        if single and self.corner_b > 0.0:
            Wc = max(W, g('lane_width_corner_m'))
            if Wc > W:
                W = W + self.corner_b * (Wc - W)
                fixed += 'b'
        if abs(W / 2.0 - self.half) > 1e-12 or note:
            self.half = W / 2.0
            lat, head, valid, conf, n_both, n_one = self.lane_error(newL, newR, w, h)
        single = single and valid
        if single:
            conf = float(g('single_confidence'))
        n_good = n_both
        ok, why = self.guard(valid, single, t)
        if no_hist:
            ok, why = False, 'width_reject_no_history'
        if ok and self.p['single_conflict'] != 'reject' and self.stats['single_rejected'] > 0:
            ok, why = False, 'single_conflict'
        if self.p['fork_enable']:
            ok, why = self._fork_check(ok, why, lat, head, t, w, h)
        if ok and conf < self.p['min_confidence']:
            ok, why = False, 'low_confidence'
        prev = self.prev_raw
        if prev is not None:
            # 직전 프레임 추정을 이번 프레임 자세로 옮겨 비교 (카메라 3~4 Hz에서 프레임 사이 차가 몇 도씩 돈다)
            prev = self._moved(prev[0], prev[1], self.prev_raw_t, t)
        self.prev_raw = (lat, head) if valid else None
        self.prev_raw_t = t
        if ok and not self.last_ok and self.p['confirm_head_rad'] > 0:
            if (prev is None or abs(head - prev[1]) > self.p['confirm_head_rad']
                    or abs(lat - prev[0]) > self.p['confirm_lat_m']):
                ok, why = False, 'confirm'
        self.last_ok = ok
        # --- 한 선 -> 두 선 전환 때만 lateral 변화 속도 제한 (따라잡으면 해제)
        lat_raw = lat
        kind = ('single' if single else 'pair') if ok else ''
        if ok:
            rate = g('pair_return_rate_mps')
            if kind == 'pair' and self.last_kind == 'single':
                self.slew_on = True
            elif kind != 'pair':
                self.slew_on = False
            gap_t = None if self.out_t is None else t - self.out_t
            if self.slew_on and rate > 0 and gap_t is not None and 0.0 < gap_t <= g('guard_memory_sec'):
                prev_lat = self._moved(self.out_lat, self.out_head, self.out_t, t)[0]
                lim = rate * gap_t
                if abs(lat - prev_lat) <= lim:
                    self.slew_on = False
                else:
                    lat = float(prev_lat + np.copysign(lim, lat - prev_lat))
                    self.counts['slew'] += 1
            else:
                self.slew_on = False
            self.last_kind, self.out_lat, self.out_head, self.out_t = kind, lat, head, t
            self.counts[kind] += 1
            if 'a' in fixed:
                self.counts['fixed_a'] += 1
            if 'b' in fixed:
                self.counts['fixed_b'] += 1
        else:
            self.slew_on = False
        self.counts['frames'] += 1
        if dbg is not None:
            cv2.putText(dbg, '%s W=%.2f mem=%s fixed=%s' % (kind or 'invalid', 2 * self.half,
                        'none' if self.w_mem is None else '%.2f' % self.w_mem, fixed or '-'),
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if ok:
            # 가드 기준값 갱신. 폭이 맞는 선 쌍이 없는 프레임은 옆 도로 선 하나만 잡은 것일 수도 있으므로
            # 새 기준을 세우지 못하고, 이미 있는 기준을 이어 갈 때만 쓴다.
            fresh = self.ref_t is not None and 0.0 <= t - self.ref_t <= self.p['guard_memory_sec']
            if n_good > 0:
                self.ref_lat, self.ref_t = lat_raw, t
                self.ref_head, self.ref_head_t = head, t
            elif self.p['single_updates_ref']:
                if fresh and abs(lat_raw - self._moved(self.ref_lat, head, self.ref_t, t)[0]) <= self.p['guard_jump_m']:
                    self.ref_lat, self.ref_t = lat_raw, t
                self.ref_head, self.ref_head_t = head, t
            self.invalid_run = 0
        else:
            # valid=false가 lost_hold 프레임 넘게 이어지면 선 추적 이력을 지운다.
            # 잘못 잡은 선을 계속 따라가며 invalid에 갇히는 것을 막고, 이력 없는 쌍 선택(차 중심에 가까운
            # 폭 맞는 쌍)으로 다시 잡게 한다.
            self.invalid_run += 1
            if self.invalid_run > self.p['lost_hold']:
                self.prevL, self.prevR = {}, {}
                self.invalid_run = 0
        return dict(gap=gap, lateral=lat, heading=head, valid=ok, raw_valid=valid,
                    confidence=conf, single=n_one > 0, guard=why, width_err=werr,
                    n_one=n_one, n_good=n_good, n_both=n_both, debug=dbg, lat_raw=lat_raw, kind=kind,
                    width_meas=wf, width_mem=self.w_mem, width_used=2.0 * self.half, fixed=fixed if ok else '',
                    width_note=note, slew=lat != lat_raw, corner_b=self.corner_b, **self.stats)
