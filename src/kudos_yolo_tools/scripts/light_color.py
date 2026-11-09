#!/usr/bin/env python3
"""
light_color.py — 신호등 박스 안의 불 색 판정 (YOLO 는 신호등 1클래스, 색은 여기서)

판정 순서
  1) 박스 안에서 "밝고 색이 진한" 픽셀만 남긴다 (켜진 램프).
  2) 그 픽셀의 색상(hue)으로 빨강/노랑/초록 표를 센다.
  3) 켜진 픽셀이 너무 적거나 두 색이 비슷하면 → 박스 세로 3등분 중 가장 밝은 칸
     (위=빨강, 가운데=노랑, 아래=초록) 으로 한 번 더 본다.
  4) 그래도 애매하면 0(모름). 판단 쪽은 모름 = 정지로 처리하므로 틀린 색보다 낫다.

반환값은 MissionObject.light_color 규약과 같다: 0 모름, 1 빨강, 2 노랑, 3 초록
ROS 없이 import 할 수 있다 (오프라인 시험용).
"""
import numpy as np
import cv2

UNKNOWN, RED, YELLOW, GREEN = 0, 1, 2, 3
NAMES = {UNKNOWN: 'unknown', RED: 'red', YELLOW: 'yellow', GREEN: 'green'}

# OpenCV HSV: H 0~179
_HUE = {RED: [(0, 10), (165, 179)], YELLOW: [(14, 38)], GREEN: [(40, 100)]}


def _hue_votes(hsv, mask):
    h = hsv[..., 0][mask]
    votes = {}
    for c, ranges in _HUE.items():
        votes[c] = int(sum(((h >= a) & (h <= b)).sum() for a, b in ranges))
    return votes


def classify(bgr, box, s_min=90, v_min=150, min_lit_frac=0.01, dominance=1.8):
    """bgr: 전체 영상, box: (x1, y1, x2, y2) → (색 코드, 근거 문자열)"""
    h_img, w_img = bgr.shape[:2]
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w_img, x2), min(h_img, y2)
    if x2 - x1 < 3 or y2 - y1 < 6:
        return UNKNOWN, 'box too small'
    roi = bgr[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lit = (hsv[..., 1] >= s_min) & (hsv[..., 2] >= v_min)
    n_lit = int(lit.sum())

    if n_lit >= max(4, min_lit_frac * lit.size):
        votes = _hue_votes(hsv, lit)
        ranked = sorted(votes.items(), key=lambda kv: kv[1], reverse=True)
        (c1, n1), (_, n2) = ranked[0], ranked[1]
        if n1 > 0 and n1 >= dominance * max(n2, 1):
            return c1, 'hue %s' % votes

    # 위치로 보조 판정: 세로 3등분 중 밝은 칸 (가로로 누운 신호등이면 가로 3등분)
    v = hsv[..., 2].astype(np.float32)
    vertical = roi.shape[0] >= roi.shape[1]
    parts = np.array_split(v, 3, axis=0 if vertical else 1)
    means = [float(p.mean()) if p.size else 0.0 for p in parts]
    order = np.argsort(means)[::-1]
    if means[order[0]] > 1.4 * max(means[order[1]], 1.0) and means[order[0]] > 90:
        return (RED, YELLOW, GREEN)[int(order[0])], 'thirds %s' % [round(m) for m in means]
    return UNKNOWN, 'unclear lit=%d' % n_lit


def classify_lamp(bgr, box, inner=0.6, hot_v=225, hot_s=120, min_hot=2, hot_dom=3.0,
                  v_lit_min=120, ratio=1.35, diff=35, green_ratio=1.5, green_diff=50, top_k=3):
    """실차 신호등용 (11/9): '어느 램프가 켜졌나' 를 보고 위치로 색을 정한다.

    실물(12 x 20.5 cm) 은 렌즈 자체가 빨강/주황/초록 플라스틱이라 꺼져 있어도 색이 보이고,
    켜진 LED 는 하얗게 탄 작은 점이거나 렌즈보다 조금 밝은 정도로 찍힌다 (tl_red_test 확인).
    → 색상(hue) 투표는 세 색이 다 나와서 판정 불가. 램프별 밝기만 비교한다.
      1) 램프마다 하얀 점(V>=hot_v & S<=hot_s, 또는 V>=250) 개수 → 한 램프만 많으면 그 램프
      2) 없으면 램프마다 가장 밝은 top_k 픽셀 평균 → 1등이 2등보다 ratio 배 & diff 이상 밝으면 그 램프
         (LED 점이 1~3 픽셀이라 흐리게(blur) 하면 사라짐 — 11/9 원본 해상도에서 확인)
         초록은 잘못 보면 빨간불에 출발하므로 더 엄격하게 (green_ratio, green_diff)
      3) 아니면 0(모름) — 판단은 모름 = 정지
    세로 신호등: 위=빨강, 가운데=노랑, 아래=초록. 가로면 왼쪽부터 빨강, 노랑, 초록.
    박스 양옆(배경·날개)은 버리고 가운데 inner 비율만 본다.
    """
    h_img, w_img = bgr.shape[:2]
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w_img, x2), min(h_img, y2)
    if x2 - x1 < 3 or y2 - y1 < 6:
        return UNKNOWN, 'box too small'
    roi = bgr[y1:y2, x1:x2]
    vertical = roi.shape[0] >= roi.shape[1]
    if not vertical:
        roi = cv2.rotate(roi, cv2.ROTATE_90_CLOCKWISE)   # 왼쪽 → 위
    w = roi.shape[1]
    m = int(round(w * (1.0 - inner) / 2.0))
    roi = roi[:, m:w - m] if w - 2 * m >= 2 else roi
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    S, V = hsv[..., 1], hsv[..., 2]
    hots, peaks = [], []
    for idx in np.array_split(np.arange(roi.shape[0]), 3):
        if idx.size == 0:
            hots.append(0); peaks.append(0); continue
        s, v = S[idx], V[idx]
        hots.append(int((((v >= hot_v) & (s <= hot_s)) | (v >= 250)).sum()))
        flat = np.sort(v.ravel())
        peaks.append(int(flat[-top_k:].mean()) if flat.size else 0)
    colors = (RED, YELLOW, GREEN)
    o = np.argsort(hots)[::-1]
    if hots[o[0]] >= min_hot and hots[o[0]] >= hot_dom * max(hots[o[1]], 0.5):
        return colors[int(o[0])], 'hot %s peak %s' % (hots, peaks)
    o = np.argsort(peaks)[::-1]
    p1, p2 = peaks[o[0]], peaks[o[1]]
    c = colors[int(o[0])]
    r_need, d_need = (green_ratio, green_diff) if c == GREEN else (ratio, diff)
    if p1 >= v_lit_min and p1 >= r_need * max(p2, 1) and p1 - p2 >= d_need:
        return c, 'peak %s hot %s' % (peaks, hots)
    return UNKNOWN, 'unclear peak %s hot %s' % (peaks, hots)


def classify_by(method, bgr, box, s_min=90, v_min=150):
    """method: 'hue' (시뮬, 기존) | 'lamp' (실차)"""
    if method == 'lamp':
        return classify_lamp(bgr, box)
    return classify(bgr, box, s_min, v_min)
