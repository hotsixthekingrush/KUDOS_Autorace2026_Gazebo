#!/usr/bin/env python3
"""
guide_from_paint.py — 트랙 페인트 메시(yellow.dae)에서 M6 지그재그 서행 유도선을 찾아 라벨 설정에 넣고,
이미 모은 사진(meta/*.json 카메라 자세)에 slow_guide_line(클래스 8) 라벨을 덧붙인다. (11/10)

  python3 guide_from_paint.py --cfg ~/yolo_data/v2/config_full --data ~/yolo_data/v2/sim_full
  python3 guide_from_paint.py ... --dry        (라벨을 쓰지 않고 몇 장에 붙을지만 셈)
  (crosswalk_from_paint.py 와 같은 방식. 다시 돌려도 이미 붙은 라벨·설정은 건너뜀)

찾는 방법 — find_zigzag() 는 균명이 zigzag_find.py 에서 고른 기준 (11/10)
  노란 페인트 조각 중 오른쪽 세로 길(x > 9.0), 코너와 횡단보도 사이(y 2.3~4.7)의
  짧은 조각(20 cm 미만)만 골라 전체 범위 = 지그재그 두 줄을 함께 감싸는 박스
  track_2026: x 9.24~9.91, y 2.60~4.38 (길이 1.77 m, 폭 0.67 m — 규정 1.677 m, 0.45~0.69 m)
라벨 규칙
  지그재그 구역(바닥 위 얇은 상자)의 꼭짓점을 사진에 투영한 박스. 박스 하단 = 차에 가까운 끝
  → YOLO 노드가 하단 ∩ 바닥으로 x_m(유도선 시작점까지 거리)을 냄. 판단은 그만큼 더 가서 헤딩 고정
결과
  CFG/targets_full_A.yaml, targets_full_B.yaml 에 slow_guide_line 대상 추가 (다음 수집부터 자동 라벨)
  CFG/viewpoints_full.yaml 에 guide_a 구역(위쪽에서 -y 로 접근), CFG/mission_points.yaml 에 점검 지점
  D/labels/all/*.txt 에 클래스 8 줄 추가, D/debug_guide/ 확인 사진 12장
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import cv2
import yaml

GUIDE = 8


def find_zigzag(pieces):
    """pieces: [(x0, x1, y0, y1)] 노란 조각 범위 → 지그재그 전체 범위 (균명 11/10)"""
    z = [p for p in pieces
         if p[0] > 9.0 and 2.3 < p[2] and p[3] < 4.7      # 1) right road, between corner and crosswalk
         and (p[1] - p[0]) < 0.2 and (p[3] - p[2]) < 0.2]  # 2) short pieces only (< 20 cm)
    if not z:
        return None
    return (min(p[0] for p in z), max(p[1] for p in z),    # 3) box around all chosen pieces
            min(p[2] for p in z), max(p[3] for p in z))


def project_ground_rect(T, K, x0, x1, y0, y1, z, min_z=0.1):
    """바닥 직사각형(월드)을 카메라 광학 좌표로 옮겨 z >= min_z 부분만 남긴 뒤 투영. 안 보이면 None"""
    P = np.array([[x0, y0, z, 1], [x1, y0, z, 1], [x1, y1, z, 1], [x0, y1, z, 1]], float)
    C = (np.asarray(T) @ P.T).T[:, :3]
    out = []                                   # Sutherland–Hodgman: 평면 z = min_z 로 자르기
    for i in range(len(C)):
        a, b = C[i], C[(i + 1) % len(C)]
        ina, inb = a[2] >= min_z, b[2] >= min_z
        if ina:
            out.append(a)
        if ina != inb:
            t = (min_z - a[2]) / (b[2] - a[2])
            out.append(a + t * (b - a))
    if len(out) < 3:
        return None
    C = np.array(out)
    K = np.asarray(K)
    return np.stack([K[0, 0] * C[:, 0] / C[:, 2] + K[0, 2], K[1, 1] * C[:, 1] / C[:, 2] + K[1, 2]], 1)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('--cfg', required=True)
    ap.add_argument('--data', required=True)
    ap.add_argument('--repo', default=os.path.expanduser('~/26v2/KUDOS_Autorace2026_Gazebo'))
    ap.add_argument('--lib', default=here, help='crosswalk_from_paint.py, yolo_geometry.py 가 있는 폴더')
    ap.add_argument('--max-dist', type=float, default=4.5, help='카메라~구역 가까운 끝 거리가 이보다 멀면 안 붙임 [m]')
    ap.add_argument('--dry', action='store_true')
    a = ap.parse_args()
    sys.path.insert(0, a.lib)
    import yolo_geometry as geo
    from crosswalk_from_paint import collada_triangles, components
    cfg, data = os.path.expanduser(a.cfg), os.path.expanduser(a.data)

    dae = os.path.join(os.path.expanduser(a.repo), 'src/kudos_arena/models/track_paint/meshes/yellow.dae')
    if not os.path.exists(dae):
        print('yellow.dae 없음: %s (--repo 확인)' % dae); sys.exit(2)
    tris = collada_triangles(dae)
    pieces = []
    for c in components(tris):
        P = c.reshape(-1, 3)[:, :2]
        (x0, y0), (x1, y1) = P.min(0), P.max(0)
        pieces.append((float(x0), float(x1), float(y0), float(y1)))
    box = find_zigzag(pieces)
    if box is None:
        print('지그재그 못 찾음 (노란 조각 %d개)' % len(pieces)); sys.exit(2)
    x0, x1, y0, y1 = box
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    size = [x1 - x0, y1 - y0, 0.004]
    center = [cx, cy, 0.004]
    print('지그재그: x %.3f~%.3f, y %.3f~%.3f (길이 %.2f m, 폭 %.2f m) — 노란 조각 %d개 중'
          % (x0, x1, y0, y1, max(size[:2]), min(size[:2]), len(pieces)))

    if not a.dry:
        # --- 설정 파일 ---
        for s in ('A', 'B'):
            f = os.path.join(cfg, 'targets_full_%s.yaml' % s)
            if os.path.exists(f) and 'slow_guide_line, world_pose' not in open(f).read():
                with open(f, 'a') as fo:
                    fo.write('  - {class: slow_guide_line, world_pose: %s, size: %s}   # M6 지그재그 yellow.dae\n'
                             % ([round(v, 4) for v in center] + [0.0], [round(v, 4) for v in size]))
        vf = os.path.join(cfg, 'viewpoints_full.yaml')
        if os.path.exists(vf):
            vp = yaml.safe_load(open(vf))
            if 'guide_a' not in set(r['name'] for r in vp['regions']):
                # 차는 오른쪽 위 코너에서 -y 로 내려옴 → 카메라는 구역의 +y 쪽에서 -y 를 봄
                vp['regions'].append({'name': 'guide_a', 'type': 'facing', 'target': [round(cx, 3), round(cy, 3)],
                                      'normal_yaw': round(math.pi / 2, 4),
                                      'dist': [round(size[1] / 2 + 0.25, 3), round(size[1] / 2 + 2.5, 3)],
                                      'lateral': [-0.12, 0.12], 'yaw_jitter': 0.25, 'weight': 1.0})
                with open(vf, 'w') as f:
                    yaml.safe_dump(vp, f, allow_unicode=True, sort_keys=False, default_flow_style=None)
        pf = os.path.join(cfg, 'mission_points.yaml')
        if os.path.exists(pf):
            mp = yaml.safe_load(open(pf))
            if not any(p['class'] == 'slow_guide_line' for p in mp['points']):
                for d in (1.5, 1.0, 0.5, 0.25):
                    mp['points'].append({'name': 'guide a %.2fm' % d, 'class': 'slow_guide_line',
                                         'x': round(cx, 3), 'y': round(y1 + d, 3),
                                         'yaw': round(-math.pi / 2, 4), 'expect_x': d})
                with open(pf, 'w') as f:
                    yaml.safe_dump(mp, f, allow_unicode=True, sort_keys=False, default_flow_style=None)

    # --- 이미 모은 사진에 라벨 덧붙이기 ---
    dbg = os.path.join(data, 'debug_guide')
    os.makedirs(dbg, exist_ok=True)
    added = shown = 0
    why = {}
    corners = geo.cuboid_corners(center, size, geo.yaw_to_rot(0.0))
    for m in sorted(glob.glob(os.path.join(data, 'meta', '*.json'))):
        stem = os.path.splitext(os.path.basename(m))[0]
        lf = os.path.join(data, 'labels', 'all', stem + '.txt')
        if not os.path.exists(lf):
            why['라벨 파일 없음'] = why.get('라벨 파일 없음', 0) + 1; continue
        old = open(lf).read()
        if any(l.startswith('%d ' % GUIDE) for l in old.splitlines()):
            why['이미 있음'] = why.get('이미 있음', 0) + 1; continue
        meta = json.load(open(m))
        T = np.array(meta['T_cam_world']); K = np.array(meta['K']); w, h = int(meta['w']), int(meta['h'])
        cam = geo.inv_T(T)[:3, 3]
        # 카메라에서 구역의 가장 가까운 점까지 거리
        near = math.hypot(max(x0 - cam[0], 0, cam[0] - x1), max(y0 - cam[1], 0, cam[1] - y1))
        if near > a.max_dist:
            continue                                    # 대부분 사진 (구역이 안 보이는 곳) — 이유로 안 셈
        # 바닥 위 직사각형을 카메라 앞쪽(z >= 0.1 m)만 남기고 투영 (11/11).
        # 예전처럼 꼭짓점 하나라도 카메라 뒤면 버리면, 지그재그 위·옆에서 일부가 보이는 사진이
        # 라벨 없이 남아 '배경'으로 학습됨 → 보이는 부분만 박스로
        uv = project_ground_rect(T, K, x0, x1, y0, y1, center[2])
        if uv is None:
            why['카메라 뒤(안 보임)'] = why.get('카메라 뒤(안 보임)', 0) + 1; continue
        poly, keep = geo.clipped_hull(uv, w, h)
        if poly is None:
            why['화면 밖'] = why.get('화면 밖', 0) + 1; continue
        bx1, by1, bx2, by2 = geo.poly_bbox(poly)
        if bx2 - bx1 < 12 or by2 - by1 < 8:
            why['작음'] = why.get('작음', 0) + 1; continue
        added += 1
        if a.dry:
            continue
        with open(lf, 'w') as f:
            f.write((old.rstrip('\n') + '\n' if old.strip() else '') + geo.yolo_line(GUIDE, poly, w, h, 'polygon'))
        if shown < 12:
            img = cv2.imread(os.path.join(data, 'images', 'all', stem + '.jpg'))
            if img is not None:
                cv2.polylines(img, [np.round(poly).astype(np.int32)], True, (0, 255, 255), 2)
                cv2.putText(img, 'slow_guide_line', (int(bx1), max(12, int(by1) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.imwrite(os.path.join(dbg, stem + '.jpg'), img)
                shown += 1
    print('지그재그 라벨 %s: %d장%s | 가까이(%.1f m 안) 있었지만 뺀 이유 %s'
          % ('붙일 수 있는 사진' if a.dry else '추가', added, '' if a.dry else ' (확인 사진 %s)' % dbg,
             a.max_dist, why))


if __name__ == '__main__':
    main()
