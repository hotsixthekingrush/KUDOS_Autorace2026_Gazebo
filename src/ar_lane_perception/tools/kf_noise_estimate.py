#!/usr/bin/env python3
"""차선 노드 칼만 잡음 값 추정 (실차 녹화용, lane-kalman).

1) 녹화 (차에서. 차선 노드를 켜고 칼만을 끈 채 차선을 따라 천천히 몰거나 손으로 밀면서):
     ros2 param set /camera_perception_node lane_filter none
     python3 tools/kf_noise_estimate.py --record ~/lane_status_real.jsonl 120
   /perception/lane_status (영상 프레임마다 JSON) 를 120 초 동안 파일로 저장한다.
2) 추정 (어디서든):
     python3 tools/kf_noise_estimate.py ~/lane_status_real.jsonl
   두 선 / 한 선 프레임별 측정 잡음 표준편차를 추정해, 넣을 값을 ros2 param set 명령과 yaml 줄로 출력한다.

추정 방법: 이어진 세 프레임(간격 0.3 s 이하, 같은 종류, valid)의 2차 차분 d = x[i+1] - 2 x[i] + x[i-1].
  차선 값이 프레임 사이에 거의 직선으로 변하면 d 에는 측정 잡음만 남고 var(d) = 6 sigma^2 -> sigma = std(d) / sqrt(6).
  코너처럼 값이 빨리 바뀌는 곳이 섞이면 크게 나오므로 중앙값 기반(1.4826 x MAD)으로 튀는 값의 영향을 줄인다.
  lateral 은 lane_status 의 lat_raw (전환 섞기·칼만 전 값) 를 쓴다.
  예측 잡음 q 는 가제보에서 고른 S4 의 q / r^2 비를 그대로 써서 r 에서 정한다 (반응 속도를 가제보와 같게 유지).
"""
import json
import math
import sys
import time

Q_RATIO = 0.005 / 0.03 ** 2      # 가제보 S4: kf_q_lat / kf_r_lat^2 (kf_q_head / kf_r_head^2 도 거의 같음, 5.6 /s)


def record(path, sec):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    n = Node('kf_noise_record')
    f = open(path, 'w')
    cnt = [0]

    def cb(m):
        f.write(m.data.strip() + '\n')
        cnt[0] += 1
    n.create_subscription(String, '/perception/lane_status', cb, 50)
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.2)
    f.close()
    print('%s: %d 줄 저장 (%.0f s)' % (path, cnt[0], time.monotonic() - t0))


def robust_std(v):
    if len(v) < 5:
        return None
    s = sorted(v)
    med = s[len(s) // 2]
    mad = sorted(abs(x - med) for x in v)[len(v) // 2]
    return 1.4826 * mad


def estimate(path):
    S = []
    for line in open(path):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if d.get('src', 'camera') != 'camera' or 't' not in d:
            continue
        S.append(d)
    S.sort(key=lambda d: d['t'])
    out = {}
    for kind in ('pair', 'single'):
        for key in ('lat', 'head'):
            k2 = 'lat_raw' if key == 'lat' else 'head'
            dd = []
            for a, b, c in zip(S, S[1:], S[2:]):
                if not (a.get('valid') and b.get('valid') and c.get('valid')):
                    continue
                if not (a.get('kind') == b.get('kind') == c.get('kind') == kind):
                    continue
                if c['t'] - b['t'] > 0.3 or b['t'] - a['t'] > 0.3:
                    continue
                if any(k2 not in x for x in (a, b, c)):
                    continue
                dd.append(c[k2] - 2 * b[k2] + a[k2])
            s = robust_std(dd)
            out[(kind, key)] = (None if s is None else s / math.sqrt(6.0), len(dd))
    n_valid = sum(1 for d in S if d.get('valid'))
    dts = sorted(b['t'] - a['t'] for a, b in zip(S, S[1:]) if 0 < b['t'] - a['t'] < 1.0)
    print('프레임 %d (valid %d), 프레임 간격 중간값 %s' % (len(S), n_valid, '%.0f ms' % (1e3 * dts[len(dts) // 2]) if dts else '-'))
    for (kind, key), (s, n) in out.items():
        unit = 'm' if key == 'lat' else 'rad'
        print('  %-6s %-4s 측정 잡음 표준편차: %s (%d 묶음)' % (kind, key, '-' if s is None else '%.4f %s' % (s, unit), n))
    rl, rh = out[('pair', 'lat')][0], out[('pair', 'head')][0]
    sl = out[('single', 'lat')][0]
    if rl is None or rh is None:
        print('두 선 프레임이 모자라 값을 정하지 못함 (두 선이 보이는 곳에서 더 녹화할 것)')
        return
    sc = 2.0 if sl is None else min(max(sl / max(rl, 1e-6), 1.0), 4.0)
    rl, rh = max(rl, 0.01), max(rh, 0.03)        # 너무 작게 잡히지 않게 바닥값
    vals = dict(kf_r_lat=round(rl, 4), kf_r_head=round(rh, 4), kf_q_lat=round(Q_RATIO * rl ** 2, 5),
                kf_q_head=round(Q_RATIO * rh ** 2, 5), kf_r_single_scale=round(sc, 2))
    print('\n넣을 값 (측정 잡음 = 추정값, 예측 잡음 = 가제보 S4 비율, 한 선 배수 = 한 선 / 두 선 잡음 비):')
    for k, v in vals.items():
        print('ros2 param set /camera_perception_node %s %s' % (k, v))
    print('\nyaml (safe3_hls.yaml 등에 붙여 넣기):')
    for k, v in vals.items():
        print('    %s: %s' % (k, v))


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == '--record':
        record(sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 120.0)
    elif len(sys.argv) == 2:
        estimate(sys.argv[1])
    else:
        print(__doc__)
