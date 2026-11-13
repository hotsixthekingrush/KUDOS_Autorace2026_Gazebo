"""camera_perception_node: 카메라 영상 -> /perception/lane_error (ar_msgs/LaneError).

검출 로직은 lane_core.LaneCore에 있다 (limo_ros2_application/detect_line.py 기반).

predict_enable (odom 예측): 마지막 valid 차선 중심선을 odom 좌표에 고정해 두고, predict_rate_hz 타이머마다
현재 차 자세 기준 lateral/heading 으로 옮겨 발행한다 (stamp = 그 odom 시각). 카메라가 느리거나(가제보 3~4 Hz,
간격 최대 1.7 s) 프레임이 invalid여도 predict_max_sec 동안은 옮긴 값이 나간다. 그 뒤로는 valid=false.
가드/시작 확인의 직전 값 비교도 odom으로 옮겨서 한다 (LaneCore.motion).
"""
import bisect
import collections
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, String
from ar_msgs.msg import LaneError

from ar_lane_perception.lane_core import DEFAULTS, LaneCore


class CameraPerceptionNode(Node):
    def __init__(self):
        super().__init__('camera_perception_node')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('publish_distance_y', False)   # 기존 limo_control용 /distance_y
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_num', 2)          # 0: 마스크, 그 외: 검출 표시 영상
        # 프레임마다 판정 사유를 /perception/lane_status (std_msgs/String, JSON)로 발행 (분석용)
        self.declare_parameter('publish_status', True)
        self.declare_parameter('predict_enable', False)       # odom 예측 발행 (시뮬 yaml에서 true)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('predict_rate_hz', 20.0)
        self.declare_parameter('predict_max_sec', 1.0)        # 마지막 valid 프레임 뒤 이 시간까지만 옮긴 값 발행
        self.declare_parameter('predict_confidence', 0.25)    # 예측 중(마지막 카메라 프레임이 invalid) 발행 confidence
        # 예측 중 lateral 계산: True 면 마지막 valid 프레임의 가운데 경로 점들(odom 고정)을 차 기준으로 옮겨 reference_distance_m
        # 지점에서 다시 잰다 (곡률 하나로 휘게 하지 않음). 점 범위를 넘으면 끝 구간 방향으로 직선 연장.
        # False 면 예전처럼 직선 하나(lateral, heading)를 옮긴다
        self.declare_parameter('predict_path', True)
        self.declare_parameter('predict_ext_max_m', 0.2)      # 연장 길이가 이보다 길면 confidence 에 아래 값을 더 곱함
        self.declare_parameter('predict_ext_conf_scale', 0.5)
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.core = LaneCore({k: self.get_parameter(k).value for k in DEFAULTS})
        self.add_on_set_parameters_callback(self._on_params)

        self.br = CvBridge()
        self.lane_pub = self.create_publisher(LaneError, '/perception/lane_error', 10)
        self.dis_pub = None
        if self.get_parameter('publish_distance_y').value:
            self.dis_pub = self.create_publisher(Int32, 'distance_y', qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, 'debug_image', 10)
        self.status_pub = self.create_publisher(String, '/perception/lane_status', 10)
        topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, topic, self.image_callback, qos_profile_sensor_data)

        self.predict = bool(self.get_parameter('predict_enable').value)
        self.poses = collections.deque(maxlen=400)   # (t, x, y, yaw) odom
        self.line = None                             # 마지막 valid 중심선: (odom 점 A, 단위 방향 d, 촬영 시각, conf, single)
        self.pred_invalid_sent = True
        self.frame_valid = True       # 마지막 카메라 프레임 valid 여부 (False 인 동안 옮긴 값 = 예측)
        self.path_odom = None         # 마지막 valid 프레임 가운데 경로 점 (odom 좌표, 가까운 쪽부터)
        if self.predict:
            self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 50)
            self.core.motion = self.motion
            self.create_timer(1.0 / float(self.get_parameter('predict_rate_hz').value), self.predict_tick)
        p = self.core.p
        self.get_logger().info(
            '구독 %s -> /perception/lane_error (reference_distance_m=%.2f, 차선 폭 실측 %.2f~%.2f m, '
            '고정 폭 기본 %.2f / 코너 %.2f, color_space=%s, predict=%s)'
            % (topic, p['reference_distance_m'], p['width_min_m'], p['width_max_m'],
               p['lane_width_default_m'], p['lane_width_corner_m'], p['color_space'], self.predict))
        self.declare_parameter('width_log_sec', 5.0)          # 고정 폭 사용 프레임 수 로그 주기 [s]. 0이면 끔
        self._wlog_t = time.monotonic()

    # ------------------------------------------------------------------
    # odom 예측
    # ------------------------------------------------------------------
    def odom_callback(self, m):
        q, pp = m.pose.pose.orientation, m.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not all(math.isfinite(v) for v in (pp.x, pp.y, yaw)):
            return
        if self.poses and t <= self.poses[-1][0]:
            if t < self.poses[-1][0] - 1.0:      # 시간이 뒤로 감 (시뮬 리셋)
                self.poses.clear()
                self.line = None
            return
        self.poses.append((t, pp.x, pp.y, yaw))

    def pose_at(self, t):
        """t의 odom 자세 (선형 보간). 기록 범위 밖이면 가장 가까운 끝 (0.3 s 넘게 벗어나면 None)."""
        P = self.poses
        if not P:
            return None
        if t <= P[0][0]:
            return P[0][1:] if P[0][0] - t <= 0.3 else None
        if t >= P[-1][0]:
            return P[-1][1:] if t - P[-1][0] <= 0.3 else None
        i = bisect.bisect_left([a[0] for a in P], t)
        a, b = P[i - 1], P[i]
        k = (t - a[0]) / max(b[0] - a[0], 1e-9)
        dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        return (a[1] + k * (b[1] - a[1]), a[2] + k * (b[2] - a[2]), a[3] + k * dyaw)

    def _to_odom(self, lat, head, pose):
        """base_link 기준 중심선 (lat, head) -> odom 좌표 (점 A, 단위 방향 d)."""
        xr = float(self.core.p['reference_distance_m'])
        c1 = math.tan(head)
        c0 = lat * math.hypot(1.0, c1) - c1 * xr
        x, y, yaw = pose
        cs, sn = math.cos(yaw), math.sin(yaw)
        A = (x + cs * 0.0 - sn * c0, y + sn * 0.0 + cs * c0)          # 로봇 기준 (0, c0)
        d = (math.cos(yaw + head), math.sin(yaw + head))
        return A, d

    def _from_odom(self, A, d, pose):
        """odom 좌표 중심선 -> 그 자세 기준 (lat, head). 차선이 차 뒤로 돌아갔으면 None."""
        xr = float(self.core.p['reference_distance_m'])
        x, y, yaw = pose
        head = math.atan2(d[1], d[0]) - yaw
        head = math.atan2(math.sin(head), math.cos(head))
        if abs(head) > math.radians(80):
            return None
        cs, sn = math.cos(yaw), math.sin(yaw)
        ax, ay = A[0] - x, A[1] - y
        Ax, Ay = cs * ax + sn * ay, -sn * ax + cs * ay                # 로봇 좌표
        c1 = math.tan(head)
        c0 = Ay - c1 * Ax
        return (c0 + c1 * xr) / math.hypot(1.0, c1), head

    def motion(self, lat, head, t_from, t_to):
        a, b = self.pose_at(t_from), self.pose_at(t_to)
        if a is None or b is None:
            return None
        A, d = self._to_odom(lat, head, a)
        return self._from_odom(A, d, b)

    def _path_lateral(self, P, pose):
        """odom 고정 경로 점 P -> 자세 pose 기준 (lateral, heading, 연장 길이 m). 못 구하면 None.
        reference_distance_m 지점의 경로 y 와 방향. 그 지점이 점 범위 밖이면 가까운/먼 끝 구간 방향으로 직선 연장."""
        xr = float(self.core.p['reference_distance_m'])
        x, y, yaw = pose
        cs, sn = math.cos(yaw), math.sin(yaw)
        Q = [(cs * (px - x) + sn * (py - y), -sn * (px - x) + cs * (py - y)) for px, py in P]
        if len(Q) < 2:
            return None
        for (x0, y0), (x1, y1) in zip(Q[:-1], Q[1:]):
            if (x0 - xr) * (x1 - xr) <= 0.0 and abs(x1 - x0) > 1e-6:
                k = (xr - x0) / (x1 - x0)
                hd = math.atan2(y1 - y0, x1 - x0)
                return float((y0 + k * (y1 - y0)) * math.cos(hd)), float(hd), 0.0
        # 범위 밖: 먼 끝(xr 이 더 멀 때) 또는 가까운 끝 구간의 방향으로 연장
        far = abs(Q[-1][0] - xr) < abs(Q[0][0] - xr)
        (ax, ay), (bx, by) = (Q[-2], Q[-1]) if far else (Q[1], Q[0])
        dx, dy = bx - ax, by - ay
        if abs(dx) < 1e-6:
            return None
        hd = math.atan2(dy, dx) if far else math.atan2(-dy, -dx)
        if abs(hd) > math.radians(80):
            return None
        k = (xr - bx) / dx
        yr = by + k * dy
        return float(yr * math.cos(hd)), float(hd), float(abs(k) * math.hypot(dx, dy))

    def predict_tick(self):
        if not self.poses:
            return
        t_now, x, y, yaw = self.poses[-1]
        L = self.line
        out = LaneError()
        out.header.stamp.sec = int(t_now)
        out.header.stamp.nanosec = int((t_now - int(t_now)) * 1e9)
        out.header.frame_id = 'base_link'
        r = None
        ext = 0.0
        predicting = not self.frame_valid
        if L is not None and 0.0 <= t_now - L[2] <= float(self.get_parameter('predict_max_sec').value):
            if predicting and self.get_parameter('predict_path').value and self.path_odom is not None:
                pr = self._path_lateral(self.path_odom, (x, y, yaw))
                if pr is not None:
                    r, ext = pr[:2], pr[2]
            else:
                r = self._from_odom(L[0], L[1], (x, y, yaw))
        if r is None:
            if self.pred_invalid_sent:
                return                # invalid는 한 번만 (카메라 프레임 쪽에서도 invalid가 나감)
            self.pred_invalid_sent = True
            out.valid = False
        else:
            self.pred_invalid_sent = False
            out.valid = True
            out.lateral_offset_m, out.heading_error_rad = float(r[0]), float(r[1])
            conf = float(L[3])
            if predicting:
                conf = float(self.get_parameter('predict_confidence').value)
                if ext > float(self.get_parameter('predict_ext_max_m').value):
                    conf *= float(self.get_parameter('predict_ext_conf_scale').value)
            out.confidence = conf
            out.single_lane_estimated = bool(L[4])
            if predicting and self.get_parameter('publish_status').value:
                # 예측으로 낸 값도 lane_status 에 남긴다 (src=predict). predict_sec = 마지막 valid 카메라 프레임 뒤 지난 시간,
                # pred_beyond = 저장한 경로 점 범위를 넘어 연장했는지, pred_ext_m = 연장 길이
                self.status_pub.publish(String(data=json.dumps(dict(
                    src='predict', t=t_now, valid=True, predicting=True, predict_sec=round(t_now - L[2], 3),
                    pred_beyond=bool(ext > 0.0), pred_ext_m=round(float(ext), 3),
                    lat=float(out.lateral_offset_m), head=float(out.heading_error_rad), conf=float(out.confidence)))))
        self.lane_pub.publish(out)

    def _on_params(self, params):
        for prm in params:
            if prm.name == 'color_space' and prm.value not in ('hls', 'lab'):
                return SetParametersResult(successful=False,
                                           reason="color_space는 'hls' 또는 'lab'")
            if prm.name == 'single_path_mode' and prm.value not in ('line', 'polyline'):
                return SetParametersResult(successful=False,
                                           reason="single_path_mode는 'line' 또는 'polyline'")
        for prm in params:
            if prm.name in DEFAULTS:
                self.core.p[prm.name] = prm.value
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        want_dbg = (self.get_parameter('publish_debug_image').value
                    and self.get_parameter('debug_image_num').value != 0)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t0 = time.perf_counter()
        r = self.core.process(img, t, want_debug=want_dbg)
        proc_ms = (time.perf_counter() - t0) * 1e3

        out = LaneError()
        out.header.stamp = msg.header.stamp          # 영상 촬영 시각
        out.header.frame_id = 'base_link'
        out.valid = bool(r['valid'])
        # valid=false여도 계산값은 그대로 실어 둔다 (디버그용, 판단팀은 무시)
        out.lateral_offset_m = float(r['lateral'])
        out.heading_error_rad = float(r['heading'])
        out.curvature = 0.0                          # 아직 추정하지 않음
        out.confidence = float(r['confidence'])
        out.single_lane_estimated = bool(r['single'])
        pose = self.pose_at(t) if self.predict else None
        if pose is None:
            self.lane_pub.publish(out)               # 예전 동작 (예측 끔, 또는 odom 없음)
        else:
            self.frame_valid = bool(out.valid)
            if out.valid:
                A, d = self._to_odom(out.lateral_offset_m, out.heading_error_rad, pose)
                self.line = (A, d, t, out.confidence, out.single_lane_estimated)
                P = r.get('path')
                if P is not None and len(P) >= 2:
                    px, py, pyaw = pose
                    c, s_ = math.cos(pyaw), math.sin(pyaw)
                    self.path_odom = [(float(px + c * a - s_ * b), float(py + s_ * a + c * b)) for a, b in P]
                else:
                    self.path_odom = None
            # 발행은 predict_tick이 한다 (valid면 곧바로 새 값으로, invalid면 predict_max_sec까지 옮긴 값, 그 뒤 invalid)
            self.predict_tick()
        if self.get_parameter('publish_status').value:
            st = {'t': t, 'valid': out.valid, 'raw_valid': bool(r['raw_valid']),
                  'reason': r['guard'] or '', 'lat': out.lateral_offset_m,
                  'head': out.heading_error_rad, 'conf': out.confidence,
                  'n_one': int(r['n_one']), 'n_good': int(r['n_good']),
                  'proc_ms': round(proc_ms, 1), 'cs': self.core.p['color_space']}
            for k in ('blobs_removed', 'rows_dropped', 'white_rows', 'fork_rows', 'single_rejected'):
                st[k] = int(r.get(k, 0))
            # 차선 폭 실측: 프레임 종류(pair/single), 이번 폭, 기억 폭, 쓴 폭, 고정 폭 경우(a=가, b=나), 속도 제한 여부
            st.update(kind=r['kind'], lat_raw=float(r['lat_raw']), width_meas=r['width_meas'],
                      width_mem=r['width_mem'], width_used=float(r['width_used']), fixed=r['fixed'],
                      width_note=r['width_note'], slew=bool(r['slew']), single=bool(r['single']))
            # 예측: 이 카메라 프레임이 invalid 이고 마지막 valid 뒤 predict_max_sec 안이면 옮긴 값이 나가는 중
            L = self.line
            age = None if L is None else t - L[2]
            pred = (self.predict and not out.valid and age is not None
                    and 0.0 <= age <= float(self.get_parameter('predict_max_sec').value))
            st.update(src='camera', predicting=bool(pred), predict_sec=round(age, 3) if pred else 0.0,
                      corner_b=float(r['corner_b']), outer_off=bool(r.get('outer_off')))
            # 경로 읽는 방식: single_path_mode, 이번 프레임에 실제로 쓴 방식(polyline 이 못 읽으면 line), 선 각도(도),
            # 꺾은선 끝에서 이어 간 길이 [m]
            pi = r.get('poly') or {}
            st.update(single_path_mode=self.core.p['single_path_mode'],
                      path_used=None if not pi else ('line' if pi['fallback'] or pi['mode'] == 'line' else 'polyline'),
                      line_angle_deg=None if not pi else round(pi['ang'], 1),
                      poly_ext_m=None if not pi else round(float(pi['ext']), 3))
            self.status_pub.publish(String(data=json.dumps(st)))
        wl = float(self.get_parameter('width_log_sec').value)
        if wl > 0 and time.monotonic() - self._wlog_t >= wl:
            self._wlog_t = time.monotonic()
            c = self.core.counts
            self.get_logger().info(
                '차선 폭: 프레임 %d (두 선 %d, 한 선 %d) | 고정 폭 사용 (가) 실측 없음 %d, (나) 휜 선 %d | '
                '폭 범위 밖 %d, 속도 제한 %d | 기억 폭 %s'
                % (c['frames'], c['pair'], c['single'], c['fixed_a'], c['fixed_b'], c['width_reject'], c['slew'],
                   'none' if self.core.w_mem is None else '%.3f' % self.core.w_mem))
        if r['guard']:
            self.get_logger().warn('lane guard: %s (width_err=%s) -> valid=false'
                                   % (r['guard'], None if r['width_err'] is None
                                      else '%.2f' % r['width_err']),
                                   throttle_duration_sec=1.0)

        if self.dis_pub is not None:
            self.dis_pub.publish(Int32(data=r['gap']))
        if self.get_parameter('publish_debug_image').value:
            if self.get_parameter('debug_image_num').value == 0:
                mask = self.core.lane_mask(img)   # color_space에 따라 HLS / LAB
                self.debug_pub.publish(self.br.cv2_to_imgmsg(mask, 'mono8'))
            elif r['debug'] is not None:
                self.debug_pub.publish(self.br.cv2_to_imgmsg(r['debug'], 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
