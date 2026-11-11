import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
try:
    # ar_msgs는 판단팀 메시지. 이 PC에는 임시 패키지(src/ar_msgs)만 있음.
    # 없거나 빌드 전이면 lane_error만 끄고 기존 출력은 그대로 동작.
    from ar_msgs.msg import LaneError
except ImportError:
    LaneError = None


class DetectLine(Node):
    def __init__(self):
        super().__init__('detect_line')
        self.br = CvBridge()
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self.image_callback,
                                 rclpy.qos.qos_profile_sensor_data)
        self.dis_publisher = self.create_publisher(Int32, 'distance_y', rclpy.qos.qos_profile_sensor_data)
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 10)

        d = self.declare_parameter
        d('lane_h_l', 0); d('lane_l_l', 90); d('lane_s_l', 100)
        d('lane_h_h', 60); d('lane_l_h', 220); d('lane_s_h', 255)
        d('side', 'both')         # both | left | right
        d('center_x', 320)        # 처음 시작할 때만 좌/우 선을 가르는 기준
        d('y_top', 320)
        d('y_bot', 470)
        d('n_rows', 8)
        d('horizon_y', 260)
        d('lane_w_bottom', 560)
        d('smooth', 0.7)          # 클수록 빠르게 반응
        d('max_step', 100.0)      # 프레임당 gap 최대 변화 (너무 작으면 반응이 늦어짐)
        d('track_tol', 90)        # 위/아래 줄에서 같은 선으로 인정하는 거리(px)
        d('track_predict', False) # True: 아래 두 줄의 기울기로 다음 줄 위치를 예측해서 추적 (False=원본)
        d('lost_hold', 5)
        d('debug_image_num', 2)
        # ---- /perception/lane_error (기존 /distance_y 계산과 별개 경로) ----
        d('publish_lane_error', True)
        d('lane_width_m', 0.55)   # 선 중심 간 차선 폭 (규정집 0.55, 도면 0.50, 실물 미측정). 시뮬은 detect_line_sim.yaml에서 0.52
        d('reference_distance_m', 0.5)  # lateral 기준점: base_link 원점(차 중심)에서 전방 m. 0 = base_link 원점
        d('cam_height_m', 0.21)   # 지면~카메라 광심
        d('cam_x_m', 0.10)        # base_link -> 카메라 전방 거리
        d('cam_pitch_rad', 0.0)   # 아래로 숙이면 +
        d('focal_px', 381.0)      # hfov 80deg, 640px -> 320/tan(40deg)

        self.lane_pub = None
        if LaneError is not None and self.get_parameter('publish_lane_error').value:
            self.lane_pub = self.create_publisher(LaneError, '/perception/lane_error', 10)
        elif LaneError is None:
            self.get_logger().warn('ar_msgs 없음: /perception/lane_error 비활성 (기존 출력은 정상)')
        self.wrow = {}
        self.prevL = {}
        self.prevR = {}
        self.gap_f = 0.0
        self.lost = 0

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
        if pl is None and pr is None:                 # 첫 시작: 가운데 기준으로만 나눔
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

    def image_callback(self, msg):
        g = lambda n: self.get_parameter(n).value
        img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]

        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hls,
                           np.array([g('lane_h_l'), g('lane_l_l'), g('lane_s_l')]),
                           np.array([g('lane_h_h'), g('lane_l_h'), g('lane_s_h')]))

        cx0 = int(g('center_x'))
        side = g('side')
        hz = g('horizon_y')
        tol = g('track_tol')
        # 가까운 줄(아래) -> 먼 줄(위) 순서로 위로 따라 올라감
        ys = np.linspace(g('y_bot'), g('y_top'), int(g('n_rows'))).astype(int)

        dbg = img.copy()
        dbg[mask > 0] = (0, 0, 255)
        cv2.line(dbg, (cx0, 0), (cx0, h), (0, 255, 0), 2)

        centers, weights, modes = [], [], ''
        newL, newR = {}, {}
        lastL = lastR = None
        l2 = r2 = None            # lastL/lastR 바로 전에 찾은 L/R (기울기 예측용)
        predict = bool(g('track_predict'))
        for y in ys:
            y = int(y)
            cv2.line(dbg, (0, y), (w, y), (90, 90, 90), 1)
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
                # 선이 비스듬하면 줄마다 크게 움직여 track_tol을 넘으므로 직선으로 연장해 예측
                if lastL is not None and l2 is not None:
                    pl = 2 * lastL - l2
                if lastR is not None and r2 is not None:
                    pr = 2 * lastR - r2
            L, R = self.find_row(means, cx0, side, exp, tol, pl, pr)

            c = None
            if L is not None and R is not None:
                wd = R - L
                if 0.6 * exp <= wd <= 1.4 * exp:
                    c = (L + R) / 2.0
                    modes += 'D'
                    self.wrow[y] = 0.8 * exp + 0.2 * wd
                else:
                    # 폭이 이상하면 직전 위치와 더 잘 이어지는 선 하나만 믿음
                    dl = abs(L - pl) if pl is not None else 1e9
                    dr = abs(R - pr) if pr is not None else 1e9
                    if dl <= dr:
                        c = L + exp / 2.0; R = None; modes += 'L'
                    else:
                        c = R - exp / 2.0; L = None; modes += 'R'
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
                cv2.circle(dbg, (int(L), y), 4, (255, 255, 0), -1)
            if R is not None:
                newR[y] = R
                r2, lastR = lastR, R
                cv2.circle(dbg, (int(R), y), 4, (0, 165, 255), -1)
            centers.append(c)
            weights.append(1.0 + (g('y_bot') - y) / max(g('y_bot') - g('y_top'), 1))
            cv2.circle(dbg, (int(c), y), 4, (255, 0, 0), -1)

        if centers:
            self.prevL, self.prevR = newL, newR
            tgt = float(np.average(centers, weights=weights))
            gap = cx0 - tgt
            step = g('smooth') * (gap - self.gap_f)
            ms = g('max_step')
            self.gap_f += float(np.clip(step, -ms, ms))
            self.lost = 0
            cv2.circle(dbg, (int(tgt), int(ys.mean())), 10, (255, 0, 0), -1)
            text, color = '%s  gap %+d' % (modes, round(self.gap_f)), (0, 255, 0)
        else:
            self.lost += 1
            if self.lost > g('lost_hold'):
                self.gap_f = 0.0
                self.prevL, self.prevR = {}, {}
            text, color = 'NO LANE (lost %d)' % self.lost, (0, 0, 255)
        cv2.putText(dbg, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        out = Int32()
        out.data = int(round(self.gap_f))
        self.dis_publisher.publish(out)

        if g('debug_image_num') == 0:
            self.debug_publisher.publish(self.br.cv2_to_imgmsg(mask, 'mono8'))
        else:
            self.debug_publisher.publish(self.br.cv2_to_imgmsg(dbg, 'bgr8'))

        # 위 기존 출력이 모두 나간 뒤에만 실행. newL/newR은 읽기만 함.
        if self.lane_pub is not None:
            try:
                self.publish_lane_error(msg, newL, newR, w, h)
            except Exception as e:  # 새 경로의 오류가 기존 동작을 멈추지 않게
                self.get_logger().error('lane_error 계산 실패: %s' % e, throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # /perception/lane_error : 지면 좌표(base_link, x 전방 / y 왼쪽 +)에서 계산
    # ------------------------------------------------------------------
    def pix_to_base(self, u, v, w, h):
        """영상 (u, v) -> base_link 지면 좌표 (x 전방, y 왼쪽). 지평선 위면 None."""
        p = lambda n: self.get_parameter(n).value
        f, hc, th = p('focal_px'), p('cam_height_m'), p('cam_pitch_rad')
        xc, yc = (u - w / 2.0) / f, (v - h / 2.0) / f      # 카메라: x 오른쪽, y 아래, z 전방
        down = yc * np.cos(th) + np.sin(th)
        fwd = np.cos(th) - yc * np.sin(th)
        if down <= 1e-6:
            return None
        t = hc / down
        return np.array([t * fwd + p('cam_x_m'), -t * xc])

    def lane_error(self, newL, newR, w, h):
        """반환: lateral_offset_m, heading_error_rad, valid, confidence, 중심점들"""
        half = self.get_parameter('lane_width_m').value / 2.0
        n_rows = max(int(self.get_parameter('n_rows').value), 1)
        G = {}
        for side, pts in (('L', newL), ('R', newR)):
            G[side] = {y: self.pix_to_base(u, y, w, h) for y, u in pts.items()}
            G[side] = {y: q for y, q in G[side].items() if q is not None}
        C, W = [], []
        n_both = n_one = 0
        for y in sorted(set(G['L']) | set(G['R']), reverse=True):
            if y in G['L'] and y in G['R']:                 # 두 선: 같은 거리의 중점
                C.append((G['L'][y] + G['R'][y]) / 2.0); W.append(1.0); n_both += 1
                continue
            side = 'L' if y in G['L'] else 'R'
            ys = sorted(G[side], reverse=True)              # 가까운 줄 -> 먼 줄
            k = ys.index(y)
            a = G[side][ys[k - 1]] if k > 0 else G[side][y]
            b = G[side][ys[k + 1]] if k + 1 < len(ys) else G[side][y]
            t = b - a
            nt = np.hypot(*t)
            t = np.array([1.0, 0.0]) if nt < 1e-6 else t / nt   # 점 하나뿐이면 전방으로 가정
            left = np.array([-t[1], t[0]])                  # 진행방향의 왼쪽 법선
            # 왼쪽 선이면 중심은 오른쪽(−left), 오른쪽 선이면 왼쪽(+left)
            C.append(G[side][y] + (-left if side == 'L' else left) * half)
            W.append(0.5); n_one += 1
        if len(C) < 2:
            return 0.0, 0.0, False, 0.0, C
        C, W = np.array(C), np.array(W)
        span = C[:, 0].max() - C[:, 0].min()
        if span < 0.10:
            return 0.0, 0.0, False, 0.0, C
        A = np.c_[np.ones(len(C)), C[:, 0]]
        sw = np.sqrt(W)
        coef = np.linalg.lstsq(A * sw[:, None], C[:, 1] * sw, rcond=None)[0]
        rms = float(np.sqrt(np.average((A @ coef - C[:, 1]) ** 2, weights=W)))
        heading = float(np.arctan(coef[1]))                 # 중심선 방향 (왼쪽으로 꺾이면 +)
        # 기준점 (x_ref, 0) -> 맞춘 중심선까지 수직거리 (왼쪽 +). x_ref = 0이면 base_link 원점, 기본 0.5 m 전방
        x_ref = float(self.get_parameter('reference_distance_m').value)
        lateral = float((coef[0] + coef[1] * x_ref) / np.hypot(1.0, coef[1]))
        # confidence: 쓸 수 있는 행 비율 x 직선맞춤 잔차 x 거리 범위 x 시야 안에서 휜 정도
        coverage = (n_both + 0.5 * n_one) / n_rows
        conf = coverage * np.exp(-(rms / 0.03) ** 2) * min(span / 0.4, 1.0)
        if len(C) >= 4:   # 가까운 3점의 방향과 전체 방향 차이가 크면 base_link로의 외삽을 믿기 어려움
            near = np.argsort(C[:, 0])[:3]
            bn = np.linalg.lstsq(A[near] * sw[near, None], C[near, 1] * sw[near], rcond=None)[0][1]
            conf *= np.exp(-(np.degrees(abs(np.arctan(bn) - np.arctan(coef[1]))) / 10.0) ** 2)
        return lateral, heading, True, float(np.clip(conf, 0.0, 1.0)), C

    def publish_lane_error(self, img_msg, newL, newR, w, h):
        lat, head, valid, conf, _ = self.lane_error(newL, newR, w, h)
        out = LaneError()
        out.header.stamp = img_msg.header.stamp             # 촬영 시각
        out.header.frame_id = 'base_link'
        out.lateral_offset_m = lat
        out.heading_error_rad = head
        out.valid = valid
        out.confidence = conf
        self.lane_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectLine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
