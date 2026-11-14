"""odom 속도 + IMU 방향으로 차 위치 적분 (11/14, 판단팀).

실차 WeGo 아커만 드라이버(limo_base_ackerman.launch.py)의 /odom 방향은 돌 때 실제보다 약 30% 크게 나온다
(판단팀 11/13 실측: 같은 주행에서 IMU 105도 vs /odom 136도, 자이로 적분 105도). 그래서 /odom 위치(x, y)도 커브를
돌고 나면 틀어진다. 바퀴 속도·거리는 정확하다 (1.6 m 직진 두 번, 줄자와 0.5% 안).
그래서 /odom 의 pose 대신 /odom 속도(twist.linear.x)를 IMU 방향으로 직접 적분한다.
판단(ar_mission_supervisor supervisor._dead_reckon)과 같은 방식이다. ROS 없이도 돌아가는 순수 파이썬.
"""
import math


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class OdomImuPose(object):
    """update(t, v) 를 /odom 마다 부르면 (x, y, yaw) 를 돌려줌. IMU 가 오기 전엔 None.
    위치 시작점은 처음 update 때 (0, 0) (예측은 상대 좌표만 써서 상관없음).
    두 /odom 사이가 max_dt 넘게 벌어지면 그 사이는 안 적분함 (멈췄다 다시 온 경우 튀지 않게)."""

    def __init__(self, max_dt=0.5):
        self.max_dt = max_dt
        self.yaw = None        # 최신 IMU 방향 [rad]
        self.state = None      # [x, y, yaw, t]

    def set_imu(self, qx, qy, qz, qw):
        yaw = quat_to_yaw(qx, qy, qz, qw)
        if math.isfinite(yaw):
            self.yaw = yaw

    def reset(self):
        self.state = None

    def update(self, t, v):
        if self.yaw is None:
            return None
        yaw = self.yaw
        if self.state is None:
            self.state = [0.0, 0.0, yaw, t]
        else:
            x, y, yaw0, t0 = self.state
            dt = t - t0
            if 0.0 < dt < self.max_dt and math.isfinite(v):
                mid = yaw0 + 0.5 * math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))
                x += v * math.cos(mid) * dt
                y += v * math.sin(mid) * dt
            self.state = [x, y, yaw, t]
        return self.state[0], self.state[1], yaw
