"""odom_imu.OdomImuPose 시험 (ROS 없이). python3 -m pytest test/test_odom_imu.py 또는 python3 test/test_odom_imu.py"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from ar_lane_perception.odom_imu import OdomImuPose  # noqa: E402


def _q(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def test_none_before_imu():
    assert OdomImuPose().update(0.0, 0.2) is None


def test_straight():
    d = OdomImuPose()
    d.set_imu(*_q(0.0))
    for i in range(101):
        p = d.update(i * 0.05, 0.2)
    assert abs(p[0] - 1.0) < 1e-6 and abs(p[1]) < 1e-9


def test_circle_closes():
    d = OdomImuPose()
    v, w, dt = 0.2, 0.4, 0.02
    for i in range(int(round(2 * math.pi / w / dt)) + 1):
        t = i * dt
        d.set_imu(*_q(w * t))
        p = d.update(t, v)
    R = v / w
    assert math.hypot(p[0] - R * math.sin(w * t), p[1] - R * (1 - math.cos(w * t))) < 0.01


def test_gap_not_integrated():
    d = OdomImuPose()
    d.set_imu(*_q(0.0))
    d.update(0.0, 0.2)
    d.update(0.1, 0.2)
    p = d.update(1.1, 0.2)
    assert abs(p[0] - 0.02) < 1e-9


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
