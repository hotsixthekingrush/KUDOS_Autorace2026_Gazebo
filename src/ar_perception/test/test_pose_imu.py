"""Numerical and timestamp failure cases without ROS."""
import math

import pytest

from ar_perception.pose_imu import ImuPose, quaternion_yaw


def feed(pose, t, yaw=0.0, v=1.0):
    pose.add_imu(t, yaw)
    pose.add_odom(t, v, t)
    pose.advance(t)


@pytest.mark.parametrize('speed', [1.0, -1.0, 0.0])
def test_straight_and_reverse_start_heading(speed):
    pose = ImuPose(max_imu_gap=0.11)
    for i in range(11):
        feed(pose, i/10, yaw=1.2, v=speed)
    assert pose.pose_at(1.0) == pytest.approx((speed+0.10, 0.0, 0.0))
    assert pose.pose_at(0.55) == pytest.approx((speed*0.55+0.10, 0.0, 0.0))


def test_quarter_circle_matches_analytic_solution():
    pose = ImuPose(capacity=1000)
    # radius 1 m, omega = 1 rad/s, initial absolute IMU yaw is not zero
    steps = 200
    for i in range(steps+1):
        t = i * math.pi/2 / steps
        feed(pose, t, yaw=0.8+t)
    assert pose.pose_at(math.pi/2) == pytest.approx((1, 1.1, math.pi/2), abs=1e-5)


def test_wrap_and_rotating_center_not_accumulated():
    pose = ImuPose(max_imu_gap=0.11)
    feed(pose, 1, yaw=math.radians(179), v=0)
    feed(pose, 1.1, yaw=math.radians(-179), v=0)
    angle = math.radians(2)
    expected = (0.1*math.cos(angle), 0.1*math.sin(angle), angle)
    for _ in range(10):
        assert pose.pose_at(1.1) == pytest.approx(expected)
    assert pose.pose_at(1.05)[2] == pytest.approx(math.radians(1))


def test_wait_for_future_imu_then_interpolate():
    pose = ImuPose(max_imu_gap=0.21)
    pose.add_imu(1, 0)
    pose.add_odom(1.1, 1, 10)
    pose.advance(10.05)
    assert pose.pose_at(1.1) is None
    pose.add_imu(1.2, 0.2)
    pose.advance(10.06)
    assert pose.yaw0 == pytest.approx(0.1)
    assert pose.pose_at(1.1) == pytest.approx((0.1, 0, 0))


def test_no_extrapolation_and_timeout_clears_old_history():
    pose = ImuPose()
    feed(pose, 1)
    generation = pose.generation
    pose.add_odom(1.1, 1, 10)
    pose.advance(10.11)
    assert pose.generation > generation
    assert pose.pose_at(1) is None
    assert pose.pose_at(1.1) is None
    feed(pose, 1.2)
    assert pose.pose_at(1.2) == pytest.approx((0.1, 0, 0))
    assert pose.pose_at(1.21) is None


def test_reverse_order_and_duplicate_do_not_change_clock_or_pose():
    pose = ImuPose(max_imu_gap=0.11)
    feed(pose, 1)
    feed(pose, 1.1)
    state = pose.state
    feed(pose, 1.05, yaw=2, v=100)
    feed(pose, 1.1, yaw=2, v=100)
    assert pose.state == state
    feed(pose, 1.2)
    assert pose.pose_at(1.2) == pytest.approx((0.3, 0, 0))


@pytest.mark.parametrize('field', ['velocity', 'yaw'])
def test_invalid_data_breaks_history(field):
    pose = ImuPose(max_imu_gap=0.11)
    feed(pose, 1)
    if field == 'velocity':
        pose.add_odom(1.1, float('nan'), 1.1)
    else:
        pose.add_imu(1.1, None)
    assert pose.pose_at(1) is None
    feed(pose, 1.2)
    assert pose.pose_at(1.2) == pytest.approx((0.1, 0, 0))


def test_imu_gap_breaks_even_if_endpoints_match_odom():
    pose = ImuPose(max_imu_gap=0.1)
    feed(pose, 1)
    feed(pose, 1.3)
    assert pose.pose_at(1.3) == pytest.approx((0.1, 0, 0))
    assert pose.pose_at(1.15) is None


def test_long_odom_gap_resets_without_guessing_distance():
    pose = ImuPose(max_dt=0.5)
    feed(pose, 1)
    for i in range(1, 21):
        pose.add_imu(1+i*0.05, 0)
    pose.add_odom(2, 1, 2)
    pose.advance(2)
    assert pose.pose_at(2) == pytest.approx((0.1, 0, 0))
    assert pose.pose_at(1) is None


def test_acceleration_uses_trapezoid_velocity():
    pose = ImuPose(max_imu_gap=0.11)
    feed(pose, 1, v=0)
    feed(pose, 1.1, v=2)
    assert pose.pose_at(1.1)[0] == pytest.approx(0.2)


def test_nominal_gap_limit_at_epoch_timestamps():
    pose = ImuPose()
    start = 1790000000.0
    for i in range(11):
        feed(pose, start+i/10)
    assert pose.pose_at(start+1) == pytest.approx((1.1, 0, 0))


def test_explicit_clock_reset_allows_new_epoch():
    pose = ImuPose()
    feed(pose, 20)
    pose.reset()
    feed(pose, 0)
    assert pose.pose_at(0) == pytest.approx((0.1, 0, 0))
    assert pose.pose_at(20) is None


def test_bounded_queues():
    pose = ImuPose(capacity=4)
    for i in range(20):
        pose.add_odom(i, 1, i)
    assert len(pose.pending) <= 4
    for i in range(20):
        pose.add_imu(i, 0)
    assert len(pose.imu) == 4


@pytest.mark.parametrize('quaternion', [(0, 0, 0, 0), (0, 0, math.nan, 1),
                                        (0, 0, math.inf, 1)])
def test_bad_quaternion(quaternion):
    assert quaternion_yaw(*quaternion) is None


def test_quaternion_normalized():
    assert quaternion_yaw(0, 0, 2, 2) == pytest.approx(math.pi/2)


@pytest.mark.parametrize('kwargs', [{'wait': 0}, {'max_dt': -1},
                                    {'max_imu_gap': math.nan}])
def test_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        ImuPose(**kwargs)
