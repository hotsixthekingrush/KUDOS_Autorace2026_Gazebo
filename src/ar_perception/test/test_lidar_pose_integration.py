"""ROS message/callback integration; no vehicle drivers or commands are used."""
from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from ar_msgs.msg import WallSegment
from ar_msgs.msg import ObstacleArray
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
import yaml

from ar_perception.lidar_perception import LidarPerception


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def make_node(monkeypatch):
    nodes = []
    def make(real=True):
        config = Path(__file__).resolve().parents[1] / 'config'
        params = yaml.safe_load((config/'lidar.yaml').read_text())['lidar_perception']['ros__parameters']
        if real:
            params.update(yaml.safe_load((config/'lidar_real.yaml').read_text())[
                'lidar_perception']['ros__parameters'])
        node = LidarPerception(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        nodes.append(node)
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        clock = SimpleNamespace(ros=1.0, steady=10.0)
        monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=round(clock.ros*1e9))))
        monkeypatch.setattr('ar_perception.lidar_perception.time.monotonic', lambda: clock.steady)
        outputs = {}
        for name in ('pub_obstacles', 'pub_event', 'pub_walls', 'pub_parking_gap',
                     'pub_debug', 'pub_clusters'):
            outputs[name] = []
            setattr(node, name, SimpleNamespace(publish=lambda msg, key=name:
                                                outputs[key].append(deepcopy(msg))))
        return node, clock, outputs
    yield make
    for node in nodes:
        node.destroy_node()


def stamp(msg, t):
    ns = round(t*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    return msg


def imu(t, yaw=0.0):
    msg = stamp(Imu(), t)
    msg.orientation.z, msg.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    return msg


def odom(t, v=1.0):
    msg = stamp(Odometry(), t)
    msg.twist.twist.linear.x = float(v)
    # Deliberately wrong global odom pose: IMU mode must not use it.
    msg.pose.pose.position.x = 100.0
    msg.pose.pose.orientation.w = 1.0
    return msg


def scan(t):
    msg = stamp(LaserScan(), t)
    msg.header.frame_id = 'laser'
    msg.range_min, msg.range_max = 0.05, 10.0
    msg.angle_min, msg.angle_increment = 0.0, 0.01
    msg.ranges = [1.0]
    return msg


def pair(node, clock, t, yaw=0, v=1):
    clock.ros = t
    node.imu_callback(imu(t, yaw))
    node.odom_callback(odom(t, v))


def test_real_pose_scan_stamp_and_offsets(make_node):
    node, clock, out = make_node()
    pair(node, clock, 1, yaw=0.7)
    pair(node, clock, 1.05, yaw=0.7)
    pair(node, clock, 1.10, yaw=0.7)
    msg = scan(1.025)
    node.on_scan(msg)
    assert node.get_odom_pose() == pytest.approx((0.125, 0, 0))
    point = node.to_base_link(msg)[0]
    assert point == pytest.approx((1.1025, 0))
    assert node.base_to_odom(*point) == pytest.approx((1.2275, 0))
    assert node.odom_to_base(*node.base_to_odom(*point)) == pytest.approx(point)
    assert msg.header.frame_id == 'laser'  # do not mutate the incoming scan
    for name in ('pub_obstacles', 'pub_walls', 'pub_parking_gap', 'pub_debug'):
        assert out[name][-1].header.frame_id == 'base_center'
        assert out[name][-1].header.stamp == msg.header.stamp


def test_scan_waits_then_uses_interpolated_pose(make_node):
    node, clock, out = make_node()
    pair(node, clock, 1)
    clock.ros = 1.025
    node.on_scan(scan(1.025))
    assert not out['pub_obstacles']
    clock.steady += 0.02
    pair(node, clock, 1.05)
    assert len(out['pub_obstacles']) == 1
    assert node.get_odom_pose() == pytest.approx((0.125, 0, 0))


def test_missing_imu_invalidates_parking_but_publishes_scan(make_node):
    node, clock, out = make_node()
    node.parking_boundaries = [{'x': 1, 'y': 1}]
    node.confirmed_parking_gap = {'center_x': 1, 'center_y': 1, 'length': 1}
    node.odom_callback(odom(1))
    node.on_scan(scan(1))
    clock.steady += 0.2
    node._drain_pose()
    assert not out['pub_parking_gap'][-1].valid
    assert not out['pub_parking_gap'][-1].confirmed
    assert out['pub_parking_gap'][-1].gap_id == 0
    assert not node.parking_boundaries
    assert node.confirmed_parking_gap is None
    assert len(out['pub_obstacles']) == 1


def test_stale_imu_cannot_integrate_new_odom(make_node):
    node, clock, out = make_node()
    pair(node, clock, 1)
    clock.ros = 1.05
    node.odom_callback(odom(1.05))
    node.on_scan(scan(1.05))
    clock.steady += 0.2
    node._drain_pose()
    assert not out['pub_parking_gap'][-1].valid
    assert node.imu_pose.pose_at(1.05) is None
    pair(node, clock, 1.1)
    node.on_scan(scan(1.1))
    assert node.get_odom_pose() == pytest.approx((0.1, 0, 0))


def test_clock_rewind_clears_queue_history_and_landmarks(make_node):
    node, clock, out = make_node()
    pair(node, clock, 1)
    node.on_scan(scan(1.01))
    node.parking_boundaries = [{'x': 5}]
    clock.ros = 0
    node._drain_pose()
    assert not node.parking_boundaries
    assert not node._scan_queue
    assert node.imu_pose.pose_at(1) is None
    pair(node, clock, 0)
    node.on_scan(scan(0))
    assert node.get_odom_pose() == pytest.approx((0.1, 0, 0))


def test_frame_recovery_reconfirms_gap(make_node, monkeypatch):
    node, clock, out = make_node()
    walls = []
    for x in (0.5, 1.3):
        wall = WallSegment()
        wall.x1 = wall.x2 = x
        wall.y1, wall.y2 = 0.3, 0.6
        wall.role = WallSegment.ROLE_SIDE
        walls.append(wall)
    monkeypatch.setattr(node, 'detect_parking_walls', lambda points: walls)
    for t in (1, 1.05, 1.1):
        pair(node, clock, t, v=0)
        node.on_scan(scan(t))
    assert out['pub_parking_gap'][-1].confirmed
    old_id = out['pub_parking_gap'][-1].gap_id
    clock.ros = 1.2
    node.on_scan(scan(1.2))
    clock.steady += 0.2
    node._drain_pose()
    assert not out['pub_parking_gap'][-1].valid
    for i, t in enumerate((1.3, 1.35, 1.4)):
        pair(node, clock, t, v=0)
        node.on_scan(scan(t))
        assert out['pub_parking_gap'][-1].confirmed == (i == 2)
    assert out['pub_parking_gap'][-1].gap_id > old_id


def test_sim_uses_raw_pose_and_no_imu_subscription(make_node):
    node, clock, out = make_node(real=False)
    msg = odom(1)
    msg.pose.pose.position.y = 2.0
    node.odom_callback(msg)
    node.on_scan(scan(1))
    assert node.get_odom_pose() == pytest.approx((100, 2, 0))
    assert node.to_base_link(scan(1))[0] == pytest.approx((1.1236, 0))
    assert all(sub.topic_name != '/imu' for sub in node.subscriptions)
    assert out['pub_obstacles'][-1].header.frame_id == 'base_link'


def test_bad_orientation_covariance_and_clock_mismatch(make_node):
    node, clock, out = make_node()
    pair(node, clock, 1)
    bad = imu(1.05)
    bad.orientation_covariance[0] = -1.0
    node.imu_callback(bad)
    assert node.imu_pose.pose_at(1) is None
    node.on_scan(scan(1000))
    assert not out['pub_parking_gap'][-1].valid


def test_mode_and_geometry_cannot_change_mid_run(make_node):
    node, _, _ = make_node()
    for name, value in [('use_imu_yaw', False), ('lidar_offset_x', 0.1236),
                        ('odom_center_offset_x', 0.0)]:
        assert not node.set_parameters([Parameter(name, value=value)])[0].successful


def test_actual_ros_topics_with_best_effort_imu():
    """Exercise DDS subscriptions/timer/publisher, in an isolated ROS domain."""
    node = LidarPerception(parameter_overrides=[
        Parameter('use_imu_yaw', value=True),
        Parameter('odom_center_offset_x', value=0.1),
        Parameter('lidar_offset_x', value=0.1025),
        Parameter('output_frame', value='base_center')])
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
    driver = Node('lidar_test_sensor_source')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    imu_pub = driver.create_publisher(Imu, '/imu', qos_profile_sensor_data)
    odom_pub = driver.create_publisher(Odometry, '/odom', 10)
    scan_pub = driver.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    received = []
    driver.create_subscription(ObstacleArray, '/perception/obstacles', received.append, 10)
    deadline = time.monotonic()+5.0
    last_sent = float('-inf')
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now-last_sent >= 0.03:
                t = driver.get_clock().now().nanoseconds*1e-9
                imu_pub.publish(imu(t, 0.7))
                odom_pub.publish(odom(t, 0))
                scan_pub.publish(scan(t))
                last_sent = now
            executor.spin_once(timeout_sec=0.005)
            if received and node.get_odom_pose() is not None:
                break
        assert received
        assert received[-1].header.frame_id == 'base_center'
        assert node.get_odom_pose() == pytest.approx((0.1, 0, 0))
    finally:
        executor.shutdown()
        driver.destroy_node()
        node.destroy_node()
