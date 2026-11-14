"""Compare every published message to the reviewed main on synthetic scans.

This is deterministic message regression, not a Gazebo world or bag replay.
Requires the reviewed main commit in the local git repository.
"""
from copy import deepcopy
import math
from pathlib import Path
import subprocess
import types

import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from ar_perception.lidar_perception import LidarPerception


BASE = '46e57694b46873193f3e5f77dbba9e2fe2f9b657'
SOURCE = 'src/ar_perception/ar_perception/lidar_perception.py'


def scene(index):
    """Ray-cast stationary cone, car-sized circle, and two parking side walls."""
    x, y, yaw = 0.003*index, 0.0, 0.002*index
    odom = Odometry()
    odom.header.stamp.sec = index+1
    odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
    odom.pose.pose.orientation.z = math.sin(yaw/2)
    odom.pose.pose.orientation.w = math.cos(yaw/2)
    scan = LaserScan()
    scan.header.stamp = odom.header.stamp
    scan.header.frame_id = 'laser_link'
    scan.range_min, scan.range_max = 0.05, 10.0
    scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/1440
    sx, sy = x+0.1236*math.cos(yaw), y+0.1236*math.sin(yaw)
    ranges = []
    for i in range(1440):
        a = scan.angle_min+i*scan.angle_increment+yaw
        dx, dy = math.cos(a), math.sin(a)
        hits = []
        for cx, cy, radius in ((0.8, -0.4, 0.05), (1.6, -0.5, 0.15)):
            ox, oy = sx-cx, sy-cy
            b = ox*dx+oy*dy
            disc = b*b-(ox*ox+oy*oy-radius*radius)
            if disc >= 0:
                distance = -b-math.sqrt(disc)
                if distance > 0:
                    hits.append(distance)
        for wall_x in (0.6, 1.4):
            if abs(dx) > 1e-9:
                distance = (wall_x-sx)/dx
                if distance > 0 and 0.3 <= sy+distance*dy <= 0.8:
                    hits.append(distance)
        ranges.append(min(hits) if hits else math.inf)
    scan.ranges = ranges
    return odom, scan


def collect(node_class):
    node = node_class()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
    outputs = {key: [] for key in ('pub_obstacles', 'pub_event', 'pub_walls',
                                   'pub_parking_gap', 'pub_debug', 'pub_clusters')}
    try:
        for key in outputs:
            setattr(node, key, types.SimpleNamespace(publish=lambda msg, k=key:
                                                     outputs[k].append(deepcopy(msg))))
        for i in range(30):
            odom, scan = scene(i)
            node.odom_callback(odom)
            node.on_scan(scan)
        return outputs
    finally:
        node.destroy_node()


def test_gazebo_mode_matches_reviewed_main():
    repo = Path(__file__).resolve().parents[3]
    result = subprocess.run(['git', 'show', f'{BASE}:{SOURCE}'], cwd=repo,
                            capture_output=True, text=True)
    if result.returncode:
        pytest.skip('Reviewed main commit is not present in this checkout')
    baseline = types.ModuleType('reviewed_main_lidar')
    exec(compile(result.stdout, 'reviewed_main_lidar.py', 'exec'), baseline.__dict__)
    config = repo/'src/ar_perception/config/lidar.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(config)])
    try:
        before = collect(baseline.LidarPerception)
        after = collect(LidarPerception)
    finally:
        rclpy.shutdown()
    assert any(m.obstacles for m in after['pub_obstacles'])
    assert any(m.walls for m in after['pub_walls'])
    assert any(m.confirmed for m in after['pub_parking_gap'])
    for key in before:
        # Compare message fields: CDR alignment padding need not be deterministic.
        assert after[key] == before[key], key
