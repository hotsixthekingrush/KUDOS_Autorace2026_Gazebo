"""Timestamped rear-axle dead reckoning for local LiDAR parking history.

No TF or global odom is published. Clock resets must call reset() explicitly;
out-of-order sensor packets alone never rewind the integration clock.
"""

from bisect import bisect_left
from collections import deque
import math

# Header seconds represented as floats lose sub-microsecond precision at epoch
# timestamps. Do not interpret a nominal 100 ms interval as a sensor outage.
TIME_EPSILON = 1e-6


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x, y, z, w):
    """Normalize a finite nonzero quaternion, returning None if invalid."""
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        return None
    norm = math.hypot(x, y, z, w)
    if norm < 1e-6 or not math.isfinite(norm):
        return None
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


class ImuPose:
    """Bounded IMU/odom queues and rear pose history, with no extrapolation.

    Sensor times are header seconds. ``now`` is monotonic receipt time, used
    only for bounded queue waiting, never for integration. Generation changes
    invalidate all downstream landmarks. Velocity is body-forward rear-axle v.
    """

    def __init__(self, max_dt=0.5, max_imu_gap=0.1, wait=0.1,
                 center_offset=0.1, capacity=400):
        if not all(math.isfinite(v) and v > 0 for v in (max_dt, max_imu_gap, wait)):
            raise ValueError('pose timing limits must be finite and positive')
        if not math.isfinite(center_offset) or capacity < 2:
            raise ValueError('invalid pose offset or capacity')
        self.max_dt = max_dt
        self.max_imu_gap = max_imu_gap
        self.wait = wait
        self.center_offset = center_offset
        self.capacity = capacity
        self.generation = 0
        self.reset()

    def reset(self):
        self.imu = deque(maxlen=self.capacity)
        self.pending = deque()
        self.last_odom_stamp = None
        self.invalidate()

    def invalidate(self):
        self.state = None
        self.yaw0 = None
        self.history = deque(maxlen=self.capacity)
        self.generation += 1

    def add_imu(self, t, yaw):
        if not math.isfinite(t) or t < 0:
            return
        if self.imu and t <= self.imu[-1][0]:
            return
        if yaw is None or not math.isfinite(yaw):
            self.imu.clear()
            self.pending.clear()
            self.invalidate()
            return
        self.imu.append((t, yaw))

    def add_odom(self, t, v, now):
        if not math.isfinite(t) or t < 0:
            return
        if self.last_odom_stamp is not None and t <= self.last_odom_stamp:
            return
        self.last_odom_stamp = t
        if not math.isfinite(v):
            self.pending.clear()
            self.invalidate()
            return
        if len(self.pending) >= self.capacity:
            self.pending.clear()
            self.invalidate()
        self.pending.append((t, v, now))

    def _yaw_at(self, t):
        samples = list(self.imu)
        i = bisect_left([p[0] for p in samples], t)
        if i < len(samples) and samples[i][0] == t:
            return samples[i][1]
        if i == 0 or i == len(samples):
            return None
        t0, y0 = samples[i-1]
        t1, y1 = samples[i]
        if t1 - t0 > self.max_imu_gap + TIME_EPSILON:
            return None
        return wrap(y0 + (t-t0)/(t1-t0) * wrap(y1-y0))

    def advance(self, now):
        while self.pending:
            t, v, received = self.pending[0]
            yaw = self._yaw_at(t)
            if yaw is None:
                if now - received < self.wait:
                    break
                self.pending.popleft()
                self.invalidate()
                continue
            self.pending.popleft()
            if self.state is not None:
                t0 = self.state[0]
                samples = list(self.imu)
                continuous = samples and samples[0][0] <= t0
                continuous = continuous and not any(
                    b[0] > t0 and a[0] < t
                    and b[0]-a[0] > self.max_imu_gap + TIME_EPSILON
                    for a, b in zip(samples, samples[1:]))
                if t-t0 > self.max_dt + TIME_EPSILON or not continuous:
                    self.invalidate()
            if self.state is None:
                self.yaw0 = yaw
                x, y, heading = 0.0, 0.0, 0.0
            else:
                t0, x, y, heading0, v0 = self.state
                heading = wrap(yaw-self.yaw0)
                mid = heading0 + 0.5 * wrap(heading-heading0)
                distance = 0.5 * (v0+v) * (t-t0)
                x += distance * math.cos(mid)
                y += distance * math.sin(mid)
            self.state = (t, x, y, heading, v)
            self.history.append((t, x, y, heading))

    def pose_at(self, t):
        """Interpolate rear pose then apply the rotating center lever arm once."""
        if not math.isfinite(t):
            return None
        samples = list(self.history)
        i = bisect_left([p[0] for p in samples], t)
        if i < len(samples) and samples[i][0] == t:
            _, x, y, yaw = samples[i]
        elif i == 0 or i == len(samples):
            return None
        else:
            t0, x0, y0, a0 = samples[i-1]
            t1, x1, y1, a1 = samples[i]
            ratio = (t-t0)/(t1-t0)
            x, y = x0+ratio*(x1-x0), y0+ratio*(y1-y0)
            yaw = wrap(a0+ratio*wrap(a1-a0))
        return (x+self.center_offset*math.cos(yaw),
                y+self.center_offset*math.sin(yaw), yaw)
