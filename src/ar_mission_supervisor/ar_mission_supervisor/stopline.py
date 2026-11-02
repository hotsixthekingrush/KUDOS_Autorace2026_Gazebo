"""정지선까지 남은 거리 추정. SLOW_ZONE, CROSSWALK_STOP, TRAFFIC_LIGHT 가 공유한다.

노드에 하나만 두고(node.stopline) 여러 State 가 이어 쓴다. 그래서 SLOW_ZONE 에서
CROSSWALK_STOP 으로 바로 넘어가도 추정값이 끊기지 않는다.

측정값은 YOLO 횡단보도(MissionObject.CROSSWALK)의 가까운 끝까지 거리 x_m 이다
(11/02 사용자 결정: 카메라 정지선 인식 대신 YOLO 횡단보도 거리). 여기서 "정지선" 은 그 끝을 말한다.
횡단보도가 가까워져 위치를 못 구하면 마지막 측정값에서 주행 거리를 빼서 추정한다.
주행 거리는 odom 위치 변화로 구하고, odom 이 없으면 명령 속도 x 시간으로 대신한다.

규정 판정 기준점은 차량의 앞바퀴이므로, 정지 판단은 "앞바퀴 ~ 정지선" 거리(gap)로 한다.
    gap = (base_link ~ 정지선 거리) - front_wheel_x
"""

import math

CONTINUE_SEC = 0.3      # 직전 State 가 이 시간 안에 갱신했으면 추정을 이어받는다
JUMP_GUARD_NEAR = 0.5   # 추정 거리가 이보다 가까울 때는
JUMP_GUARD_UP = 0.10    # 측정값이 이만큼 이상 갑자기 멀어지면 무시 (횡단보도 줄무늬 오인 등)


class StopLineTracker(object):

    def __init__(self, node):
        self.node = node
        self.reset()

    def reset(self):
        self.est = None             # base_link ~ 정지선 [m]
        self.last_stamp = None      # 마지막으로 반영한 측정의 수신 시각
        self.last_pose = None
        self.last_time = None
        self.last_speed = 0.0

    def begin(self, now):
        """State 진입 시 호출. 바로 앞 State 가 쓰던 추정이 있으면 이어받고, 아니면 새로 시작."""
        if self.last_time is not None and (now - self.last_time) <= CONTINUE_SEC:
            return
        self.reset()
        self.last_speed = self.node.last_v    # 진입 직전 속도 (첫 측정 보정용)

    def update(self, now, commanded_speed):
        """매 주기 1회. commanded_speed = 직전 주기에 보낸 선속도."""
        node = self.node
        meas = node.stopline_measurement()
        pose = node.odom_pose()

        fresh = meas is not None and meas[1] != self.last_stamp
        if fresh:
            dist, stamp = meas
            self.last_stamp = stamp
            # 새 측정: 수신 이후 지금까지 이동한 만큼 빼서 반영
            dist -= self.last_speed * max(0.0, now - stamp)
            jumped = (self.est is not None and self.est < JUMP_GUARD_NEAR
                      and dist > self.est + JUMP_GUARD_UP)
            if jumped:
                node.dbg('stopline/rejected', dist)
                fresh = False
            else:
                self.est = dist

        if not fresh and self.est is not None:
            if pose is not None and self.last_pose is not None:
                self.est -= math.hypot(pose[0] - self.last_pose[0],
                                       pose[1] - self.last_pose[1])
            elif self.last_time is not None:
                self.est -= self.last_speed * (now - self.last_time)

        self.last_pose = pose
        self.last_time = now
        self.last_speed = commanded_speed
        node.dbg('stopline/est', self.est)
        return self.est

    def gap(self):
        """앞바퀴 ~ 정지선 [m]. 아직 정지선을 한 번도 못 봤으면 None."""
        if self.est is None:
            return None
        return self.est - self.node.front_wheel_x


def approach_speed(cfg, gap):
    """정지선 접근 속도. slowdown_distance > 0 이면 가까워질수록 감속."""
    base = cfg['approach_speed']
    if gap is None or cfg['slowdown_distance'] <= 0.0:
        return base
    ratio = min(1.0, max(0.0, gap) / cfg['slowdown_distance'])
    return max(cfg['min_speed'], base * ratio)
