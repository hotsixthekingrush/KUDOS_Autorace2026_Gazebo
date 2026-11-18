"""미션 6 + 3 - 서행 유도선(지그재그) 통과 + 횡단보도 정지 (11/18 사용자: 두 State 를 하나로 합침).

진입: YOLO 가 노면의 지그재그 서행 유도선을 인식하면.
동작 (merge_crosswalk true, 기본):
  1) 유도선 시작점에 도달할 때까지는 speed(0.2 m/s)로 일반 차선 추종.
     (유도선을 코너 끝에서 미리 볼 수 있으므로 차가 직선에 들어선 뒤 방향을 잡는다)
     시작점 = 진입 순간 YOLO 가 준 유도선 거리 + lock_offset 만큼 주행한 지점 (odom).
     유도선 거리나 odom 이 없으면 진입 후 lock_delay_sec 뒤.
  2) 그 순간의 odom 헤딩을 목표로 고정하고,
       조향 = k_yaw x 헤딩 오차 + lane_blend x confidence x 차선 추종 조향
     으로 직진 헤딩을 유지한다. 지그재그 차선을 그대로 따라가 좌우로 흔들리는 것을 막는다.
     차선이 안 보이면 헤딩 유지만 한다.
  3) 바닥 횡단보도(YOLO CROSSWALK) 측정을 floor_min_meas 번 이상 받으면 정지선 거리를 이어서 추정하고
     (화면 밖으로 빠지면 odom), 앞바퀴 ~ 정지선이 crosswalk.stop_gap 이하가 되면 이 State 안에서 정지.
     crosswalk.wait_sec 기다린 뒤 LANE_FOLLOWING. 보행자 표지판은 안 본다 (다른 State 로 안 넘어감).
  - 횡단보도를 한 번도 못 잡으면 max_duration 뒤 복귀. 정지선을 잡은 뒤에는 멈출 때까지 시간 제한 없음.
  - 정지선이 차 기준점 뒤로 exit_past_stopline 만큼 넘어가면(멈추지 못하고 지남) 바로 복귀.
  - 지그재그를 놓쳤을 때는 LANE_FOLLOWING 에서 보행자 표지판 -> CROSSWALK_STOP 이 대신 (그대로 남김).
예전 방식 (merge_crosswalk false, 11/15~11/17): 보행자 표지판을 보면 정지선이 handoff_stopline 안일 때
  CROSSWALK_STOP 으로 바로 넘어감 (표지판은 한 번 보면 기억, 정지선이 안 보이면 표지판이 handoff_sign 안일 때).
규정: 직진 헤딩을 유지하며 서행 통과. 횡단보도는 정지선 ±5cm 완전정지, 3초 이상 대기, 15초 이내 출발.
"""

import math

from ar_mission_supervisor.base_state import SafeState
from ar_mission_supervisor.controllers import clamp, steering_to_omega
from ar_mission_supervisor.geometry import normalize_angle
from ar_mission_supervisor.lane_driver import LaneDriver


def heading_hold_steer(node, driver, target_yaw, yaw, now):
    """고정 헤딩 유지 조향 (SLOW_ZONE, CROSSWALK_STOP 공용, 11/16).
    조향 = k_yaw x 헤딩 오차 + lane_blend x confidence x 차선 추종 조향. (조향, 헤딩 오차, 차선 몫) 을 돌려준다."""
    cfg = node.slow_zone
    yaw_err = normalize_angle(target_yaw - yaw)
    steer = cfg['k_yaw'] * yaw_err
    lane = node.lane_error(now)
    lane_part = 0.0
    if lane is not None and cfg['lane_blend'] > 0.0:
        confidence = clamp(lane[2], 0.0, 1.0)
        lane_part = cfg['lane_blend'] * confidence * driver.lane_steer(lane, now)
    steer = clamp(steer + lane_part, -node.max_steer, node.max_steer)
    return steer, yaw_err, lane_part


class SlowZone(SafeState):

    OUTCOMES = ['done', 'to_crosswalk']
    HANDOFF = ('to_crosswalk',)          # 바로 넘어갈 수 있는 미션

    def __init__(self, node):
        SafeState.__init__(self, node, 'SLOW_ZONE', self.OUTCOMES)
        self.driver = LaneDriver(node)
        self.entered_at = None
        self.target_yaw = None
        self.start_pose = None
        self.lock_after = None
        self.ped_seen = None        # (11/15) 보행자 표지판을 본 적 있으면 그 outcome (기억, 예전 방식)
        self.stopped_at = None      # (11/18) 합친 방식: 횡단보도에서 멈춘 시각

    def on_enter(self):
        node = self.node
        self.driver.reset()
        self.entered_at = node.clock_sec()
        self.target_yaw = None
        self.ped_seen = None
        self.stopped_at = None
        # 보행자 정지선 추정을 여기서부터 이어간다 (예전 방식에선 CROSSWALK_STOP 이 이어받음)
        node.stopline.begin(self.entered_at)
        # 헤딩 고정 시점: 유도선 시작점까지 남은 거리
        self.start_pose = node.odom_pose()
        line = node.slow_line_distance()
        self.lock_after = None
        if line is not None and self.start_pose is not None:
            self.lock_after = max(0.0, line) + node.slow_zone['lock_offset']
            node.get_logger().info('[SLOW_ZONE] 진입 - %.2fm 더 가서 헤딩 고정' % self.lock_after)
        else:
            node.get_logger().info('[SLOW_ZONE] 진입 - 유도선 거리 모름, %.1f초 뒤 헤딩 고정'
                                   % node.slow_zone['lock_delay_sec'])

    def run(self):
        node = self.node
        cfg = node.slow_zone

        while node.ok():
            if self.reset_requested():
                return 'aborted'

            now = node.clock_sec()

            # (11/18 사용자) 합친 방식: 횡단보도 정지선에서 멈춘 뒤 wait_sec 기다리고 끝
            if self.stopped_at is not None:
                node.publish_stop()
                waited = now - self.stopped_at
                node.dbg('slow_zone/waited', waited)
                if waited >= node.crosswalk['wait_sec']:
                    node.get_logger().info('[SLOW_ZONE] 횡단보도 %.1f초 정지 완료 - 출발' % waited)
                    # 같은 횡단보도의 보행자 표지판으로 LANE_FOLLOWING 이 CROSSWALK_STOP 에 또 들어가지 않게
                    node.set_lockout('CROSSWALK_STOP')
                    return 'done'
                self.sleep()
                continue

            node.stopline.update(now, node.last_v)
            measured = False

            if cfg['merge_crosswalk']:
                # 보행자 표지판은 안 보고, 바닥 횡단보도(YOLO) 정지선만 보고 이 State 안에서 멈춘다
                tr = node.stopline
                measured = tr.est is not None and tr.n_meas >= max(1, int(cfg['floor_min_meas']))
                gap = tr.gap() if measured else None
                node.dbg('slow_zone/gap', gap)
                if gap is not None and gap <= node.crosswalk['stop_gap']:
                    node.get_logger().info('[SLOW_ZONE] 앞바퀴~횡단보도 정지선 %.3fm - 정지' % gap)
                    self.stopped_at = now
                    node.publish_stop()
                    self.sleep()
                    continue
            else:
                # 예전 방식 (merge_crosswalk false): 보행자 표지판을 보면 CROSSWALK_STOP 으로 넘김
                outcome = node.detect_mission(allowed=self.HANDOFF)
                if outcome is not None and self.ped_seen is None:
                    self.ped_seen = outcome
                    node.get_logger().info('[SLOW_ZONE] 보행자 표지판 봄 - 정지선 %.2fm 안에 오면 CROSSWALK_STOP'
                                           % cfg['handoff_stopline'])
                # (11/15) 표지판이 화면 밖으로 빠져도 기억한 채로 정지선 거리만 보고 넘어간다
                if self.ped_seen is not None and self._near_crosswalk(cfg, sign_visible=outcome is not None):
                    node.get_logger().info('[SLOW_ZONE] 보행자 표지판 인식 - CROSSWALK_STOP 으로 바로 전환')
                    if self.target_yaw is not None:
                        node.heading_handoff = (self.target_yaw, now)   # (11/16) CROSSWALK_STOP 이 이 헤딩으로 이어감
                    return self.ped_seen

            # (11/15) 표지판을 못 보고 횡단보도를 지났으면 헤딩 고정 직진을 끝내고 일반 주행으로
            line = node.stopline.est
            if cfg['exit_past_stopline'] > 0.0 and line is not None and line <= -cfg['exit_past_stopline']:
                node.get_logger().warn('[SLOW_ZONE] 횡단보도에서 못 멈추고 정지선을 %.2fm 지남 - 복귀' % -line)
                return 'done'

            elapsed = now - self.entered_at
            if elapsed > cfg['max_duration'] and not measured:      # 정지선을 잡았으면 멈출 때까지 감
                node.get_logger().warn('[SLOW_ZONE] 횡단보도 미인식 %.0f초 - 복귀'
                                       % cfg['max_duration'])
                return 'done'

            yaw = node.odom_yaw()
            if self.target_yaw is None and yaw is not None and self._ready_to_lock(elapsed):
                self.target_yaw = yaw
                node.get_logger().info('[SLOW_ZONE] 헤딩 고정 %.1f도' % math.degrees(yaw))

            if self.target_yaw is None or yaw is None:
                # 고정 전이거나 odom 이 끊김 -> 일반 차선 추종으로 서행
                node.dbg('slow_zone/mode', 'lane')
                self.driver.step(cfg['speed'])
            else:
                self._hold_heading(now, yaw, cfg)

            self.sleep()
        return 'aborted'

    def _ready_to_lock(self, elapsed):
        node = self.node
        cfg = node.slow_zone
        pose = node.odom_pose()
        if self.lock_after is None or pose is None:
            return elapsed >= cfg['lock_delay_sec']
        travelled = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1])
        node.dbg('slow_zone/travelled', travelled)
        return travelled >= self.lock_after

    def _near_crosswalk(self, cfg, sign_visible=True):
        """보행자 표지판을 본 상태에서, 넘어갈 만큼 정지선에 가까운지."""
        node = self.node
        line = node.stopline.est            # 정지선 추정 (화면 밖으로 빠져도 이어서 추정)
        if line is not None:
            node.dbg('slow_zone/handoff_line', line)
            return line <= cfg['handoff_stopline']
        sign = node.pedestrian_sign_distance() if sign_visible else None
        if sign is not None and cfg['handoff_sign'] > 0.0:
            node.dbg('slow_zone/handoff_sign', sign)
            return sign <= cfg['handoff_sign']
        return True                      # 거리 정보가 전혀 없으면 (표지판도 화면 밖) 바로 넘어간다

    def _hold_heading(self, now, yaw, cfg):
        node = self.node
        steer, yaw_err, lane_part = heading_hold_steer(node, self.driver, self.target_yaw, yaw, now)

        v = cfg['speed']
        node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))

        node.dbg('slow_zone/mode', 'heading')
        node.dbg('slow_zone/yaw_err', yaw_err)
        node.dbg('slow_zone/lane_part', lane_part)
        node.dbg('slow_zone/steer', steer)
