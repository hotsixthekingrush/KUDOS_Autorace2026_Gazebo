"""차선 추종 주행 로직. lane_following 과 미션 State 들이 공유한다."""

import math

from ar_mission_supervisor.controllers import clamp, steering_to_omega


class LaneDriver(object):

    def __init__(self, node):
        self.node = node
        self.reset()

    def reset(self):
        self.prev_lateral = 0.0
        self.prev_stamp = None
        self.d_term = 0.0
        self.lost_since = None
        self.last_cmd = (0.0, 0.0)
        self.last_steer = 0.0

    def step(self, target_speed, offset=0.0):
        """차선이 보이면 추종, 유실이면 페일세이프. 매 주기 1회 호출.

        offset: 차선 중앙 기준 목표 횡방향 위치(m, 왼쪽 +). 차선 변경에 사용.
        """
        node = self.node
        now = node.clock_sec()
        lane = node.lane_error(now)
        if lane is not None:
            self.lost_since = None
            self._drive(lane, now, target_speed, offset)
        else:
            if self.lost_since is None:
                self.lost_since = now
            self._failsafe(now - self.lost_since)

    # ------------------------------------------------------------------
    def lane_steer(self, lane, now, offset=0.0):
        """차선 오차 -> 조향각(rad). 발행은 하지 않는다 (다른 제어와 섞을 때 사용).

        lane = node.lane_error() 반환값 (lateral, heading, confidence, x_ref, stamp).
        x_ref > 0 (11/04 기본): 차 앞 x_ref 지점의 차선 중앙(+ offset)을 Pure Pursuit 로 따라간다.
          곡률 = 2 y / (x^2 + y^2). 카메라가 실제로 보는 지점이라 코너에서도 값이 맞고, 앞을 보고 미리 꺾는다.
        x_ref = 0: 예전 PD. D 는 새 차선 값이 왔을 때만 (찍은 시각 차이로) 다시 계산한다.
          (제어는 20 Hz, 차선은 3~5 Hz 라 매 주기 미분하면 새 값이 올 때만 튐)
        """
        node = self.node
        # lateral = 로봇 기준 차선 중앙의 위치 (왼쪽 +).
        # 목표 지점은 차선 중앙에서 offset 만큼 옆이므로 로봇 기준 위치는 lateral + offset.
        lateral = lane[0] + offset
        heading = lane[1]
        x_ref = lane[3] if len(lane) > 3 else 0.0
        stamp = lane[4] if len(lane) > 4 else now

        if self.prev_stamp is None or stamp != self.prev_stamp:
            if self.prev_stamp is not None and stamp > self.prev_stamp:
                self.d_term = (lateral - self.prev_lateral) / (stamp - self.prev_stamp)
            else:
                self.d_term = 0.0
            self.prev_lateral = lateral
            self.prev_stamp = stamp

        if x_ref > 1e-3:
            kappa = 2.0 * lateral / (x_ref * x_ref + lateral * lateral)
            return clamp(math.atan(node.wheel_base * kappa), -node.max_steer, node.max_steer)

        g = node.lane_gains
        steer = g['kp_lateral'] * lateral + g['kd_lateral'] * self.d_term + g['kp_heading'] * heading
        return clamp(steer, -node.max_steer, node.max_steer)

    def _drive(self, lane, now, target_speed, offset=0.0):
        node = self.node
        steer = self.lane_steer(lane, now, offset)
        omega = steering_to_omega(target_speed, steer, node.wheel_base)
        lateral, heading = lane[0] + offset, lane[1]
        self.last_cmd = (target_speed, omega)
        self.last_steer = steer
        node.publish_cmd(target_speed, omega)

        node.dbg('lane/mode', 'track')
        node.dbg('lane/lateral', lateral)
        node.dbg('lane/heading', heading)
        node.dbg('lane/steer', steer)

    def _failsafe(self, lost_sec):
        """유실 지속시간에 따른 단계적 대응: 직전 명령 유지 -> 저속(직전 조향각 유지) -> 정지.
        (11/10) 저속 구간은 직진 대신 직전 조향각을 유지한다 (creep_keep_steer). 직선에선 직전 조향이 거의 0이라
        예전과 같고, 곡선에선 계속 돈다. lane_yolo_run3·차선 팀원 시험: 코너에서 차선을 놓친 뒤 직진해서 바깥 선을 넘음.
        (11/05) 예전 규정(터널)용 '양옆 벽 따라가기' 단계는 뺌. 2026 규정 트랙엔 벽이 없고
        (규정집 VII 시설물: 신호등·광고판·표지판·LIMO·방지턱·라바콘·노면 표시뿐),
        lane_yolo_run3 에서 벽이 없는데도 라이다가 양옆 0.8 m 안에 뭔가를 봐서 켜졌음."""
        node = self.node
        cfg = node.failsafe
        node.dbg('lane/lost_sec', lost_sec)

        # (1) 단기 유실 - 직전 명령 유지
        if lost_sec < cfg['hold_sec']:
            node.dbg('lane/mode', 'hold')
            v, omega = self.last_cmd
            node.publish_cmd(v, omega)
            return

        # (2) 저속 - 직전 조향각 유지 (같은 곡률). creep_keep_steer false 면 직진
        if lost_sec < cfg['stop_after_sec']:
            node.dbg('lane/mode', 'creep')
            v = cfg['creep_speed']
            steer = self.last_steer if cfg['creep_keep_steer'] else 0.0
            node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))
            return

        # (3) 장시간 유실 - 정차 후 재검출 대기
        node.dbg('lane/mode', 'stop')
        node.publish_stop()