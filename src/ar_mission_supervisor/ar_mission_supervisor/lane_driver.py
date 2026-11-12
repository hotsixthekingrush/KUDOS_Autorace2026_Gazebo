"""차선 추종 주행 로직. lane_following 과 미션 State 들이 공유한다.

조향 방식 (lane.omega_pd):
  true (11/12 기본) - 차선 팀원 제안, 예전 limo_control 식. 회전속도 = pd_kp * lateral + pd_kd * (lateral 변화율).
    같은 곡률의 조향각으로 바꿔 쓰고, 조향각 변화는 초당 steer_rate_max 까지.
    Pure Pursuit 는 lateral 이 정확한 m 단위일 때만 맞게 꺾는데, 실차 카메라 값이 아직 시뮬 값이라 실차 곡선을 못 돎.
    이 방식은 실차에서 게인을 맞추면 그 오차까지 흡수 (팀원 실차 곡선 2곳 통과, Gazebo 첫 코너 4/4).
  false - Pure Pursuit (11/04, 아래 lane_steer 설명).
"""

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
        self.pd_prev = None        # omega_pd: 직전 차선 값 (평활한 lateral, 찍은 시각, 원래 lateral)
        self.pd_rate = 0.0         # omega_pd: 평활한 lateral 변화율 [m/s]

    def _pd(self):
        return bool(self.node.lane_gains.get('omega_pd', False))

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
    def lane_omega(self, lane, offset=0.0):
        """omega_pd: 회전속도 = pd_kp * lateral + pd_kd * (lateral 변화율), +-pd_max_omega.
        변화율은 새 차선 값(찍은 시각이 바뀜)이 왔을 때만 다시 계산 (제어 20 Hz, 차선은 그보다 느림).
        이웃 값이 pd_d_jump 넘게 다르면 (한 선 <-> 두 선, 차로 바뀜) 그 변화율은 버림 (D 가 튀지 않게)."""
        g = self.node.lane_gains
        raw = lane[0] + offset
        stamp = lane[4]
        if self.pd_prev is None or stamp != self.pd_prev[1]:
            if self.pd_prev is None or abs(raw - self.pd_prev[2]) > g['pd_d_jump']:
                # 처음이거나 튐 (한 선 <-> 두 선, 차로 바뀜): 평활을 새 값에서 다시 시작, 변화율 버림
                filt = raw
                self.pd_rate = 0.0
            else:
                # (11/12) 옆오차도 평활 (Gazebo pd_lane_run1: 한 선 구간에서 0.1초마다 1 cm 씩 흔들려 핸들이 좌우로 뒤집힘).
                #   시간 상수 pd_lat_tau: 차선 값이 드물게 오면(느린 카메라) 그만큼 덜 평활 -> 늦어지지 않게
                dt = stamp - self.pd_prev[1]
                tau = g['pd_lat_tau']
                k = 1.0 if tau <= 0.0 or dt <= 0.0 else 1.0 - math.exp(-dt / tau)
                filt = self.pd_prev[0] + k * (raw - self.pd_prev[0])
                if 0.02 < dt < 1.0:
                    a = g['pd_d_smooth']
                    self.pd_rate = (1.0 - a) * self.pd_rate + a * (filt - self.pd_prev[0]) / dt
            self.pd_prev = (filt, stamp, raw)
        lateral = self.pd_prev[0]
        return clamp(g['pd_kp'] * lateral + g['pd_kd'] * self.pd_rate, -g['pd_max_omega'], g['pd_max_omega'])

    def lane_steer(self, lane, now, offset=0.0, speed=None):
        """차선 오차 -> 조향각(rad). 발행은 하지 않는다 (다른 제어와 섞을 때 사용).

        omega_pd (11/12 기본): lane_omega 의 회전속도를 speed 에서 같은 곡률의 조향각으로 바꿈.
          speed 를 안 주면 마지막으로 보낸 속도 (없으면 min_speed).

        lane = node.lane_error() 반환값 (lateral, heading, confidence, x_ref, stamp).
        x_ref > 0 (11/04 기본): 차 앞 x_ref 지점의 차선 중앙(+ offset)을 Pure Pursuit 로 따라간다.
          곡률 = 2 y / (x^2 + y^2). 카메라가 실제로 보는 지점이라 코너에서도 값이 맞고, 앞을 보고 미리 꺾는다.
        x_ref = 0: 예전 PD. D 는 새 차선 값이 왔을 때만 (찍은 시각 차이로) 다시 계산한다.
          (제어는 20 Hz, 차선은 3~5 Hz 라 매 주기 미분하면 새 값이 올 때만 튐)
        """
        node = self.node
        if self._pd():
            omega = self.lane_omega(lane, offset)
            v = speed if speed is not None else getattr(node, 'last_v', 0.0)
            v = max(abs(v), node.lane_gains['min_speed'], 1e-3)
            return clamp(math.atan(node.wheel_base * omega / v), -node.max_steer, node.max_steer)
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

    def _limit_steer(self, steer, now):
        """omega_pd: 조향각 변화를 초당 steer_rate_max 까지. 직전 값은 노드에 둬서 State 가 바뀌어도 이어짐."""
        node = self.node
        prev = getattr(node, 'lane_steer_out', None)
        if prev is not None and 0.0 < now - prev[1] < 0.5:
            d = node.lane_gains['steer_rate_max'] * (now - prev[1])
            steer = clamp(steer, prev[0] - d, prev[0] + d)
        elif prev is not None and now == prev[1]:
            steer = prev[0]
        node.lane_steer_out = (steer, now)
        return steer

    def _drive(self, lane, now, target_speed, offset=0.0):
        node = self.node
        steer = self.lane_steer(lane, now, offset, speed=target_speed)
        if self._pd():
            steer = self._limit_steer(steer, now)
            node.dbg('lane/omega_pd', self.lane_omega(lane, offset))
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