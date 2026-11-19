"""차선 추종 주행 로직. lane_following 과 미션 State 들이 공유한다.

조향 방식 (lane.omega_pd):
  true (11/12 기본) - 차선 팀원 제안, 예전 limo_control 식. 회전속도 = pd_kp * lateral + pd_kd * (lateral 변화율).
    같은 곡률의 조향각으로 바꿔 쓰고, 조향각 변화는 초당 steer_rate_max 까지.
    Pure Pursuit 는 lateral 이 정확한 m 단위일 때만 맞게 꺾는데, 실차 카메라 값이 아직 시뮬 값이라 실차 곡선을 못 돎.
    이 방식은 실차에서 게인을 맞추면 그 오차까지 흡수 (팀원 실차 곡선 2곳 통과, Gazebo 첫 코너 4/4).
  false - Pure Pursuit (11/04, 아래 lane_steer 설명).
  (11/15, 차선 팀원 제안) lane.pd_speed_ref > 0 이면 그 속도보다 느릴 때 회전속도에 (속도 / pd_speed_ref) 를 곱해
    조향 세기(옆오차 1 m 당 조향각)를 pd_speed_ref 때로 묶는다. 회전속도를 속도로 나눠 조향각을 구하는 구조라
    느릴수록 세지기 때문 (0.12 m/s 에서 옆오차 5 cm 면 조향 한계, 차선 팀원 real_1114_e 직선 13% · 코너 60% 한계).
    pd_speed_ref 보다 빠르면 예전과 같다. 주차(PARKING 칸 찾기)는 이 제한을 안 씀 (사용자 결정, LaneDriver(speed_ref=False)).
"""

import math

from ar_mission_supervisor.controllers import clamp, steering_to_omega


class LaneDriver(object):

    def __init__(self, node, speed_ref=True):
        self.node = node
        self.speed_ref = speed_ref     # (11/15) False 면 lane.pd_speed_ref 저속 제한 안 씀 (주차)
        self.reset()

    def reset(self):
        self.prev_lateral = 0.0
        self.prev_stamp = None
        self.d_term = 0.0
        self.lost_since = None
        self.last_cmd = (0.0, 0.0)
        self.last_steer = 0.0
        self.lost_kappa = None     # (11/19) 차선을 놓친 순간 IMU(odom 방향)로 잰 곡률 [1/m] (False = 못 잼)
        self.last_head = None      # (11/19) 마지막 차선 값 (시각, 그때 차 방향, 차선 방향)
        self.lost_target = None    # (11/19) 놓친 뒤 돌아 갈 방향 (yaw) 과 이유
        self.lost_logged = False
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
                self.lost_kappa = self._measure_kappa(now)
                self.lost_target = self._lost_target(now)
                self.lost_logged = False
            self._failsafe(now - self.lost_since)

    def _lost_target(self, now):
        """(11/19 사용자·차선 팀원) 놓친 뒤 돌아 갈 방향 (yaw, 이유) 또는 None.
        (a) 놓치기 직전 head_max_age 초 안 카메라 방향이 |head_min_deg| 넘으면 그때 차 방향 + 차선 방향 (최대 90도)
        (b) 그 값이 없고 갈림길 뒤 ~ 라바콘 전이면 지금 방향에서 오른쪽 course_turn_deg (코스상 다음 코너가 오른쪽)
        둘 다 아니면 None (imu_keep -> creep -> stop)."""
        node = self.node
        cfg = node.failsafe
        pose = node.odom_pose()
        if pose is None:
            return None
        lh = self.last_head
        if cfg['head_follow'] and lh is not None and now - lh[0] <= cfg['head_max_age']:
            if abs(lh[2]) >= math.radians(cfg['head_min_deg']):
                hd = clamp(lh[2], -math.pi / 2, math.pi / 2)
                return (lh[1] + hd, '놓치기 직전 차선 방향 %+.0f도' % math.degrees(lh[2]))
            return None        # 직전 방향이 거의 직진 -> 그대로 (imu_keep)
        if node.after_ab_active() and node.after_ab['course_right']:
            ab = node.after_ab
            return (pose[2] - math.radians(ab['course_turn_deg']),
                    '방향 값 없음 + 갈림길 뒤라 오른쪽 %.0f도' % ab['course_turn_deg'])
        return None

    def _measure_kappa(self, now):
        """(11/19 사용자·차선 팀원) 차선을 놓친 순간 직전 imu_rate_window 초 동안 실제로 돈 곡률 = 회전 속도 / 속도.
        조향 명령(last_steer)은 흔들리는 중이면 한쪽 끝일 수 있어서, 실제 돈 양을 씀. 못 재면 False."""
        node = self.node
        cfg = node.failsafe
        if not cfg['imu_keep']:
            return False
        rate = node.turn_rate(now, window=cfg['imu_rate_window'])
        v = abs(getattr(node, 'last_v', 0.0))
        if rate is None or v < 0.05:
            return False
        return rate / v

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
        omega = g['pd_kp'] * lateral + g['pd_kd'] * self.pd_rate
        k_head = float(g.get('pd_head_k', 0.0))
        if k_head > 0.0:
            # (11/19 차선 팀원 제안, 시험 중) |차선 방향| 이 pd_head_dead_deg 넘으면 넘은 만큼 더 돎 (+-pd_head_max).
            #   1118 comp_lap14 145초: lat 0 인데 방향 -28도 -> 조향 -0.04 로 직진하다 C3 에서 놓침
            hd = lane[1]
            dead = math.radians(float(g.get('pd_head_dead_deg', 10.0)))
            if abs(hd) > dead:
                extra = k_head * (abs(hd) - dead) * (1.0 if hd > 0 else -1.0)
                lim = float(g.get('pd_head_max', 0.5))
                omega += clamp(extra, -lim, lim)
        return clamp(omega, -g['pd_max_omega'], g['pd_max_omega'])

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
            ref = float(node.lane_gains.get('pd_speed_ref', 0.0))
            if self.speed_ref and ref > 0.0 and v < ref:
                omega *= v / ref            # (11/15) 느릴 때 조향 세기를 pd_speed_ref 때로 묶음
            fast = float(node.lane_gains.get('pd_fast_ref', 0.0))
            if fast > 0.0 and v > fast:
                # (11/19) 빠를수록 세기를 (fast/v)^pd_fast_exp 로 줄임. 실차 지연(카메라 ~0.2 s + 명령->회전 0.25 s)으로
                #   0.4 m/s 에서 맞춘 세기가 1.0 m/s 에선 좌우로 커지며 차선 밖으로 나감 (r1118/pdsim 시뮬)
                omega *= (fast / v) ** float(node.lane_gains.get('pd_fast_exp', 0.0))
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
        pose = node.odom_pose()
        if pose is not None:
            self.last_head = (now, pose[2], lane.heading)
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

        # (00) (11/19 사용자·차선 팀원) 돌아 갈 방향이 있으면 (놓치기 직전 차선 방향, 또는 갈림길 뒤 오른쪽 90도)
        #   imu_speed 로 그 방향까지 돌고 그 방향 유지 -> stop_after_sec 에 정지 (그 전에 선을 다시 잡으면 차선 따라가기)
        tgt = self.lost_target
        pose = node.odom_pose()
        if tgt is not None and pose is not None and lost_sec < cfg['stop_after_sec']:
            if not self.lost_logged:
                self.lost_logged = True
                node.get_logger().info('[LANE] 차선 놓침 - %s 로 돌아 감 (목표 %.0f도, 지금 %.0f도)'
                                       % (tgt[1], math.degrees(tgt[0]), math.degrees(pose[2])))
            node.dbg('lane/mode', 'head')
            err = math.atan2(math.sin(tgt[0] - pose[2]), math.cos(tgt[0] - pose[2]))
            steer = clamp(cfg['head_k_yaw'] * err, -node.max_steer, node.max_steer)
            v = min(max(abs(self.last_cmd[0]), cfg['creep_speed']), cfg['imu_speed'])
            node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))
            return

        # (0) (11/19 사용자·차선 팀원) 놓친 순간 실제로 돌던 곡률(IMU) 그대로, imu_speed 로 줄여 imu_keep_sec 동안
        #   1118 comp_lap14: 코너 앞에서 선을 놓치고 hold -> creep -> stop 으로 11초 섬
        kappa = self.lost_kappa
        if kappa not in (None, False) and lost_sec < cfg['imu_keep_sec']:
            node.dbg('lane/mode', 'imu')
            v = min(max(abs(self.last_cmd[0]), cfg['creep_speed']), cfg['imu_speed'])
            steer = clamp(math.atan(node.wheel_base * kappa), -node.max_steer, node.max_steer)
            node.dbg('lane/lost_kappa', kappa)
            node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))
            return

        # (1) 단기 유실 - 직전 명령 유지 (imu_keep 을 못 쓸 때만)
        if kappa in (None, False) and lost_sec < cfg['hold_sec']:
            node.dbg('lane/mode', 'hold')
            v, omega = self.last_cmd
            node.publish_cmd(v, omega)
            return

        # (2) 저속 - 직전 조향각 유지 (같은 곡률). creep_keep_steer false 면 직진. (11/19) IMU 곡률을 쟀으면 그 곡률
        if lost_sec < cfg['stop_after_sec']:
            node.dbg('lane/mode', 'creep')
            v = cfg['creep_speed']
            if not cfg['creep_keep_steer']:
                steer = 0.0
            elif kappa not in (None, False):
                steer = clamp(math.atan(node.wheel_base * kappa), -node.max_steer, node.max_steer)
            else:
                steer = self.last_steer
            node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))
            return

        # (3) 장시간 유실 - 정차 후 재검출 대기
        node.dbg('lane/mode', 'stop')
        node.publish_stop()