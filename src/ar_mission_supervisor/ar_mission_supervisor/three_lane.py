"""3차선 구간 공용 상태·주행 - AVOID_STATIC 과 LEFT_A_RIGHT_B 가 같이 쓴다.

차선 번호 (진행방향 기준): 0 = 왼쪽, 1 = 가운데, 2 = 오른쪽.
  2026 트랙에서는 3차선 끝의 A/B 섬이 가운데 차선을 막고 있어서
  A(좌) 경로는 0번, B(우) 경로는 2번 차선으로 이어진다.
좌표는 AVOID_STATIC 진입 순간의 차선 방향 기준(LaneFrame). 가운데 차선 중앙이 d = 0,
왼쪽 차선 중앙이 d = +lane_width, 오른쪽이 d = -lane_width.
(11/12) 카메라 차선을 못 믿어서 차 방향을 기준으로 쓸 때는, 차 방향이 트랙 축(odom 기준 axis_ref_deg + 90도 단위)에서
  axis_snap_max_deg 안이면 그 축으로 맞춘다 (axis_snap). Gazebo pd_lane_run1~3: 두 번째 코너를 16~20도 덜 돈 채
  (카메라는 3차선 입구에서 옆 0, 방향 +20도로 틀림) 진입 -> 차 방향을 기준으로 잡아 바깥 노란 선 위까지 감.
  트랙이 직각으로만 되어 있고, odom yaw 0 이 출발 직선 방향이라는 가정 (출발선에 똑바로 놓고 odom 을 켤 것).

(11/12) 차 방향을 기준으로 쓸 때는 옆 위치(d = 0)도 차 위치라서, 차가 차선 가운데에서 비켜 있으면 세 차선이 다 같이
  밀린다 (pd_lane_run4: 11 cm). 가까이(lat_snap_range) 확실히 본 LIMO 가 lat_snap_min_items 대 이상 모이면,
  LIMO 가 차선 가운데(d = 0, +-lane_width)에 서 있다고 보고 옆 위치를 한 번 맞춘다 (lat_snap).

차선 번호는 변수로 들고 다니되(시작 = 가운데), 차선 변경을 '끝냈을 때만' 바꾼다.
odom 으로 계산한 실제 위치(d)와 계속 어긋나면 odom 쪽을 따른다.
(11/12, lane_from_odom) 차선 바꾸는 중에도 차선 번호는 odom 위치로 고친다 (pd_lane_run4: 가운데~오른쪽인데 계속 '왼쪽').
(11/12, lane_commit, 기본 끔) 켜면 목표 차선에 들어가기 전에는 목표를 안 바꾼다 (그 목표가 곧 막히고 switch_hold 가
  지났을 때만 예외). pd_lane_run4 에서 목표가 0.15초 만에 뒤집혀서 넣어 봤지만, 0.55 배치는 계속 다시 골라야 빠져나가는
  경우가 있어 run.py 4개가 막힘 -> 끔. '바로 옆 차선만 목표' 도 시험했지만 0.55 배치는 오른쪽 -> 왼쪽을 가운데 LIMO 사이로
  대각선으로 가로질러야 해서 static_noise 40번 중 2번만 성공 -> 코드에 안 넣음.

step(now, goal) 한 번 = 한 제어 주기:
  - 라이다 장애물 지도 갱신 (LIMO 는 차선 방향으로 놓인 0.322 x 0.22 m 박스 + 여유 clearance 로 본다)
    (11/04) 위치는 본 것의 평균(map_average), 이미 확실한 LIMO 와 겹치는 새 검출은 버림(ghost_*).
  - 후보 차선 3곳(차선 가운데)마다, 그 차선으로 가는 주행을 실제 조향 한계(max_steer, wheel_base)로
    앞 plan_horizon 만큼 미리 굴려 보고(rollout) 장애물과 닿는지 검사한다.
    (11/04) 보통(lookahead)으로 굴려서 막히는 차선은 급하게 꺾는 것(lookahead_short)도 굴려 보고
    더 멀리 가는 쪽으로 따라간다. 차 모양은 몸통 사각형 + (Gazebo 만) 바퀴를 감싼 사각형.
  - (11/05) plan_depth 2: 차선마다 '그 차선으로 가다가 switch_step 마다 옆 차선으로 한 번 더 옮기면' 까지
    plan_horizon_long 만큼 굴려 보고 가장 멀리 가는 거리(reach)로 비교한다 (막다른 차선 피하기, static_run3).
    굴릴 때 3차선 바깥 선을 넘어도 막힌 것으로 본다 (edge_check).
  - 지금 목표 차선이 끝까지 안 닿으면 유지한다 (goal 차선이 끝까지 비면 goal 로).
    아니면 reach 가 switch_margin 이상 긴 차선으로 바꾸되, 바꾼 뒤 switch_hold 초는 유지한다
    (지금 목표가 commit_distance 안에서 막히면 바로 바꿈). goal 은 +0.3 m 가산.
    (11/12, lane_commit 켰을 때만) 목표 차선으로 가는 중(아직 목표 차선 밖)에는 목표를 고정, 목표가 commit_distance
    안에서 막히고 switch_hold 가 지났을 때만 다시 고른다.
  - 차선 사이(선 위)로 가는 경로는 쓰지 않는다. 고른 차선도 commit_distance 보다 짧게 닿으면
    감속하고, stop_distance 안에서 닿게 되면 선다.
  - (11/12) 지금 자리에서 이미 여유 박스(LIMO + clearance)와 겹치는 LIMO 는, 지금 안 겹칠 만큼만 여유를 줄여서 본다
    (overlap_relax, 진짜 크기보다 overlap_extra 까지). 그보다 더 가까워지는 길은 그대로 막힘.
    static_run6: 진짜 틈 2.6 cm 인데 여유 1.5 cm + 지도 오차 2 cm 로 '이미 닿음' -> 어느 길도 0 m, 후진도 못 해서
    제한시간까지 22초 정지 (제한시간 뒤 차선 주행으로 그냥 빠져나감).
  - (11/04 사용자 요청) 선 채로 backup_after 초가 지나면, 뒤가 비어 있을 때 backup_speed 로 backup_distance 만큼
    후진하고(차 방향을 차선 방향으로 펴면서) 다시 계산한다. 한 미션에 backup_max 번까지. LIMO 는 안 움직여서
    서서 기다리기만 하면 계속 막혀 있고, 규정상 진행 불가는 미션 처음부터 재출발이라서.
  - (11/16 사용자) 한 번 후진으론 안 풀릴 때 앞뒤로 움직여 빠져나갈 길을 찾는다 (maneuver, 하이브리드 A*): 앞/뒤 x 조향
    (왼쪽 끝/0/오른쪽 끝) 0.1 m 조각을 이어 붙여, LIMO 박스·바깥 선에 안 닿고 시작 자리보다 maneuver_max_back 넘게
    뒤로 안 가면서 maneuver_goal_ahead 앞의 차선 가운데(방향 maneuver_goal_heading_deg 안)까지 가는 가장 싼 길
    (뒤로 가기·앞뒤 바꾸기에 비용). 찾으면 조각대로 움직이고 (조향 바뀔 땐 서서 바퀴부터 돌림), 움직이며 지도로 계속
    확인해 닿을 것 같거나 예상 자리에서 어긋나면 다시 찾음. 못 찾으면 아래 후진으로.
    순서: 막히고 backup_after 뒤 아래 후진 고르기(search)를 먼저 해 보고, 고른 후진 뒤 앞 거리가 maneuver_if_reach_below
    넘게 나오면 그 후진(빠름). 안 나오거나, 후진했던 자리 maneuver_after_backup_within 안에서 또 막히면 앞뒤 움직임.
    길은 LIMO 박스를 maneuver_margin 키워서 찾고(안 나오면 0.01, 0), 움직일 땐 maneuver_check_slack 줄인 박스로 확인.
    움직이다 보통 계획으로 끝까지 갈 차선이 생기고 방향이 maneuver_handback_deg 안이면 차선 따라가기로 돌아감.
    static_1116_1 (실차): 두 줄 LIMO 가 1 m 사이로 열린 차선이 반대 -> 차선 가운데로만 가는 계획으론 못 지나가고
    같은 자리를 오감. 녹화 지도로 찾으면 앞뒤 바꿈 3~7번, 3.5~4.7 m 로 지나감 (차 방향을 60~80도까지 틀어 비스듬히 건넘).
    static_noise 실차 설정 40번 (후진 고르기만 -> 앞뒤 움직임 넣은 뒤): 간격 0.48 34 -> 39 (평균 36초), 0.46 7 -> 38 (43초),
    0.55/0.50/0.75 40 -> 40 (24/25/31초), 0.55 유령 LIMO 31 -> 32, Gazebo 28 -> 31.
    static_1116_1 배치 (LAYOUT="1.12:1,1.80:0,1.89:1,2.80:1,2.90:2" X0=9.09 Y0=0.03): 똑바로 진입 40/40 (22초),
    20도 틀어 진입 39/40 (49초), 39도(녹화 때) 39/40 (46초). 예전 코드는 10번씩 10 / 0 / 0.
    조심: 최소 회전 반경은 실측 조향 한계 0.33 rad 로 약 0.58 m (0.46 아님). 실차 odom 이 조각마다 어긋나면 다시 찾느라 더 걸림.
  - (11/16 사용자) 후진을 꼭 차선과 평행하게 펴지 않는다 (backup_mode 'search'). 막히면 backup_after 뒤,
    조향 6가지(차선 방향으로 펴기 + 고정 조향 -최대/-반/0/+반/+최대) x 후진 거리(backup_search_dists) 를 지도(LIMO 박스)로
    굴려 보고, LIMO 에 닿거나 3차선 바깥 선을 넘는(edge_check, 앞으로 굴릴 때와 같은 검사) 후진은 버린다.
    남은 후보마다 그 자리에서 앞으로 굴려 본 거리(plan reach)가 가장 긴 것을 고름 (같으면 짧은 후진·작은 조향).
    지금보다 backup_min_gain 넘게 나아지는 후보가 없으면 예전처럼 펴면서 backup_distance 후진.
    횟수는 backup_max (3 -> 8). 기다림 backup_after 는 1.0 s 그대로 (0.3 s 로 줄이면 static_noise 간격 0.48 에서 34 -> 21/40 로 나빠짐:
    라이다 지도가 자리 잡기 전에 후진함). 후진 거리 후보를 0.15 부터 하면 너무 조금 물러나 다시 막히는 일이 있어 0.20 부터.
    static_noise 실차 설정 40번 (예전 -> 지금): 간격 0.48 34 -> 34, 0.46 6 -> 7, 0.55 유령 LIMO 31 -> 31, 0.55/0.50/0.75 40 -> 40,
    Gazebo 29 -> 28 = 이 시험 배치들에선 거의 같음 (펴는 후진이 막히는 배치가 아님). 후진 뒤 고를 때 본 차선을 유지하게도
    해 봤지만 0.48 에서 28 로 나빠져서 뺌.
  - (11/16 사용자) 입구 사선 보호 (back_floor): 3차선 입구는 0.55 m 동안 45도 사선으로 넓어지는데 바깥 선 검사는 3차선 전체
    폭이라, 입구 쪽으로 후진하면 사선을 넘는다 (static_1116_1). 입구 선 위치를 모르므로 '첫 LIMO 가운데 + back_floor_offset 에서
    다 넓어진다' 로 보고, 그보다 뒤에서는 폭이 back_floor_slope (1.0 = 45도) 로 좁아져 가운데 차선 폭까지 줄어든다고 본다.
    뒤로 갈 때만 차 네 모서리를 검사 (후진 고르기·펴는 후진·앞뒤 움직임의 뒤로 가는 조각).
    '첫 LIMO 뒤로 후진 금지' 를 먼저 넣어 봤지만 static_1116_1 배치에서 10번 중 0번 통과 (첫 LIMO 옆에서 막히면 그 뒤로 조금
    물러나야만 빠져나옴) -> 사선 모양으로. 앞으로 가다 사선을 넘는 것은 못 막는다 (입구 선 위치가 있어야 함 - 차선 팀원 흰 가로선).
    static_noise MOUTH=0.62 (입구 선이 첫 LIMO 0.5 m 앞이라 침) 네 배치, 뒤로 가다 사선 넘음: 끔 20/20·15/20 (최대 25 cm)
    -> offset 0 39/40·15/40 (9 cm) -> offset 0.10 0/40·1/40 (1 cm). 통과 39/40 씩, 시간 8초쯤 더. 기본 배치는 그대로.
  - (11/16 사용자·차선 팀원) 진입 때 '가운데 차선' 은 가정일 뿐 (넓어지는 입구에서 왼쪽 선을 따라 왼쪽 차선으로 들어와 있을 수
    있음 -> 없는 차선을 있다고 보고 진짜 차선은 트랙 밖으로 봄). 차선 팀원 흰 점선 노드(/perception/white_status)가 켜져 있으면
    진입 뒤 white.pending_dist 동안 확인해 기준 좌표를 한 번만 고친다 (_ident_begin / _ident_step, white_lines.py).
    양쪽 점선을 여러 번 보면 바로 확정. 한쪽만 보이면 '한 차선 밀렸을 수도' 로 두고, 양쪽 가정에 다 있는 차선만 쓰면서
    (edge_shrink) 천천히 가며 라이다 LIMO 로 가린다. 못 가리면 처음 가정 그대로.
    static_noise WHITE=1: 실제로는 왼쪽/오른쪽 차선(Y0=-0.52/+0.52, LANEHEAD=0)인데 가운데로 가정 -> 흰 점선 없이 0/20 (전부 트랙 밖),
    있으면 20/20. 제대로 가운데로 들어온 경우 (한쪽 점선이 늘 가려진 WMISS 포함) 는 그대로: 0.55 40·40·40, 0.48 39·40·38, 0.46 38.
"""

import math

from ar_mission_supervisor.controllers import clamp, steering_to_omega
from ar_mission_supervisor.geometry import find_largest_gap, normalize_angle
from ar_mission_supervisor.lane_frame import LaneFrame, ObstacleMap, pursue


class StepResult(object):
    def __init__(self, pose, s_, d, psi, blocked=False):
        self.pose, self.s, self.d, self.psi = pose, s_, d, psi
        self.blocked = blocked


class ThreeLanePlanner(object):

    LANES = (0, 1, 2)
    NAMES = ('왼쪽', '가운데', '오른쪽')

    def __init__(self, node):
        self.node = node
        self.frame = LaneFrame()
        self.map = ObstacleMap()
        self.active = False
        self.released_at = None
        self.lane = 1
        self.target = 1
        self.changing = False
        self.mismatch_since = None
        self.logged_blocked = False
        self.max_obstacle_s = None
        self.blocked_since = None
        self.backing = None         # 후진 중이면 (시작 x, 시작 y, 조향 또는 None, 거리)
        self.maneuver = None        # (11/16) 앞뒤로 움직여 빠져나가는 중이면 그 계획
        self.maneuvers = 0
        self.logged_floor = False
        self.last_backup_s = None   # 마지막으로 후진을 시작한 자리 s (그 근처에서 또 막히면 후진 말고 앞뒤 움직임)
        self.backups = 0
        self.target_since = -1e9
        self.lat_snap_pending = False   # 옆 위치를 LIMO 로 맞춰야 하는지 (차 위치 기준일 때, 11/12)
        self.ident = None               # (11/16) 흰 점선으로 몇 번 차선인지 확인하는 중이면 {'t0', 's0', 'hyp'}
        self.edge_shrink = (0.0, 0.0)   # (오른쪽 바깥 선, 왼쪽 바깥 선)을 안쪽으로 당긴 양 [m] - 확인 전까지 양쪽 가정 모두에서 안전한 폭만
        self.relax_logged = False       # 겹친 LIMO 여유 줄임 로그 (11/12)
        self.lat_offset = 0.0

    @property
    def cfg(self):
        return self.node.avoid_static

    # ------------------------------------------------------------------
    # 수명
    # ------------------------------------------------------------------
    def begin(self, lane):
        pose = self.node.odom_pose()
        if pose is None:
            self.active = False
            return False
        lane_ref, why = self.node.lane_for_frame()
        self.frame.begin(pose, lane_ref)
        cfg = self.cfg
        self.lat_offset = 0.0
        self.relax_logged = False
        known = self._known_axis(pose)          # (11/16 A안) 3차선 직선 방향을 알면 방향은 늘 그 값
        if lane_ref is not None:
            self.lat_snap_pending = False
            self.node.get_logger().info('[3차선] 기준 좌표: %s%s' % (why, known or ''))
        else:
            self.lat_snap_pending = bool(cfg['lat_snap'])
            self.node.get_logger().info('[3차선] 기준 좌표: 차 방향 (%s)%s%s' % (
                why, known if known is not None else self._snap_axis(pose),
                ' - 옆 위치는 LIMO 보이면 맞춤' if self.lat_snap_pending else ''))
        ghost = None
        if cfg['ghost_min_hits'] > 0:
            ghost = (cfg['limo_length'] + cfg['ghost_margin'], cfg['limo_width'] + cfg['ghost_margin'],
                     cfg['ghost_min_hits'])
        self.map = ObstacleMap(merge=cfg['merge_distance'], average=cfg['map_average'], ghost=ghost)
        self.lane = self.target = int(lane)
        self.changing = False
        self.mismatch_since = None
        self.logged_blocked = False
        self.max_obstacle_s = None
        self.blocked_since = None
        self.backing = None
        self.backups = 0
        self.maneuver = None
        self.maneuvers = 0
        self.last_backup_s = None
        self.logged_floor = False
        self.target_since = -1e9
        self.active = True
        self.released_at = None
        self._ident_begin()
        return True

    def _known_axis(self, pose):
        """(11/16 A안) 3차선 직선 방향(three_lane_axis_deg)을 쓸 수 있으면 기준 좌표 방향을 그 값으로. 로그 글 또는 None."""
        cfg = self.cfg
        if not cfg['three_lane_axis_use']:
            return None
        known = self.node.known_lane_axis(pose[2])
        if known is None:
            self.node.get_logger().warn('[3차선] 차 방향 %.0f도가 3차선 방향 %.0f도에서 %.0f도 넘게 벗어남 - 3차선 방향 안 씀'
                                        % (math.degrees(pose[2]), cfg['three_lane_axis_deg'], cfg['three_lane_axis_max_deg']))
            return None
        axis, err = known
        ox, oy, oyaw = self.frame.origin
        self.frame.origin = (ox, oy, axis)
        return ' -> 방향은 3차선 방향 %.0f도로 (차 방향 %.0f도, %+.0f도 차이)' % (
            math.degrees(axis), math.degrees(pose[2]), math.degrees(err))

    def _snap_axis(self, pose):
        """(11/12) 차 방향 기준일 때 가까운 트랙 축으로 맞춤. 로그에 붙일 설명을 돌려준다."""
        cfg = self.cfg
        if not cfg['axis_snap']:
            return ''
        x, y, yaw = pose
        ref = math.radians(cfg['axis_ref_deg'])
        rel = math.atan2(math.sin(yaw - ref), math.cos(yaw - ref))
        axis = ref + round(rel / (math.pi / 2.0)) * (math.pi / 2.0)
        err = math.atan2(math.sin(yaw - axis), math.cos(yaw - axis))
        if abs(err) > math.radians(cfg['axis_snap_max_deg']):
            return ' - 트랙 축에서 %.0f도 벗어나 그대로 씀' % math.degrees(abs(err))
        ox, oy, _ = self.frame.origin
        self.frame.origin = (ox, oy, math.atan2(math.sin(axis), math.cos(axis)))
        return ' -> 트랙 축 %.0f도로 맞춤 (차 방향 %.0f도, %+.0f도 차이)' % (
            math.degrees(math.atan2(math.sin(axis), math.cos(axis))), math.degrees(yaw), math.degrees(err))

    def _snap_lateral(self, s_car, who):
        """(11/12) 차 위치 기준일 때, 가까이 확실히 본 LIMO 가 차선 가운데(d = 0, +-lane_width)에 오도록 옆으로 한 번 옮긴다.
        LIMO 마다 '가까운 차선 가운데에서 얼마나 비켜 있나'를 구해, lat_snap_spread 안에 가장 많이 모인 값들
        (lat_snap_min_items 대 이상이고 절반 넘게)의 평균만큼 옮김. 유령·잘못 찍힌 LIMO 는 묶음에서 빠짐.
        lat_snap_max 보다 크면 어느 차선인지 애매해서 안 옮김. 맞췄거나 안 맞추기로 했으면 True."""
        cfg = self.cfg
        w = self.width()
        items = [it for it in self.map.confirmed(cfg['confirm_hits'])
                 if s_car - 0.3 <= it['s'] <= s_car + cfg['lat_snap_range'] and abs(it['d']) <= 1.5 * w]
        need = max(1, int(cfg['lat_snap_min_items']))
        if len(items) < need:
            return False
        res = sorted(it['d'] - w * round(it['d'] / w) for it in items)
        group = []
        for i in range(len(res)):
            g = [r for r in res[i:] if r - res[i] <= cfg['lat_snap_spread']]
            if len(g) > len(group):
                group = g
        if len(group) < need or 2 * len(group) <= len(res):
            return False                    # 아직 모자라거나 제각각 - 더 보고 판단
        off = sum(group) / len(group)
        log = self.node.get_logger()
        if abs(off) > cfg['lat_snap_max']:
            log.warn('[%s] LIMO %d대가 차선 가운데에서 %+.2fm 비켜 있음 - %.2fm 넘어 어느 차선인지 애매해서 옆 위치 그대로'
                     % (who, len(group), off, cfg['lat_snap_max']))
            return True
        nx, ny = self.frame.frame_to_odom(0.0, off)
        self.frame.origin = (nx, ny, self.frame.origin[2])
        for it in self.map.items:
            it['d'] -= off
        self.lat_offset = off
        log.info('[%s] 기준 좌표 옆 위치를 LIMO %d대(전체 %d대)로 맞춤: %+.2fm (LIMO 가 차선 가운데에 있다는 가정)'
                 % (who, len(group), len(res), off))
        return True

    # ---------------- (11/16 사용자·차선 팀원) 흰 점선으로 '실제로 몇 번 차선인지' ----------------
    def _ident_begin(self):
        """진입 때 '지금 가운데 차선' 은 가정일 뿐 (왼쪽 선을 따라 왼쪽 차선으로 들어와 있을 수 있음).
        흰 점선 노드가 살아 있으면 white.pending_dist / pending_sec 동안 확인한다 (white_lines.py 설명).
          - 양쪽 점선을 본 관측이 모이면 (그 k 로만 설명되는 관측 min_votes 개 이상, 전체의 ratio 이상) 바로 확정:
            기준 좌표를 k 차선 + 잔차만큼 한 번 옮기고 끝. 그 뒤로는 odom 으로 유지 (다시 안 옮김).
          - 한쪽만 계속 보이면 '한 차선 밀렸을 수도' 로만 둔다: 양쪽 가정 모두에서 있는 차선만 쓰고(바깥 선을 한 차선
            안으로), 천천히 가면서 라이다 LIMO 로 가린다 - 한쪽 가정에서만 차선 가운데에 놓이는 LIMO 가 있으면 그쪽으로 확정.
          - 끝내 못 가리면 처음 가정(가운데) 그대로."""
        self.ident = None
        self.edge_shrink = (0.0, 0.0)
        node = self.node
        wl = getattr(node, 'white_lines', None)
        cfg = getattr(node, 'white', None)
        if wl is None or cfg is None or not cfg['use']:
            return
        self.ident = {'t0': node.clock_sec(), 's0': 0.0, 'hyp': None, 'seen': False}

    def _ident_end(self):
        self.ident = None
        self.edge_shrink = (0.0, 0.0)
        half = 1.5 * self.width() + 0.15
        self.map.items = [it for it in self.map.items if abs(it['d']) <= half]

    def _shift_frame(self, off, now):
        """기준 좌표를 옆으로 off 만큼 옮김 (차·LIMO 의 d 가 off 만큼 줄어듦). 한 차선쯤 옮기면 차선 번호·계획도 새로."""
        nx, ny = self.frame.frame_to_odom(0.0, off)
        self.frame.origin = (nx, ny, self.frame.origin[2])
        for it in self.map.items:
            it['d'] -= off
        self.lat_offset += off
        if abs(off) > 0.5 * self.width():
            pose = self.node.odom_pose()
            if pose is not None:
                self.lane = self.target = self.lane_of(self.frame.car(pose)[1])
            self.changing = False
            self.mismatch_since = None
            self.backing = None
            self.maneuver = None
            self.blocked_since = None
            self.logged_blocked = False
            self.target_since = now

    def _ident_step(self, pose, s_, d, psi, now, who):
        """흰 점선 확인 한 주기. 기준 좌표를 옮겼으면 True."""
        node = self.node
        cfg = node.white
        idn = self.ident
        log = node.get_logger()
        w = self.width()
        if s_ - idn['s0'] > cfg['pending_dist'] or now - idn['t0'] > cfg['pending_sec']:
            if idn['hyp'] is not None:
                log.warn('[%s] 흰 점선: 실제로는 %s 차선일 수도 있었는데 끝내 확인 못 함 - 처음 가정 그대로 감 (지금 %s 차선)'
                         % (who, self._true_lane_name(d, idn['hyp']), self.name(self.lane_of(d))))
            elif idn['seen']:
                log.info('[%s] 흰 점선으로 차선을 확정할 만큼은 못 봄 - 처음 가정 그대로' % who)
            self._ident_end()
            return False
        v = node.white_lines.votes(now, pose, self.frame, w)
        if v is None:
            return False
        idn['seen'] = True
        n, supp, excl = v['n'], v['supp'], v['excl']
        node.dbg('static/white_n', float(n))
        cand = [k for k in (-1, 0, 1) if supp[k] >= cfg['min_votes'] and supp[k] >= cfg['ratio'] * n]
        both_n, left_n, right_n = v['kinds']
        seen_txt = '양쪽 %d번, 왼쪽만 %d번, 오른쪽만 %d번' % (both_n, left_n, right_n)
        if len(cand) == 1 and excl[cand[0]] >= cfg['min_votes']:
            return self._ident_confirm(cand[0], v['fine'][cand[0]], d, now, who, '흰 점선 ' + seen_txt)
        hyp = None
        if len(cand) == 2 and 0 in cand and n >= cfg['single_min_votes']:
            hyp = cand[0] if cand[1] == 0 else cand[1]
        if hyp is None:
            return False
        if idn['hyp'] != hyp:
            cut = (w, 0.0) if hyp > 0 else (0.0, w)          # hyp -1: 실제로는 한 차선 왼쪽 -> 기준 좌표의 왼쪽 차선은 없는 차선
            ext = 0.5 * math.hypot(self.cfg['car_length'], self.cfg['car_width'])
            lim = 1.5 * w
            inside = (d + ext <= lim - cut[1]) and (d - ext >= -lim + cut[0])
            idn['hyp'] = hyp
            self.edge_shrink = cut if inside else (0.0, 0.0)
            log.warn('[%s] 흰 점선이 한쪽에만 보임 (%s) - 실제로는 %s 차선일 수도 있음. 확인될 때까지 천천히%s'
                     % (who, seen_txt, self._true_lane_name(d, hyp),
                        (', 기준 좌표의 %s 차선으로는 안 감' % self.name(0 if hyp < 0 else 2)) if inside else ''))
        # 라이다 LIMO 로 가리기: 한쪽 가정에서만 차선 가운데(3차선 안)에 놓이는 LIMO
        tol = cfg['limo_tol']
        only0 = onlyk = 0
        for it in self.map.confirmed(self.cfg['confirm_hits']):
            if not (s_ - 0.3 <= it['s'] <= s_ + 3.5):
                continue
            ok = []
            for k in (0, hyp):
                dd = it['d'] - k * w
                j = math.floor(dd / w + 0.5)
                ok.append(abs(j) <= 1 and abs(dd - j * w) <= tol)
            if ok[0] and not ok[1]:
                only0 += 1
            elif ok[1] and not ok[0]:
                onlyk += 1
        if onlyk >= 1 and only0 == 0:
            return self._ident_confirm(hyp, None, d, now, who,
                                       '흰 점선 %s + 그 가정에서만 차선에 놓이는 LIMO %d대' % (seen_txt, onlyk))
        if only0 >= 1 and onlyk == 0:
            return self._ident_confirm(0, None, d, now, who,
                                       '흰 점선은 한쪽만(%s) 보였지만 LIMO %d대가 처음 가정에서만 차선에 놓임' % (seen_txt, only0))
        return False

    def _true_lane_name(self, d, k):
        """기준 좌표가 k 차선 어긋났다면 차가 실제로 있는 차선 이름."""
        return self.name(self.lane_of(d - k * self.width()))

    def _ident_confirm(self, k, fine, d, now, who, why):
        cfg = self.node.white
        log = self.node.get_logger()
        w = self.width()
        if fine is None or not cfg['use_offset'] or abs(fine) > cfg['max_fine']:
            fine = 0.0
        off = k * w + fine
        was = self.name(self.lane_of(d))
        moved = abs(off) > 0.02
        if moved:
            self._shift_frame(off, now)
        self._ident_end()                   # 옮긴 뒤에 정리 (3차선 밖 LIMO 는 옮긴 좌표로 걸러야 함)
        if k == 0:
            log.info('[%s] 차선 확인: %s 차선 맞음 (%s)%s' % (who, was, why,
                                                         ' - 옆 위치 %+.2fm 맞춤' % off if moved else ''))
        else:
            log.warn('[%s] 차선 확인: 실제로는 %s 차선 (가정은 %s) - 기준 좌표를 옆으로 %+.2fm 옮김 (%s)'
                     % (who, self.name(self.lane_of(d - off)), was, off, why))
        return moved

    def release(self, now):
        """AVOID_STATIC 이 끝남. 잠깐 동안은 LEFT_A_RIGHT_B 가 이어받을 수 있다."""
        self.released_at = now

    def resume(self):
        self.released_at = None

    def fresh(self, now):
        if not self.active:
            return False
        if self.released_at is None:
            return True
        return (now - self.released_at) <= self.cfg['context_timeout']

    def finish(self):
        self.active = False

    # ------------------------------------------------------------------
    # 차선 기하
    # ------------------------------------------------------------------
    def width(self):
        return self.cfg['lane_width']

    def center(self, lane):
        return (1 - lane) * self.width()

    def lane_of(self, d):
        return int(max(0, min(2, round(1.0 - d / self.width()))))

    def name(self, lane):
        return self.NAMES[lane]

    @staticmethod
    def dist_str(x):
        return 'inf' if math.isinf(x) else '%.2fm' % x

    # ------------------------------------------------------------------
    # 장애물 지도
    # ------------------------------------------------------------------
    def update(self, now):
        """장애물 지도 갱신. 반환: (pose, s, d, psi) 또는 None(odom 없음)."""
        pose = self.node.odom_pose()
        if pose is None:
            return None
        s_, d, psi = self.frame.car(pose)
        cfg = self.cfg
        half = 1.5 * self.width() + 0.15
        if self.ident is not None and self.ident['hyp'] is not None:
            half += self.width()            # 차선이 한 칸 밀렸을 수도 있는 동안은 한 차선 더 바깥 LIMO 도 봄 (확인용)

        def accept(so, do):
            return abs(do) <= half and so > s_ - 1.0

        self.map.update(self.frame, pose, self.node.obstacles(), now, accept,
                        cfg['min_obstacle_radius'], cfg['max_obstacle_radius'])
        self.map.prune(s_ - 1.5)
        items = self.map.confirmed(cfg['confirm_hits'])
        if items:
            last = max(it['s'] for it in items)
            self.max_obstacle_s = last if self.max_obstacle_s is None else max(self.max_obstacle_s, last)
        return pose, s_, d, psi

    def free_ahead(self, lane, s_car, back=0.0):
        """lane 에서 차(base_link) 앞쪽 가장 가까운 장애물 가장자리까지 거리 [m]. 없으면 inf."""
        best = float('inf')
        c = self.center(lane)
        for it in self.map.confirmed(self.cfg['confirm_hits']):
            if abs(it['d'] - c) > 0.5 * self.width():
                continue
            if it['s'] + it['r'] - s_car < -back:
                continue
            best = min(best, it['s'] - it['r'] - s_car)
        return best

    def obstacles_ahead(self, s_car):
        return [it for it in self.map.confirmed(self.cfg['confirm_hits'])
                if it['s'] + it['r'] > s_car]

    # ------------------------------------------------------------------
    # 미리 굴려보기 (rollout)
    # ------------------------------------------------------------------
    def _boxes(self, s_car):
        cfg = self.cfg
        hl = 0.5 * cfg['limo_length'] + cfg['clearance']
        hw = 0.5 * cfg['limo_width'] + cfg['clearance']
        return [(it['s'], it['d'], hl, hw)
                for it in self.map.confirmed(cfg['confirm_hits'])
                if it['s'] + hl > s_car - 0.5]

    def _shapes(self):
        """검사할 차 모양 [(반 길이, 반 폭), ...]. 실차는 몸통 하나.
        Gazebo 는 몸통 상자(앞뒤 0.19 x 좌우 0.31) + 바퀴 네 개를 감싼 사각형(0.33 x 0.258) 두 개 (11/04)."""
        cfg = self.cfg
        shapes = [(0.5 * cfg['car_length'], 0.5 * cfg['car_width'])]
        if cfg['wheel_box_length'] > 0.0 and cfg['wheel_box_width'] > 0.0:
            shapes.append((0.5 * cfg['wheel_box_length'], 0.5 * cfg['wheel_box_width']))
        return shapes

    @classmethod
    def _car_hits(cls, s_, d, psi, boxes, shapes):
        for half_l, half_w in shapes:
            if cls._rect_hits(s_, d, psi, boxes, half_l, half_w):
                return True
        return False

    @staticmethod
    def _rect_hits(s_, d, psi, boxes, half_l, half_w):
        c, sn = math.cos(psi), math.sin(psi)
        car = [(s_ + a * c - b * sn, d + a * sn + b * c)
               for a, b in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w))]
        cs = [p[0] for p in car]
        cd = [p[1] for p in car]
        for bs, bd, hl, hw in boxes:
            # 1) 박스 축으로 분리되는지
            if max(cs) < bs - hl or min(cs) > bs + hl or max(cd) < bd - hw or min(cd) > bd + hw:
                continue
            # 2) 차 축으로 분리되는지
            box = ((bs - hl, bd - hw), (bs + hl, bd - hw), (bs + hl, bd + hw), (bs - hl, bd + hw))
            separated = False
            for ax, ay in ((c, sn), (-sn, c)):
                pc = [x * ax + y * ay for x, y in car]
                pb = [x * ax + y * ay for x, y in box]
                if max(pc) < min(pb) or max(pb) < min(pc):
                    separated = True
                    break
            if not separated:
                return True
        return False

    def rollout(self, s0, d0, psi0, d_target, boxes, horizon, la=None):
        """d_target 차선으로 가는 주행을 horizon m 앞까지 굴려 보고, 닿기 전까지 간 거리 [m].
        la: Pure Pursuit 따라가는 거리 (None 이면 lookahead). 짧을수록 급하게 꺾는다."""
        return self._roll(s0, d0, psi0, d_target, boxes, horizon, la, 0.0)[0]

    def _roll(self, s0, d0, psi0, d_target, boxes, horizon, la, mark_step):
        """rollout + mark_step [m] 마다 지나간 상태 [(간 거리, s, d, psi)] (0 이면 기록 안 함)."""
        node = self.node
        cfg = self.cfg
        wb, ms = node.wheel_base, node.max_steer
        if la is None:
            la = cfg['lookahead']
        shapes = self._shapes()
        ds = 0.04
        s_, d, psi = s0, d0, psi0
        marks = []
        next_mark = mark_step if mark_step > 0.0 else float('inf')
        if self._car_hits(s_, d, psi, boxes, shapes):
            return 0.0, marks
        traveled = 0.0
        while traveled < horizon:
            c, sn = math.cos(psi), math.sin(psi)
            dx, dy = la, d_target - d
            bx, by = c * dx + sn * dy, -sn * dx + c * dy
            l2 = bx * bx + by * by
            kappa = 0.0 if l2 < 1e-9 else 2.0 * by / l2
            steer = clamp(math.atan(kappa * wb), -ms, ms)
            psi += ds * math.tan(steer) / wb
            s_ += ds * math.cos(psi)
            d += ds * math.sin(psi)
            traveled += ds
            if self._car_hits(s_, d, psi, boxes, shapes) or self._off_edge(d, psi):
                return traveled, marks
            if traveled >= next_mark - 1e-9:
                marks.append((traveled, s_, d, psi))
                next_mark += mark_step
        return horizon, marks

    def _off_edge(self, d, psi):
        """바퀴(Gazebo 는 바퀴 사각형, 실차는 몸통)가 3차선 바깥 선을 넘는지 (edge_check, 11/05)."""
        cfg = self.cfg
        if not cfg['edge_check']:
            return False
        if cfg['wheel_box_length'] > 0.0 and cfg['wheel_box_width'] > 0.0:
            hl, hw = 0.5 * cfg['wheel_box_length'], 0.5 * cfg['wheel_box_width']
        else:
            hl, hw = 0.5 * cfg['car_length'], 0.5 * cfg['car_width']
        ext = hl * abs(math.sin(psi)) + hw * abs(math.cos(psi))
        lim = 1.5 * self.width() + cfg['edge_margin']
        cut_right, cut_left = self.edge_shrink
        return d + ext > lim - cut_left or d - ext < -lim + cut_right

    def _las(self):
        cfg = self.cfg
        las = [cfg['lookahead']]
        if cfg['lookahead_short'] > 0.0:
            las.append(cfg['lookahead_short'])
        return las

    def plan(self, s_, d, psi, boxes):
        """차선마다 {'reach', 'la', 'free'} 와 굴려 보는 거리 H.
        reach: 그 차선으로 가다가 switch_step 마다 '여기서 옆 차선으로 한 번 더 옮기면' 까지 굴려 보고
               가장 멀리 가는 거리 (H 까지). 막다른 차선을 피하려고 (11/05, static_run3). plan_depth 1 이면 그 차선만.
        free:  그 차선으로만 갈 때 닿기 전까지 거리 (plan_horizon 까지, 감속·정지 판단에 씀).
        la:    그 차선으로 갈 때 따라갈 거리 (lookahead / lookahead_short 중 reach 가 긴 것)."""
        cfg = self.cfg
        depth = int(cfg['plan_depth'])
        H = max(cfg['plan_horizon_long'], cfg['plan_horizon']) if depth >= 2 else cfg['plan_horizon']
        step = cfg['switch_step'] if depth >= 2 else 0.0
        out = {}
        for l1 in self.LANES:
            best = None
            for la in self._las():
                f1, marks = self._roll(s_, d, psi, self.center(l1), boxes, H, la, step)
                r = f1
                if depth >= 2 and f1 < H - 1e-6:
                    for a, ss, dd, pp in marks:
                        if a >= f1 - 1e-6 or r >= H - 1e-6:
                            break
                        for l2 in self.LANES:
                            if l2 != l1:
                                r = max(r, a + self.rollout(ss, dd, pp, self.center(l2), boxes, H - a))
                if best is None or r > best['reach'] + 1e-6:
                    best = {'reach': r, 'la': la, 'free': min(f1, cfg['plan_horizon'])}
                if best['reach'] >= H - 1e-6:
                    break
            out[l1] = best
        return out, H

    def _all_boxes(self):
        cfg = self.cfg
        hl = 0.5 * cfg['limo_length'] + cfg['clearance']
        hw = 0.5 * cfg['limo_width'] + cfg['clearance']
        return [(it['s'], it['d'], hl, hw) for it in self.map.confirmed(cfg['confirm_hits'])]

    def _relax(self, boxes, s_, d, psi, who=None):
        """(11/12) 지금 자리에서 이미 겹친 박스는 지금 안 겹칠 만큼(2 mm 씩) + overlap_slack 만큼 줄인다
        (clearance + overlap_extra 까지). overlap_slack 은 방향 틀 때 차 뒤쪽이 옆으로 도는 몫 (없으면 멀어지는 길도 4 cm 에서 막힘).
        그만큼 줄여도 겹치면(진짜로 닿았다고 봄) 그대로 둔다. 그 LIMO 에 지금보다 overlap_slack 넘게 가까워지는 길은 막힘."""
        cfg = self.cfg
        if not cfg['overlap_relax']:
            return boxes
        shapes = self._shapes()
        limit = cfg['clearance'] + cfg['overlap_extra']
        out, relaxed = [], []
        for b in boxes:
            if not self._car_hits(s_, d, psi, [b], shapes):
                out.append(b)
                continue
            bs, bd, hl, hw = b
            nb, k = b, 0.0
            while k < limit - 1e-9:
                k = min(k + 0.002, limit)
                if not self._car_hits(s_, d, psi, [(bs, bd, hl - k, hw - k)], shapes):
                    k = min(k + cfg['overlap_slack'], limit)
                    nb = (bs, bd, hl - k, hw - k)
                    break
            out.append(nb)
            if nb is not b:
                relaxed.append((bs, bd, k))
        if who is not None:
            self.node.dbg('static/relaxed', float(len(relaxed)))
            if relaxed and not self.relax_logged:
                self.node.get_logger().warn(
                    '[%s] 지금 자리에서 LIMO 여유 박스와 겹침 - 그 LIMO 는 여유를 줄여서 봄 (지금보다 크게 가까워지는 길은 막힘) (%s)'
                    % (who, ', '.join('s=%.2f d=%+.2f %.1fcm' % (a, b_, 100.0 * k_) for a, b_, k_ in relaxed)))
            self.relax_logged = bool(relaxed)
        return out

    def _back_floor(self, maneuver=False):
        """(11/16 사용자) 후진 한계 s = 첫 LIMO 가운데 (+ back_floor_offset). 이보다 뒤는 입구 사선 구간으로 봄 (_behind_floor).
        3차선 입구는 0.55 m 동안 45도 사선으로 넓어지는데 바깥 선 검사(edge_check)는 전체 폭으로 봐서, 입구 쪽으로 후진하면
        사선을 넘는다 (static_1116_1). 입구 선 위치를 모르므로 '첫 LIMO 가운데에서 다 넓어진다' 로 본다. 끄면 None."""
        cfg = self.cfg
        if not cfg['back_floor'] or (maneuver and not cfg['maneuver_back_floor']):
            return None
        items = self.map.confirmed(cfg['confirm_hits'])
        if not items:
            return None
        return min(it['s'] for it in items) + cfg['back_floor_offset']

    def _behind_floor(self, s_, d, psi, floor):
        """후진 한계(첫 LIMO 가운데)보다 뒤에서 입구 사선 밖으로 나가는지. 한계 뒤로는 3차선 폭이 뒤로 1 m 갈 때마다
        back_floor_slope m 씩 (1.0 = 45도) 좁아져 가운데 차선 폭까지 줄어든다고 보고, 차 네 모서리를 검사한다.
        (가운데 차선은 들어온 차선과 곧게 이어져서 그 폭 안은 늘 괜찮음. slope 를 크게 하면 한계 뒤 = 가운데 차선 폭만.)"""
        if floor is None:
            return False
        cfg = self.cfg
        hl, hw = 0.5 * cfg['car_length'], 0.5 * cfg['car_width']
        c, sn = math.cos(psi), math.sin(psi)
        if s_ - (hl * abs(c) + hw * abs(sn)) >= floor:
            return False
        w = self.width()
        for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
            cs = s_ + a * c - b * sn
            if cs >= floor:
                continue
            cd = d + a * sn + b * c
            if abs(cd) > max(0.5 * w, 1.5 * w - cfg['back_floor_slope'] * (floor - cs)) + cfg['edge_margin']:
                return True
        return False

    def back_steer(self, psi):
        """후진하면서 차 방향을 차선 방향(psi = 0)으로 펴는 조향각. 후진(v < 0)이면 psi 가 줄어든다."""
        return clamp(self.cfg['backup_k_heading'] * psi, -self.node.max_steer, self.node.max_steer)

    def _back_sim(self, s0, d0, psi0, boxes, dist, steer=None, marks=()):
        """후진하며 dist 만큼 굴려 봄. steer None = back_steer(차선 방향으로 펴기), 아니면 그 조향 고정.
        (닿기 전까지 간 거리, [(marks 거리, s, d, psi) ...]). LIMO 박스에 닿거나 바깥 선을 넘기(edge_check) 직전에 멈춤 (11/16)."""
        wb = self.node.wheel_base
        shapes = self._shapes()
        ds = 0.02
        s_, d, psi = s0, d0, psi0
        traveled = 0.0
        got = []
        todo = sorted(m for m in marks if m > 0.0)
        floor = self._back_floor()
        while traveled < dist - 1e-9:
            st = self.back_steer(psi) if steer is None else steer
            psi -= ds * math.tan(st) / wb
            s_ -= ds * math.cos(psi)
            d -= ds * math.sin(psi)
            traveled += ds
            if (self._car_hits(s_, d, psi, boxes, shapes) or self._off_edge(d, psi)
                    or self._behind_floor(s_, d, psi, floor)):
                return traveled - ds, got
            while todo and traveled >= todo[0] - 1e-9:
                got.append((todo.pop(0), s_, d, psi))
        return dist, got

    def rollout_back(self, s0, d0, psi0, boxes, dist, steer=None):
        """후진하며 dist 만큼 굴려 보고, 닿기(또는 바깥 선 넘기) 전까지 간 거리 [m]."""
        return self._back_sim(s0, d0, psi0, boxes, dist, steer)[0]

    # ---------------- (11/16 사용자) 앞뒤로 움직여 빠져나가기 (하이브리드 A*) ----------------
    def _maneuver_plan(self, s0, d0, psi0, boxes, now):
        """막힌 자리에서 앞/뒤 x 조향(왼쪽 끝/0/오른쪽 끝) 조각을 이어 붙여 앞으로 maneuver_goal_ahead 넘게 나가는 길.
        LIMO 박스에 닿거나 바깥 선을 넘거나 시작 자리보다 maneuver_max_back 넘게 뒤로 가는 길은 버림.
        반환: [{'dir', 'steer', 'len', 'end': (s, d, psi)}] (같은 조각은 합침) 또는 None."""
        import heapq
        import time as _time
        cfg = self.cfg
        wb = self.node.wheel_base
        ms = self.node.max_steer
        step = cfg['maneuver_step']
        n_sub = max(2, int(round(step / 0.035)))
        ds = step / n_sub
        shapes = self._shapes()
        s_min = s0 - cfg['maneuver_max_back']
        goal_s = s0 + cfg['maneuver_goal_ahead']
        goal_psi = math.radians(cfg['maneuver_goal_heading_deg'])
        band = cfg['maneuver_goal_lane_tol']
        reach_s = goal_s + 0.6
        bxs = [b for b in boxes if s_min - 0.6 <= b[0] <= reach_s]
        prims = [(dr, st) for dr in (1, -1) for st in (-ms, 0.0, ms)]
        rs, rd, rp = 0.04, 0.04, math.radians(7.5)
        floor = self._back_floor(maneuver=True)

        def move(s_, d, psi, dr, st):
            for _ in range(n_sub):
                psi += dr * ds * math.tan(st) / wb
                s_ += dr * ds * math.cos(psi)
                d += dr * ds * math.sin(psi)
                if (s_ < s_min or self._off_edge(d, psi) or self._car_hits(s_, d, psi, bxs, shapes)
                        or (dr < 0 and self._behind_floor(s_, d, psi, floor))):
                    return None
            return s_, d, psi

        def is_goal(s_, d, psi):
            return (s_ >= goal_s and abs(psi) <= goal_psi
                    and min(abs(d - self.center(ln)) for ln in self.LANES) <= band)

        t_end = _time.time() + cfg['maneuver_time_budget']
        nodes = [((s0, d0, psi0), -1, None)]
        openq = [(max(0.0, goal_s - s0), 0.0, 0, 0, None)]
        seen = {}
        found = None
        n = 0
        while openq and n < cfg['maneuver_max_nodes']:
            if (n & 255) == 0 and _time.time() > t_end:
                break
            _, g, idx, prev, prev_st = heapq.heappop(openq)
            n += 1
            st_ = nodes[idx][0]
            if is_goal(*st_):
                found = idx
                break
            k = (int(round(st_[0] / rs)), int(round(st_[1] / rd)), int(round(st_[2] / rp)))
            if k in seen and seen[k] <= g:
                continue
            seen[k] = g
            for dr, st in prims:
                nx = move(st_[0], st_[1], st_[2], dr, st)
                if nx is None or abs(nx[2]) > math.radians(100.0):
                    continue
                ng = (g + step * (1.0 if dr > 0 else cfg['maneuver_back_cost'])
                      + (cfg['maneuver_switch_cost'] if prev != 0 and dr != prev else 0.0)
                      + (cfg['maneuver_steer_cost'] if prev_st is not None and st != prev_st else 0.0))
                nk = (int(round(nx[0] / rs)), int(round(nx[1] / rd)), int(round(nx[2] / rp)))
                if nk in seen and seen[nk] <= ng:
                    continue
                nodes.append((nx, idx, (dr, st)))
                heapq.heappush(openq, (ng + max(0.0, goal_s - nx[0]), ng, len(nodes) - 1, dr, st))
        self.node.dbg('static/maneuver_nodes', float(n))
        if found is None:
            return None
        seq = []
        i = found
        while i > 0:
            st_, pi, prim = nodes[i]
            seq.append((prim, st_))
            i = pi
        seq.reverse()
        segs = []
        for (dr, st), end in seq:
            if segs and segs[-1]['dir'] == dr and segs[-1]['steer'] == st:
                segs[-1]['len'] += step
                segs[-1]['end'] = end
            else:
                segs.append({'dir': dr, 'steer': st, 'len': step, 'end': end})
        return segs

    def _start_maneuver(self, pose, s_, d, psi, now, who, why):
        cfg = self.cfg
        log = self.node.get_logger()
        import time as _time
        t0 = _time.time()
        # 길은 박스를 maneuver_margin 만큼 키워서 찾음 (움직일 땐 원래 박스로 확인 -> 그만큼 어긋나도 됨).
        #   그렇게는 길이 없으면 절반, 그래도 없으면 안 키우고 찾음 (촘촘한 배치)
        segs = None
        for mg in (cfg['maneuver_margin'], 0.5 * cfg['maneuver_margin'], 0.0):
            big = [(bs, bd, hl + mg, hw + mg) for bs, bd, hl, hw in self._all_boxes()]
            segs = self._maneuver_plan(s_, d, psi, self._relax(big, s_, d, psi), now)
            if segs is not None or mg <= 0.0:
                break
        dt = _time.time() - t0
        if segs is None:
            log.warn('[%s] %s - 앞뒤로 움직여 빠져나갈 길도 못 찾음 (계산 %.1f초)' % (who, why, dt))
            self.maneuver = None
            return False
        flips = sum(1 for a, b in zip(segs, segs[1:]) if a['dir'] != b['dir'])
        log.warn('[%s] %s - 앞뒤로 움직여 빠져나감: 조각 %d개, 앞뒤 바꿈 %d번, 총 %.1fm (계산 %.1f초) [%s]'
                 % (who, why, len(segs), flips, sum(g['len'] for g in segs), dt,
                    ' '.join('%s%.1f%s' % ('앞' if g['dir'] > 0 else '뒤', g['len'],
                                           'L' if g['steer'] > 0 else ('R' if g['steer'] < 0 else '0')) for g in segs)))
        self.maneuver = {'segs': segs, 'i': 0, 'xy': (pose[0], pose[1]),
                         'settle_until': now + cfg['maneuver_settle'], 'replans': 0}
        return True

    def _maneuver_step(self, pose, s_, d, psi, now, who):
        """앞뒤 움직임 한 박자. 계속 움직이는 중이면 True (이번 박자 명령 냄), 끝났거나 그만두면 False."""
        node = self.node
        cfg = self.cfg
        m = self.maneuver
        seg = m['segs'][m['i']]
        node.dbg('static/mode', 'maneuver')
        node.dbg('static/maneuver_seg', float(m['i']))
        if now < m['settle_until']:
            node.publish_steer_hold(seg['steer'])           # 서서 바퀴부터 돌려 놓음
            return True
        done = math.hypot(pose[0] - m['xy'][0], pose[1] - m['xy'][1])
        left = seg['len'] - done
        bxs = self._relax(self._all_boxes(), s_, d, psi)
        look = min(0.06, max(0.0, left) + 0.02)
        sl = cfg['maneuver_check_slack']        # 움직일 때 확인은 이만큼 덜 깐깐하게 (계획과 몇 mm 차이로 멈추지 않게)
        chk = [(bs, bd, hl - sl, hw - sl) for bs, bd, hl, hw in bxs]
        if self._seg_hits(s_, d, psi, seg['dir'], seg['steer'], look, chk):
            return self._replan_maneuver(pose, s_, d, psi, now, who, '움직이다 LIMO 에 닿을 것 같음')
        if left <= 0.0:
            es, ed, ep = seg['end']
            off_d, off_p = abs(d - ed), abs(normalize_angle(psi - ep))
            m['i'] += 1
            m['xy'] = (pose[0], pose[1])
            if m['i'] >= len(m['segs']):
                node.get_logger().info('[%s] 앞뒤로 움직여 빠져나옴 (s=%.2f d=%+.2f %.0f도) - 다시 차선 따라감'
                                       % (who, s_, d, math.degrees(psi)))
                self.maneuver = None
                self.blocked_since = None
                self.logged_blocked = False
                return False
            if off_d > cfg['maneuver_tol_d'] or off_p > math.radians(cfg['maneuver_tol_deg']):
                return self._replan_maneuver(pose, s_, d, psi, now, who,
                                             '예상 자리와 어긋남 (옆 %.2fm, 방향 %.0f도)' % (off_d, math.degrees(off_p)))
            nxt = m['segs'][m['i']]
            if nxt['dir'] > 0 and seg['dir'] > 0 and abs(psi) <= math.radians(cfg['maneuver_handback_deg']):
                # 좁은 데를 빠져나왔으면 (보통 계획으로 끝까지 갈 차선이 있으면) 남은 조각은 버리고 차선 따라가기로
                info, horizon = self.plan(s_, d, psi, bxs)
                if max(v_['reach'] for v_ in info.values()) >= horizon - 1e-6:
                    node.get_logger().info('[%s] 좁은 데를 빠져나옴 (s=%.2f d=%+.2f %.0f도, 조각 %d/%d) - 다시 차선 따라감'
                                           % (who, s_, d, math.degrees(psi), m['i'], len(m['segs'])))
                    self.maneuver = None
                    self.blocked_since = None
                    self.logged_blocked = False
                    return False
            if nxt['dir'] != seg['dir'] or abs(nxt['steer'] - seg['steer']) >= cfg['maneuver_settle_steer']:
                m['settle_until'] = now + cfg['maneuver_settle']
                node.publish_stop()
                return True
            seg = nxt
        v = cfg['maneuver_speed'] if seg['dir'] > 0 else -cfg['maneuver_back_speed']
        node.publish_cmd(v, steering_to_omega(v, seg['steer'], node.wheel_base))
        return True

    def _seg_hits(self, s_, d, psi, dr, st, dist, boxes):
        wb = self.node.wheel_base
        shapes = self._shapes()
        ds = 0.02
        t = 0.0
        while t < dist - 1e-9:
            psi += dr * ds * math.tan(st) / wb
            s_ += dr * ds * math.cos(psi)
            d += dr * ds * math.sin(psi)
            t += ds
            if self._car_hits(s_, d, psi, boxes, shapes) or self._off_edge(d, psi):
                return True
        return False

    def _replan_maneuver(self, pose, s_, d, psi, now, who, why):
        self.node.publish_stop()
        m = self.maneuver
        if m['replans'] >= self.cfg['maneuver_max_replans']:
            self.node.get_logger().warn('[%s] %s - 다시 찾기 %d번 넘음, 앞뒤 움직임 그만' % (who, why, m['replans']))
            self.maneuver = None
            return False
        replans = m['replans'] + 1
        if self._start_maneuver(pose, s_, d, psi, now, who, why + ' - 다시 찾음'):
            self.maneuver['replans'] = replans
            return True
        return False

    def _choose_backup(self, s_, d, psi, boxes, now_reach):
        """(11/16) 후진 후보 중 앞으로 갈 거리가 가장 길어지는 것. (조향 또는 None, 거리, 그 뒤 앞 거리) 또는 None."""
        cfg = self.cfg
        ms = self.node.max_steer
        dists = [float(v) for v in cfg['backup_search_dists']]
        best = None
        for st in (None, -ms, -0.5 * ms, 0.0, 0.5 * ms, ms):
            _, got = self._back_sim(s_, d, psi, boxes, max(dists), st, dists)
            for r, ss, dd, pp in got:
                info, _ = self.plan(ss, dd, pp, boxes)
                ahead = max(v['reach'] for v in info.values())
                key = (round(ahead, 3), -r, -(0.0 if st is None else abs(st)))
                if best is None or key > best[0]:
                    best = (key, st, r, ahead)
        if best is None or best[3] < now_reach + cfg['backup_min_gain']:
            return None
        return best[1], best[2], best[3]

    def _back_step(self, pose, s_, d, psi, now, who):
        """후진 중 한 주기. 다 했으면 False (다시 계산으로)."""
        cfg = self.cfg
        node = self.node
        x0, y0, steer, dist = self.backing
        done = math.hypot(pose[0] - x0, pose[1] - y0)
        if (done >= dist
                or self.rollout_back(s_, d, psi, self._relax(self._all_boxes(), s_, d, psi), 0.04, steer) < 0.04):
            node.get_logger().info('[%s] 후진 %.2fm 끝 - 다시 계산' % (who, done))
            self.backing = None
            self.blocked_since = None
            self.logged_blocked = False
            return False
        v = -cfg['backup_speed']
        st = self.back_steer(psi) if steer is None else steer
        node.publish_cmd(v, steering_to_omega(v, st, node.wheel_base))
        node.dbg('static/mode', 'backup')
        return True

    # ------------------------------------------------------------------
    # 한 제어 주기
    # ------------------------------------------------------------------
    def step(self, now, goal=None, who='3차선'):
        """반환: StepResult 또는 None(odom 없음). 길이 없어 서 있으면 result.blocked."""
        node = self.node
        cfg = self.cfg
        log = node.get_logger()
        car = self.update(now)
        if car is None:
            return None
        pose, s_, d, psi = car
        if self.lat_snap_pending and self._snap_lateral(s_, who):
            self.lat_snap_pending = False
            s_, d, psi = self.frame.car(pose)
        if self.ident is not None and self._ident_step(pose, s_, d, psi, now, who):
            s_, d, psi = self.frame.car(pose)
        node.dbg('static/lat_offset', self.lat_offset)

        if self.changing and self.settled(d, psi):
            self.lane = self.target
            self.changing = False
            log.info('[%s] 차선 변경 완료 - %s 차선 (d=%.2fm)' % (who, self.name(self.lane), d))
        if not self.changing:
            self.check_lane(d, now, who)
        elif cfg['lane_from_odom']:
            self.lane = self.lane_of(d)     # 바꾸는 중에도 지금 있는 차선 (11/12)

        if self.backing is not None and self._back_step(pose, s_, d, psi, now, who):
            return StepResult(pose, s_, d, psi, blocked=True)
        if self.maneuver is not None and self._maneuver_step(pose, s_, d, psi, now, who):
            return StepResult(pose, s_, d, psi, blocked=True)

        boxes = self._relax(self._boxes(s_), s_, d, psi, who)
        # 차선마다 굴려 봄: 보통(lookahead) / 급하게(lookahead_short, 11/04) x 한 번 더 옆 차선으로 옮기기(11/05)
        info, horizon = self.plan(s_, d, psi, boxes)
        reach = dict((ln, info[ln]['reach']) for ln in self.LANES)
        free = dict((ln, info[ln]['free']) for ln in self.LANES)
        full = [ln for ln in self.LANES if reach[ln] >= horizon - 1e-6]
        node.dbg('static/free_target', free[self.target])
        node.dbg('static/reach_target', reach[self.target])

        def score(ln):
            sc = reach[ln] - 0.02 * abs(ln - self.lane)
            if goal is not None and ln == goal:
                sc += 0.3
            return sc

        old = self.target
        held = now - self.target_since
        if cfg['lane_commit'] and self.changing and self.lane != old:
            # (11/12) 목표 차선으로 가는 중: 고정. 목표가 곧 막히고 switch_hold 가 지났을 때만 다시 고름
            new = old
            if free[old] < cfg['commit_distance'] and held >= cfg['switch_hold']:
                best = max(self.LANES, key=score)
                if best != old and score(best) > score(old) + cfg['switch_margin']:
                    new = best
        elif goal is not None and goal in full:
            new = goal
        elif old in full:
            new = old
        else:
            # 자꾸 바꾸지 않게: switch_margin 이상 나을 때만, 바꾼 뒤 switch_hold 초는 유지
            # (지금 목표가 commit_distance 안에서 막히면 바로 바꿈)
            best = max(self.LANES, key=score)
            new = old
            if best != old and score(best) > score(old) + cfg['switch_margin']:
                if held >= cfg['switch_hold'] or free[old] < cfg['commit_distance']:
                    new = best

        if new != old:
            log.info('[%s] 목표 차선 %s -> %s (지금 %s / 앞 %.1fm 굴려본 결과 %s)'
                     % (who, self.name(old), self.name(new), self.name(self.lane), horizon,
                        ' '.join('%s=%.2f' % (self.name(ln), reach[ln]) for ln in self.LANES)))
            self.target = new
            self.target_since = now
        if self.target != self.lane:
            self.changing = True

        v = cfg['speed']
        if self.ident is not None and self.ident['hyp'] is not None:
            v = min(v, cfg['slow_speed'])       # 몇 번 차선인지 확인 중 - 천천히
        if free[self.target] < cfg['commit_distance']:
            if not self.logged_blocked:
                log.warn('[%s] 갈 수 있는 거리가 짧음 (최대 %.2fm) - 감속, %.2fm 안이면 정지'
                         % (who, free[self.target], cfg['stop_distance']))
                self.logged_blocked = True
            if free[self.target] <= cfg['stop_distance']:
                node.publish_stop()
                node.dbg('static/mode', 'stop')
                if self.blocked_since is None:
                    self.blocked_since = now
                elif now - self.blocked_since >= cfg['backup_after']:
                    bxs = self._relax(self._all_boxes(), s_, d, psi)
                    pick = None
                    if cfg['backup_mode'] == 'search' and self.backups < cfg['backup_max']:
                        pick = self._choose_backup(s_, d, psi, bxs, max(reach.values()))
                    # (11/16) 한 번 후진으로 충분히 뚫리면 (앞 maneuver_if_reach_below 넘게) 그게 빠름.
                    #   아니면 앞뒤로 움직여 빠져나갈 길을 찾고, 그것도 없으면 후진
                    quick = (pick is not None and pick[2] >= cfg['maneuver_if_reach_below']
                             and (self.last_backup_s is None
                                  or abs(s_ - self.last_backup_s) > cfg['maneuver_after_backup_within']))
                    if (not quick and cfg['maneuver'] and self.maneuvers < cfg['maneuver_max']
                            and self._start_maneuver(pose, s_, d, psi, now, who,
                                                     '%.1f초 막힘 (차선별 갈 거리 %s)' % (
                                                         now - self.blocked_since,
                                                         ' '.join('%s=%.2f' % (self.name(ln), free[ln])
                                                                  for ln in self.LANES)))):
                        self.maneuvers += 1
                        return StepResult(pose, s_, d, psi, blocked=True)
                    if self.backups >= cfg['backup_max']:
                        return StepResult(pose, s_, d, psi, blocked=True)
                    if pick is not None:
                        st, dist, ahead = pick
                        how = '조향 %+.2f' % st if st is not None else '차선 방향으로 펴며'
                        room = dist
                        extra = ' -> 그 뒤 앞으로 %.2fm (지금 %.2fm)' % (ahead, max(reach.values()))
                    else:
                        st, how, extra = None, '차선 방향으로 펴며', ''
                        room = min(self.rollout_back(s_, d, psi, bxs, cfg['backup_distance']), cfg['backup_distance'])
                    if room < 0.05 and not self.logged_floor:
                        fl = self._back_floor()
                        if fl is not None and self._behind_floor(s_ - 0.05 * math.cos(psi), d - 0.05 * math.sin(psi),
                                                                 psi, fl):
                            log.warn('[%s] %.1f초 막힘 - 첫 LIMO(s=%.2f) 뒤 입구 사선 구간이라 후진 안 함 (지금 s=%.2f d=%+.2f)'
                                     % (who, now - self.blocked_since, fl, s_, d))
                            self.logged_floor = True
                    if room >= 0.05:
                        self.backups += 1
                        self.last_backup_s = s_
                        self.backing = (pose[0], pose[1], st, room)
                        log.warn('[%s] %.1f초 막힘 - %.2fm 후진 (%s)%s 후 다시 계산 (%d/%d번째, 차선별 갈 거리 %s)'
                                 % (who, now - self.blocked_since, room, how, extra,
                                    self.backups, cfg['backup_max'],
                                    ' '.join('%s=%.2f' % (self.name(ln), free[ln]) for ln in self.LANES)))
                return StepResult(pose, s_, d, psi, blocked=True)
            v = cfg['slow_speed']
            self.blocked_since = None
        else:
            self.logged_blocked = False
            self.blocked_since = None

        node.dbg('static/lane', self.lane)
        node.dbg('static/target', self.target)
        node.dbg('static/d', d)
        node.dbg('static/s', s_)
        node.dbg('static/lookahead', info[self.target]['la'])
        node.publish_cmd(v, self.steer(pose, s_, self.center(self.target), v, info[self.target]['la']))
        return StepResult(pose, s_, d, psi)

    def settled(self, d, psi):
        return (abs(d - self.center(self.target)) <= self.cfg['settle_tolerance']
                and abs(psi) <= self.cfg['settle_heading'])

    def check_lane(self, d, now, who='3차선'):
        """차선 변경 중이 아닐 때 odom 위치와 차선 번호가 계속 다르면 odom 을 따른다."""
        actual = self.lane_of(d)
        if actual == self.lane:
            self.mismatch_since = None
            return
        if self.mismatch_since is None:
            self.mismatch_since = now
        elif now - self.mismatch_since >= 0.5:
            self.node.get_logger().warn(
                '[%s] 차선 번호 %d 와 odom 위치(%s, d=%.2fm)가 달라 odom 을 따름'
                % (who, self.lane, self.name(actual), d))
            self.lane = self.target = actual
            self.mismatch_since = None

    def steer(self, pose, s_car, d_target, v, la=None):
        omega, _ = pursue(self.node, self.frame, pose,
                          s_car + (self.cfg['lookahead'] if la is None else la), d_target, v)
        return omega

    def gap_step(self):
        """odom 이 없을 때만 쓰는 비상 주행: 전방 스캔에서 가장 넓은 빈 공간 쪽으로 저속."""
        node = self.node
        cfg = self.cfg
        v = cfg['gap_speed']
        points = node.forward_scan_points(cfg['gap_range'])
        gap = find_largest_gap(points, cfg['robot_width'], cfg['gap_range'])
        if gap is None:
            node.publish_cmd(v, 0.0)
            return
        steer = clamp(cfg['gap_gain'] * gap[0], -node.max_steer, node.max_steer)
        node.publish_cmd(v, steering_to_omega(v, steer, node.wheel_base))
