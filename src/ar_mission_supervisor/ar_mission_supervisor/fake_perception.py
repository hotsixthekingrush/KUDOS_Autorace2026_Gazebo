"""테스트용 가짜 인지 노드 + 간이 차량 시뮬레이터 (Autorace 2026 미션).

- 시나리오를 파라미터로 골라 인지 토픽(YOLO, 정지선, 라이다)을 대신 발행한다.
- 미션 물체는 "시나리오 시작 위치 기준 앞쪽 distance m" 에 고정되고,
  차가 움직이면 남은 거리가 줄어든다.
- publish_odom:=true (기본) 이면 /cmd_vel 을 적분해 /odom 을 직접 발행한다.
  publish_odom:=false 이면 Gazebo 의 /odom 을 받아 차 위치로 쓴다.
실차에서는 절대 함께 실행하지 말 것.

시나리오 (-p scenario:=...)
  lane            차선만
  traffic_light   신호등 + 정지선. 색은 -p light_color:=red|yellow|green (실행 중 param set 가능)
  pedestrian      보행자 표지판 + 정지선
  bump            서행 표지판
  slow_zone       distance 앞에서 시작하는 지그재그 유도선(zigzag_length) + 0.2m 뒤 보행자 정지선
  sign_a, sign_b  갈림길 A / B 표지판
  cone            좌우 라바콘 3쌍 (라이다 cone 이벤트)
  static_3lane    3차선 정차 LIMO 7대 (가운데 차선에서 시작) + 끝에 A/B 표지판(-p sign:=A|B|none)
                  (-p spacing:=LIMO 진행방향 간격, 기본 0.55 = 연습 Gazebo 배치)
  parking         2026 주차: 오른쪽 칸, 목표 칸 양옆 LIMO 2대 + 아랫줄 LIMO 1대 + 주차 표지판
                  (-p gap:=목표 칸 중심까지 거리)
  * static_3lane / parking 에서 -p fake_limos:=false 면 가짜 LIMO 를 안 보내고
    (차선·표지판만 보냄) Gazebo 의 진짜 LIMO 를 라이다 노드가 보게 한다.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from ar_msgs.msg import (Obstacle, ObstacleArray, LaneError,
                         MissionObject, MissionObjectArray)


# --- 2026 트랙 치수 (도면) ---------------------------------------------------
LANE_PITCH = 0.52          # 3차선 차선 중앙 간격 (차선 500 + 점선 20 mm)
STATIC_LANES = (1, 2, 0, 1, 2, 0, 1)   # 0 왼쪽, 1 가운데, 2 오른쪽 (연습 Gazebo 배치)
ISLAND_AFTER = 0.5         # 마지막 LIMO 뒤 A/B 섬(가운데 차선 막힘)이 시작하는 거리
BAY_PITCH = 0.52           # 주차 칸 간격 (칸 500 + 선 20 mm)
BAY_DEPTH = 0.60
BAY_ENTRY = 0.27           # 차선 중앙 -> 칸 입구 선 (차선 반폭 0.25 + 선 0.02)
LIMO_LEN, LIMO_WID, LIMO_R = 0.322, 0.22, 0.16

LIGHT = {
    'red': MissionObject.LIGHT_RED,
    'yellow': MissionObject.LIGHT_YELLOW,
    'green': MissionObject.LIGHT_GREEN,
    'unknown': MissionObject.LIGHT_UNKNOWN,
}


class FakePerception(Node):

    def __init__(self):
        super().__init__('fake_perception')

        self.declare_parameter('scenario', 'lane')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('lateral_error', 0.0)
        self.declare_parameter('heading_error', 0.0)
        self.declare_parameter('distance', 1.5)        # 정지선/표지판까지 거리 (시나리오 시작 기준)
        self.declare_parameter('sign_position_valid', True)  # bump: false 면 표지판 위치 없이 보냄 (시험용)
        self.declare_parameter('sign_lateral', -0.35)        # bump: 표지판 좌우 위치 (왼쪽 +) [m]
        self.declare_parameter('side_lateral', -1.15)        # side_signs: 신호등·주차 표지판 좌우 위치 (왼쪽 +) [m]
        self.declare_parameter('lane_valid', True)           # false 면 차선 valid=false (차선 유실 시험)
        self.declare_parameter('lane_confidence', 1.0)       # 차선 확신도 (코너 감속 시험)
        self.declare_parameter('lane_single', False)        # single_lane_estimated (한 선 추정 시험, 11/11)
        self.declare_parameter('crosswalk_after_line', 0.06) # 정지선 -> 횡단보도 줄무늬 앞끝 (판단 yolo.crosswalk_to_stopline 과 같게)
        self.declare_parameter('light_color', 'red')
        self.declare_parameter('stopline_visible', 1.2)   # 횡단보도(정지 기준) 위치가 이 거리 안에서만 나옴
        self.declare_parameter('stopline_min', 0.25)      # 이보다 가까우면 박스 아래가 화면 밖 (위치 없음)
        self.declare_parameter('zigzag_length', 1.677)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('sign', 'A')            # static_3lane 끝의 A/B 표지판 (none = 없음)
        self.declare_parameter('gap', 1.5)             # parking: 목표 칸 중심까지 거리
        self.declare_parameter('spacing', 0.55)        # static_3lane: LIMO 진행방향 간격 (연습 Gazebo 0.55)
        self.declare_parameter('fake_limos', True)     # false: 가짜 LIMO 안 보냄 (진짜 라이다와 같이 쓸 때)
        self.declare_parameter('cmd_is_steering', False)  # publish_odom 일 때: angular.z 를 조향각으로 해석 (판단 sim 설정과 맞춤)
        self.declare_parameter('wheel_base', 0.24)
        self.declare_parameter('steer_rate_deg', 0.0)   # >0 이면 조향이 이 속도로만 바뀜 (Gazebo 느린 조향 흉내, 시험용)
        self.declare_parameter('start_yaw_deg', 0.0)    # publish_odom 일 때 처음 차 방향 (차선은 계속 오차 0 으로 보냄 = 기울어진 진입 시험, 11/10)
        self.declare_parameter('limo_depth_offset', 0.0)  # parking: 윗줄 LIMO 를 칸 가운데보다 이만큼 더 안쪽(+)에 세움 [m]

        pub = self.create_publisher
        self.pub_lane = pub(LaneError, '/perception/lane_error', 10)
        self.pub_event = pub(String, '/perception/event', 10)
        self.pub_objects = pub(MissionObjectArray, '/perception/mission_objects', 10)
        self.pub_obstacles = pub(ObstacleArray, '/perception/obstacles', 10)

        # --- 차 위치 ---
        self.x = self.y = self.yaw = 0.0
        self.origin = (0.0, 0.0, 0.0)      # 시나리오 시작 시 차 위치
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.steer = 0.0
        self.sim_dt = 0.02
        self.last_scenario = None

        self.own_odom = bool(self._p('publish_odom'))
        if self.own_odom:
            self.pub_odom = pub(Odometry, '/odom', 10)
            self.create_subscription(Twist, '/cmd_vel', self._cb_cmd, 10)
            self.create_timer(self.sim_dt, self._sim_step)
        else:
            self.create_subscription(Odometry, '/odom', self._cb_odom, 10)

        rate = float(self._p('rate'))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('가짜 인지 시작 - scenario=%s' % self._p('scenario'))

    # ------------------------------------------------------------------
    # 차 위치
    # ------------------------------------------------------------------
    def _cb_cmd(self, msg):
        self.cmd_v = msg.linear.x
        wb = float(self._p('wheel_base'))
        if self._p('cmd_is_steering'):
            # Gazebo 아커만 플러그인과 같은 해석: angular.z = 조향각, 후진이면 부호 반대
            self.cmd_steer = max(-0.5236, min(0.5236, msg.angular.z * math.copysign(1.0, msg.linear.x)))
            self.cmd_w = msg.linear.x * math.tan(self.cmd_steer) / wb
        else:
            self.cmd_w = msg.angular.z
            v = msg.linear.x
            self.cmd_steer = math.atan(wb * msg.angular.z / v) if abs(v) > 1e-3 else 0.0

    def _cb_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _sim_step(self):
        dt = self.sim_dt
        rate = math.radians(float(self._p('steer_rate_deg')))
        if rate > 0.0:
            # 조향이 rate 로만 따라가는 차 (Gazebo 11/02 녹화: 약 3.4도/s)
            d = getattr(self, 'cmd_steer', 0.0) - self.steer
            self.steer += max(-rate * dt, min(rate * dt, d))
            self.cmd_w = self.cmd_v * math.tan(self.steer) / float(self._p('wheel_base'))
        self.x += self.cmd_v * math.cos(self.yaw) * dt
        self.y += self.cmd_v * math.sin(self.yaw) * dt
        self.yaw += self.cmd_w * dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.cmd_v
        odom.twist.twist.angular.z = self.cmd_w
        self.pub_odom.publish(odom)

    def _to_base(self, wx, wy):
        """시나리오 좌표(시작 위치 원점, 시작 방향 +x) -> 현재 base_link 좌표."""
        ox, oy, oyaw = self.origin
        # 시나리오 좌표 -> odom 좌표
        gx = ox + math.cos(oyaw) * wx - math.sin(oyaw) * wy
        gy = oy + math.sin(oyaw) * wx + math.cos(oyaw) * wy
        # odom 좌표 -> base_link
        dx, dy = gx - self.x, gy - self.y
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        return (c * dx - s * dy, s * dx + c * dy)

    def _ahead(self, wx):
        """시나리오 x 위치까지 남은 전방 거리 (base_link x)."""
        return self._to_base(wx, 0.0)[0]

    # ------------------------------------------------------------------
    def _p(self, name):
        return self.get_parameter(name).value

    def _header(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'base_link'
        return h

    def _tick(self):
        scenario = self._p('scenario')
        if scenario != self.last_scenario:
            if self.own_odom:
                # 트랙(시나리오 좌표)은 +x 방향 그대로, 차만 start_yaw_deg 만큼 틀어져서 시작
                self.x = self.y = 0.0
                self.yaw = math.radians(float(self._p('start_yaw_deg')))
                self.origin = (0.0, 0.0, 0.0)
            else:
                self.origin = (self.x, self.y, self.yaw)
            self.last_scenario = scenario
            self.get_logger().info('시나리오 %s 시작 - 현재 위치를 기준점으로 사용' % scenario)

        lane = LaneError()
        lane.header = self._header()
        lane.valid = bool(self._p('lane_valid'))
        lane.lateral_offset_m = float(self._p('lateral_error'))
        lane.heading_error_rad = float(self._p('heading_error'))
        lane.confidence = float(self._p('lane_confidence'))
        lane.single_lane_estimated = bool(self._p('lane_single'))
        self.pub_lane.publish(lane)

        self.objects = []
        self.event = 'none'
        handler = getattr(self, '_s_' + scenario, None)
        if handler is not None:
            handler()

        arr = MissionObjectArray()
        arr.header = self._header()
        arr.objects = self.objects
        self.pub_objects.publish(arr)

        msg = String()
        msg.data = self.event
        self.pub_event.publish(msg)

    # ------------------------------------------------------------------
    # 발행 도우미
    # ------------------------------------------------------------------
    def _object(self, class_id, x_m, y_m=0.3, color=MissionObject.LIGHT_UNKNOWN):
        """앞쪽에 보이는 물체만 YOLO 결과로 추가 (뒤로 지나가면 안 보임)."""
        if x_m < 0.15:
            return
        o = MissionObject()
        o.class_id = class_id
        o.confidence = 0.9
        o.light_color = color
        o.position_valid = True
        o.x_m = float(x_m)
        o.y_m = float(y_m)
        o.position_source = MissionObject.POS_DEPTH
        self.objects.append(o)

    def _stopline(self, dist):
        """dist = 정지선까지 거리. YOLO 처럼 횡단보도 줄무늬 앞끝(정지선 + crosswalk_after_line)을 보낸다.
        보이는 거리 안일 때만 위치와 함께 보냄."""
        cw = dist + float(self._p('crosswalk_after_line'))
        if self._p('stopline_min') <= cw <= self._p('stopline_visible'):
            self._object(MissionObject.CROSSWALK, cw, 0.0)

    # ------------------------------------------------------------------
    # 시나리오
    # ------------------------------------------------------------------
    def _s_lane(self):
        pass

    def _s_traffic_light(self):
        line = self._ahead(self._p('distance'))
        color = LIGHT.get(str(self._p('light_color')), MissionObject.LIGHT_UNKNOWN)
        self._object(MissionObject.TRAFFIC_LIGHT, line + 0.3, 0.35, color)
        self._stopline(line)

    def _s_pedestrian(self):
        line = self._ahead(self._p('distance'))
        self._object(MissionObject.PEDESTRIAN_SIGN, line + 0.2, -0.35)
        self._stopline(line)

    def _s_bump(self):
        self._object(MissionObject.SLOW_SIGN, self._ahead(self._p('distance')),
                     float(self._p('sign_lateral')))
        if self.objects and not self._p('sign_position_valid'):
            self.objects[-1].position_valid = False

    def _s_side_signs(self):
        """옆 길 표지판 시험 (11/05): 빨간 신호등(distance+0.3)·주차 표지판(distance+2.0)을 side_lateral 옆에,
        정지 기준 횡단보도(distance)를 side_lateral x 0.6 옆에 둔다 (Gazebo 위쪽 길에서 본 안쪽 길과 비슷)."""
        d = self._ahead(self._p('distance'))
        lat = float(self._p('side_lateral'))
        self._object(MissionObject.TRAFFIC_LIGHT, d + 0.3, lat, MissionObject.LIGHT_RED)
        self._object(MissionObject.PARKING_SIGN, d + 2.0, lat)
        if self._p('stopline_min') <= d <= self._p('stopline_visible'):
            self._object(MissionObject.CROSSWALK, d, 0.6 * lat)

    def _s_slow_zone(self):
        """distance 앞에서 유도선 시작, zigzag_length 뒤 끝, 그 0.2m 뒤 보행자 정지선."""
        start_wx = self._p('distance')
        zig_start = self._ahead(start_wx)
        zig_end = self._ahead(start_wx + self._p('zigzag_length'))
        if zig_end > 0.3 and zig_start < 2.0:
            # 유도선의 가까운 끝까지 거리. 차가 유도선 위에 있으면 화면 아래 끝(0.3m)
            self._object(MissionObject.SLOW_GUIDE_LINE, max(0.3, zig_start), 0.0)
        line_wx = start_wx + self._p('zigzag_length') + 0.2
        line = self._ahead(line_wx)
        # 실제 트랙처럼 표지판은 유도선 입구에서부터 보인다
        self._object(MissionObject.PEDESTRIAN_SIGN, line + 0.2, -0.35)
        self._stopline(line)

    def _s_sign_a(self):
        self._object(MissionObject.SIGN_A, self._ahead(self._p('distance')), 0.4)

    def _s_sign_b(self):
        self._object(MissionObject.SIGN_B, self._ahead(self._p('distance')), 0.4)

    def _s_cone(self):
        """좌우 라바콘 3쌍을 앞쪽에 배치 (라이다 이벤트)."""
        self.event = 'cone'
        arr = ObstacleArray()
        arr.header = self._header()
        for i in range(3):
            wx = 0.6 + 0.45 * i
            for side in (1.0, -1.0):
                x, y = self._to_base(wx, side * 0.32)
                o = Obstacle()
                o.x = x
                o.y = y
                o.radius = 0.09
                o.point_count = 12
                o.is_dynamic = False
                arr.obstacles.append(o)
        self.pub_obstacles.publish(arr)

    # --- 2026 정적 장애물 / 주차 --------------------------------------------
    def static_limos(self):
        """3차선 LIMO 7대의 시나리오 좌표 [(x, y)] (가운데 차선 중앙 y = 0, 왼쪽 +)."""
        first = float(self._p('distance'))
        spacing = float(self._p('spacing'))
        return [(first + spacing * i, (1 - lane) * LANE_PITCH)
                for i, lane in enumerate(STATIC_LANES)]

    def parking_limos(self):
        """주차: 목표 칸 양옆(윗줄) 2대 + 목표 칸 바로 뒤 아랫줄 1대 [(x, y)]."""
        gap = float(self._p('gap'))
        bay_mid = -(BAY_ENTRY + 0.5 * BAY_DEPTH)
        top = bay_mid - float(self._p('limo_depth_offset'))
        bottom = bay_mid - BAY_DEPTH - 0.02
        return [(gap - BAY_PITCH, top), (gap + BAY_PITCH, top), (gap, bottom)]

    def _publish_limos(self, limos):
        """라이다 시야(앞 0.1~3 m, 좌우 1.2 m) 안의 LIMO 를 장애물로, 카메라 시야 안은 YOLO 로."""
        if not self._p('fake_limos'):
            return
        arr = ObstacleArray()
        arr.header = self._header()
        for wx, wy in limos:
            x, y = self._to_base(wx, wy)
            if 0.1 <= x <= 3.0 and abs(y) <= 1.2:
                o = Obstacle()
                o.x, o.y, o.radius, o.point_count = x, y, LIMO_R, 20
                arr.obstacles.append(o)
            if 0.3 <= x <= 2.5 and abs(math.atan2(y, x)) < math.radians(35):
                self._object(MissionObject.LIMO, x, y)
        self.pub_obstacles.publish(arr)

    def _s_static_3lane(self):
        limos = self.static_limos()
        self._publish_limos(limos)
        sign = str(self._p('sign')).upper()
        if sign in ('A', 'B'):
            sx = limos[-1][0] + ISLAND_AFTER + 0.15
            cls = MissionObject.SIGN_A if sign == 'A' else MissionObject.SIGN_B
            x, y = self._to_base(sx, 0.0)
            if 0.3 <= x <= 2.5 and abs(math.atan2(y, x)) < math.radians(35):
                self._object(cls, x, y)

    def _s_parking(self):
        """2026 주차: 주차 표지판 + 목표 칸 양옆 LIMO (벽 없음)."""
        self._object(MissionObject.PARKING_SIGN, self._ahead(0.6), -0.45)
        self._publish_limos(self.parking_limos())


def main(args=None):
    rclpy.init(args=args)
    node = FakePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
