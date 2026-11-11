import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool

class LimoControl(Node):
    def __init__(self):
        super().__init__('limo_control')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.e_stop_subscription = self.create_subscription(
            Bool, 'e_stop', self.e_stop_callback, 10)
        self.distance_subscription = self.create_subscription(
            Int32, 'distance_y', self.distance_callback, rclpy.qos.qos_profile_sensor_data)

        self.e_stop_flag = True
        self.gap = 0
        self.d_gap = 0.0          # gap의 변화율 (px/s), 흔들림을 잡아주는 D항
        self.prev_gap = None
        self.prev_t = None

        self.declare_parameter('default_speed', 0.2)
        self.declare_parameter('p_gain', 0.01)
        self.declare_parameter('d_gain', 0.005)          # 흔들림 억제 (0이면 이전과 동일)
        self.declare_parameter('max_ang', 1.0)          # 회전 속도 상한 (rad/s)
        self.declare_parameter('slow_gap', 150.0)       # gap이 이 값일 때 속도가 최저로 감소
        self.declare_parameter('min_speed_ratio', 0.4)  # 최저 속도 비율

        self.default_speed = self.get_parameter('default_speed')
        self.p_gain = self.get_parameter('p_gain')

    def e_stop_callback(self, msg):
        self.e_stop_flag = msg.data

    def distance_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_t is not None:
            dt = now - self.prev_t
            if 0.02 < dt < 1.0:
                raw = (msg.data - self.prev_gap) / dt
                self.d_gap = 0.7 * self.d_gap + 0.3 * raw   # 잡음 줄이기
        self.prev_gap = msg.data
        self.prev_t = now
        self.gap = msg.data

    def timer_callback(self):
        d_gain = self.get_parameter('d_gain').value
        max_ang = self.get_parameter('max_ang').value
        slow_gap = self.get_parameter('slow_gap').value
        min_ratio = self.get_parameter('min_speed_ratio').value

        msg = Twist()
        ratio = max(min_ratio, 1.0 - abs(self.gap) / slow_gap)
        msg.linear.x = self.default_speed.value * ratio
        ang = self.gap * self.p_gain.value + self.d_gap * d_gain
        msg.angular.z = max(-max_ang, min(max_ang, ang))

        if self.e_stop_flag:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    limo_control = LimoControl()

    rclpy.spin(limo_control)

    limo_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
