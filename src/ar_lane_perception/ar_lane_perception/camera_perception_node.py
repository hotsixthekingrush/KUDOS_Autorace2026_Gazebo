"""camera_perception_node: 카메라 영상 -> /perception/lane_error (ar_msgs/LaneError).

검출 로직은 lane_core.LaneCore에 있다 (limo_ros2_application/detect_line.py 기반).
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from ar_msgs.msg import LaneError

from ar_lane_perception.lane_core import DEFAULTS, LaneCore


class CameraPerceptionNode(Node):
    def __init__(self):
        super().__init__('camera_perception_node')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('publish_distance_y', False)   # 기존 limo_control용 /distance_y
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_num', 2)          # 0: 마스크, 그 외: 검출 표시 영상
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.core = LaneCore({k: self.get_parameter(k).value for k in DEFAULTS})
        self.add_on_set_parameters_callback(self._on_params)

        self.br = CvBridge()
        self.lane_pub = self.create_publisher(LaneError, '/perception/lane_error', 10)
        self.dis_pub = None
        if self.get_parameter('publish_distance_y').value:
            self.dis_pub = self.create_publisher(Int32, 'distance_y', qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, 'debug_image', 10)
        topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info('구독 %s -> /perception/lane_error (reference_distance_m=%.2f, lane_width_m=%.2f)'
                               % (topic, self.core.p['reference_distance_m'], self.core.p['lane_width_m']))

    def _on_params(self, params):
        for prm in params:
            if prm.name in DEFAULTS:
                self.core.p[prm.name] = prm.value
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        want_dbg = (self.get_parameter('publish_debug_image').value
                    and self.get_parameter('debug_image_num').value != 0)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        r = self.core.process(img, t, want_debug=want_dbg)

        out = LaneError()
        out.header.stamp = msg.header.stamp          # 영상 촬영 시각
        out.header.frame_id = 'base_link'
        out.valid = bool(r['valid'])
        # valid=false여도 계산값은 그대로 실어 둔다 (디버그용, 판단팀은 무시)
        out.lateral_offset_m = float(r['lateral'])
        out.heading_error_rad = float(r['heading'])
        out.curvature = 0.0                          # 아직 추정하지 않음
        out.confidence = float(r['confidence'])
        out.single_lane_estimated = bool(r['single'])
        self.lane_pub.publish(out)
        if r['guard']:
            self.get_logger().warn('lane guard: %s (width_err=%s) -> valid=false'
                                   % (r['guard'], None if r['width_err'] is None
                                      else '%.2f' % r['width_err']),
                                   throttle_duration_sec=1.0)

        if self.dis_pub is not None:
            self.dis_pub.publish(Int32(data=r['gap']))
        if self.get_parameter('publish_debug_image').value:
            if self.get_parameter('debug_image_num').value == 0:
                g = self.core.p
                hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
                mask = cv2.inRange(hls, np.array([g['lane_h_l'], g['lane_l_l'], g['lane_s_l']]),
                                   np.array([g['lane_h_h'], g['lane_l_h'], g['lane_s_h']]))
                self.debug_pub.publish(self.br.cv2_to_imgmsg(mask, 'mono8'))
            elif r['debug'] is not None:
                self.debug_pub.publish(self.br.cv2_to_imgmsg(r['debug'], 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
