#!/usr/bin/env python3

import json
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

image_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


class CombinedVisionViewNode(Node):
    def __init__(self):
        super().__init__('combined_vision_view')
        self.bridge = CvBridge()
        self.declare_parameter('left_apriltag_image_topic', '/camera_left/apriltag/debug_image')
        self.declare_parameter('left_yolo_detections_topic', '/camera_left/yolo/detections')
        self.declare_parameter('right_yolo_image_topic', '/camera_right/yolo/debug_image')
        self.declare_parameter('window_name', 'Stereo Vision')
        self.declare_parameter('display_width', 960)
        self.left_image = None
        self.right_image = None
        self.left_detections = []
        self.window_name = self.get_parameter('window_name').value
        self.display_width = int(self.get_parameter('display_width').value)
        self.create_subscription(Image, self.get_parameter('left_apriltag_image_topic').value, self.left_image_callback, image_qos)
        self.create_subscription(String, self.get_parameter('left_yolo_detections_topic').value, self.left_detections_callback, 10)
        self.create_subscription(Image, self.get_parameter('right_yolo_image_topic').value, self.right_image_callback, image_qos)
        self.timer = self.create_timer(1.0 / 20.0, self.show_combined_view)
        self.get_logger().info('Combined view started: left=AprilTag+YOLO, right=YOLO')

    def left_image_callback(self, msg):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Left image conversion failed: {exc}')

    def right_image_callback(self, msg):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Right image conversion failed: {exc}')

    def left_detections_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.left_detections = data.get('detections', []) if isinstance(data, dict) else data
            if not isinstance(self.left_detections, list):
                self.left_detections = []
        except (json.JSONDecodeError, TypeError):
            self.left_detections = []

    def draw_left_yolo(self, image):
        for detection in self.left_detections:
            bbox = detection.get('bbox')
            center = detection.get('pixel_center')
            track_key = str(detection.get('track_key', 'YOLO'))
            confidence = detection.get('confidence')
            if isinstance(bbox, list) and len(bbox) == 4:
                x1, y1, x2, y2 = [int(round(value)) for value in bbox]
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 165, 255), 3)
                label = f'{track_key} {float(confidence):.2f}' if confidence is not None else track_key
                cv2.putText(image, label, (x1, max(25, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            if isinstance(center, list) and len(center) == 2:
                u, v = int(round(center[0])), int(round(center[1]))
                cv2.circle(image, (u, v), 6, (0, 0, 255), -1)
                cv2.putText(image, f'({u},{v})', (u + 10, v + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return image

    def make_panel(self, image, title):
        height, width = image.shape[:2]
        target_width = max(320, self.display_width // 2)
        target_height = max(1, int(height * target_width / width))
        panel = cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)
        cv2.rectangle(panel, (0, 0), (target_width, 38), (0, 0, 0), -1)
        cv2.putText(panel, title, (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        return panel

    def show_combined_view(self):
        if self.left_image is None or self.right_image is None:
            return
        left = self.draw_left_yolo(self.left_image.copy())
        right = self.right_image.copy()
        left_panel = self.make_panel(left, 'LEFT: YOLO + AprilTag')
        right_panel = self.make_panel(right, 'RIGHT: YOLO')
        common_height = min(left_panel.shape[0], right_panel.shape[0])
        left_panel = cv2.resize(left_panel, (left_panel.shape[1], common_height))
        right_panel = cv2.resize(right_panel, (right_panel.shape[1], common_height))
        combined = np.hstack((left_panel, right_panel))
        cv2.imshow(self.window_name, combined)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CombinedVisionViewNode()
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
