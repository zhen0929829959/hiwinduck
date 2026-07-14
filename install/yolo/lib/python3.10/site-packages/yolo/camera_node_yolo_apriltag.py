#!/usr/bin/env python3

import json
import cv2
import rclpy
import torch
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from pupil_apriltags import Detector


class YoloAprilTagPlaneNode(Node):
    def __init__(self):
        super().__init__('yolo_apriltag_plane_node')

        self.bridge = CvBridge()

        # ===== Camera intrinsics for 1920x1080 =====
        self.fx = 1362.7784437304256
        self.fy = 1361.8357978878923
        self.cx = 962.1412542551096
        self.cy = 538.5980610605569

        self.camera_matrix = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.dist_coeffs = np.array([
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ], dtype=np.float64)

        # AprilTag size, meter
        self.tag_size = 0.0475

        # 如果 RJ45 平面跟 AprilTag 平面有高度差，可以改這裡
        # 單位：meter
        # 0.0 代表 YOLO 目標點就在 AprilTag 所在平面上
        self.plane_offset_m = 0.0

        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # ===== YOLO device =====
        if torch.cuda.is_available():
            self.device = 0
            self.get_logger().info(
                f'Using GPU: {torch.cuda.get_device_name(0)}'
            )
        else:
            self.device = 'cpu'
            self.get_logger().warn('CUDA not available, using CPU')

        self.model = YOLO('src/yolo/best3.pt')

        # Center smoothing
        self.smooth_centers = {}
        self.alpha = 0.25

        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            String,
            '/yolo_apriltag/detections',
            10
        )

        self.get_logger().info(
            'YOLO + AprilTag plane node started, no depth needed'
        )

    def clean_mask(self, mask, w_img, h_img):
        mask = (mask * 255).astype('uint8')
        mask = cv2.resize(mask, (w_img, h_img))

        _, binary = cv2.threshold(
            mask,
            127,
            255,
            cv2.THRESH_BINARY
        )

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (5, 5)
        )

        binary = cv2.morphologyEx(
            binary,
            cv2.MORPH_OPEN,
            kernel,
            iterations=1
        )

        binary = cv2.morphologyEx(
            binary,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2
        )

        return binary

    def get_center_from_mask(self, binary_mask):
        contours, _ = cv2.findContours(
            binary_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            return None, None, None, 0.0

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area < 50:
            return None, None, None, area

        M = cv2.moments(largest_contour)

        if M['m00'] == 0:
            return None, None, None, area

        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])

        rect = cv2.minAreaRect(largest_contour)
        angle = rect[2]

        return u, v, angle, area

    def smooth_center(self, track_key, u, v):
        if track_key in self.smooth_centers:
            old_u, old_v = self.smooth_centers[track_key]
            u = int(self.alpha * u + (1.0 - self.alpha) * old_u)
            v = int(self.alpha * v + (1.0 - self.alpha) * old_v)

        self.smooth_centers[track_key] = (u, v)

        return u, v

    def make_sorted_items(self, boxes):
        items = []

        for i, box in enumerate(boxes):
            confidence = float(box.conf[0].item())
            bbox = box.xyxy[0].tolist()
            class_id = int(box.cls[0].item())

            x_min, y_min, x_max, y_max = map(int, bbox)
            cx_box = int((x_min + x_max) / 2)
            cy_box = int((y_min + y_max) / 2)

            items.append({
                'original_i': i,
                'box': box,
                'bbox': bbox,
                'class_id': class_id,
                'confidence': confidence,
                'cx_box': cx_box,
                'cy_box': cy_box
            })

        items.sort(key=lambda item: (item['class_id'], item['cx_box']))

        class_count = {}

        for item in items:
            class_id = item['class_id']

            if class_id not in class_count:
                class_count[class_id] = 0

            item['stable_id'] = class_count[class_id]
            class_count[class_id] += 1

        return items

    def detect_best_tag(self, gray):
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=self.tag_size
        )

        if len(tags) == 0:
            return None

        # 如果看到多個 tag，選面積最大的
        best_tag = max(
            tags,
            key=lambda tag: cv2.contourArea(tag.corners.astype(np.float32))
        )

        return best_tag

    def pixel_to_camera_on_tag_plane(self, u, v, tag):
        R_tag = tag.pose_R
        t_tag = tag.pose_t.reshape(3)

        # AprilTag 平面的法向量，tag Z 軸在 camera frame 中的方向
        normal = R_tag[:, 2]

        # 如果 RJ45 平面跟 tag 平面有一點高度差，可沿 normal 補償
        point_on_plane = t_tag + normal * self.plane_offset_m

        pixel = np.array([[[float(u), float(v)]]], dtype=np.float64)

        undistorted = cv2.undistortPoints(
            pixel,
            self.camera_matrix,
            self.dist_coeffs
        )

        x_norm = undistorted[0, 0, 0]
        y_norm = undistorted[0, 0, 1]

        ray = np.array([x_norm, y_norm, 1.0], dtype=np.float64)

        denom = float(np.dot(normal, ray))

        if abs(denom) < 1e-9:
            return None

        scale = float(np.dot(normal, point_on_plane) / denom)

        if scale <= 0:
            return None

        point_camera_m = scale * ray

        return point_camera_m

    def draw_apriltag(self, frame, tag):
        corners = tag.corners.astype(int)

        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        center_x, center_y = tag.center
        u = int(center_x)
        v = int(center_y)

        cv2.circle(frame, (u, v), 5, (0, 255, 0), -1)

        cv2.putText(
            frame,
            f'Tag ID:{tag.tag_id}',
            (u + 10, v - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'Image cv_bridge error: {e}')
            return

        h_img, w_img = image.shape[:2]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        tag = self.detect_best_tag(gray)

        results = self.model(
            image,
            conf=0.6,
            device=self.device,
            verbose=False
        )

        annotated_image = results[0].plot()
        detections = []

        if tag is not None:
            self.draw_apriltag(annotated_image, tag)

        boxes = results[0].boxes
        masks = results[0].masks

        sorted_items = self.make_sorted_items(boxes)

        for item in sorted_items:
            i = item['original_i']
            bbox = item['bbox']
            confidence = item['confidence']
            class_id = item['class_id']
            stable_id = item['stable_id']

            class_name = self.model.names[class_id]

            x_min, y_min, x_max, y_max = map(int, bbox)

            x_min = max(0, min(x_min, w_img - 1))
            x_max = max(0, min(x_max, w_img - 1))
            y_min = max(0, min(y_min, h_img - 1))
            y_max = max(0, min(y_max, h_img - 1))

            u = int((x_min + x_max) / 2)
            v = int((y_min + y_max) / 2)

            angle = 0.0
            area = 0.0
            center_source = 'box'

            if masks is not None and i < len(masks.data):
                raw_mask = masks.data[i].cpu().numpy()
                binary_mask = self.clean_mask(raw_mask, w_img, h_img)

                mask_u, mask_v, mask_angle, mask_area = self.get_center_from_mask(
                    binary_mask
                )

                if mask_u is not None:
                    u = mask_u
                    v = mask_v
                    angle = mask_angle
                    area = mask_area
                    center_source = 'seg_binary'

            track_key = f'{class_name}_{stable_id}'
            u, v = self.smooth_center(track_key, u, v)

            camera_xyz_m = None
            camera_xyz_mm = [None, None, None]

            if tag is not None:
                point_camera_m = self.pixel_to_camera_on_tag_plane(
                    u,
                    v,
                    tag
                )

                if point_camera_m is not None:
                    camera_xyz_m = [
                        float(point_camera_m[0]),
                        float(point_camera_m[1]),
                        float(point_camera_m[2])
                    ]

                    camera_xyz_mm = [
                        float(point_camera_m[0] * 1000.0),
                        float(point_camera_m[1] * 1000.0),
                        float(point_camera_m[2] * 1000.0)
                    ]

            cv2.circle(
                annotated_image,
                (u, v),
                6,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                annotated_image,
                f'{class_name}_{stable_id}',
                (u + 10, v - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

            cv2.putText(
                annotated_image,
                f'px=({u},{v})',
                (u + 10, v + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

            if camera_xyz_m is not None:
                cv2.putText(
                    annotated_image,
                    f'x={camera_xyz_mm[0]:.1f}, y={camera_xyz_mm[1]:.1f}, z={camera_xyz_mm[2]:.1f} mm',
                    (u + 10, v + 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

            detections.append({
                'class_name': class_name,
                'stable_id': stable_id,
                'track_key': track_key,
                'bbox': bbox,
                'confidence': confidence,
                'class_id': class_id,
                'angle': float(angle),
                'mask_area': float(area),
                'pixel_center': [int(u), int(v)],
                'center_source': center_source,

                # 給你原本手臂程式比較好接
                'camera_xyz': camera_xyz_mm,

                # 額外保留公尺版本
                'camera_xyz_m': camera_xyz_m,

                'xyz_source': 'apriltag_plane' if camera_xyz_m is not None else 'none',
                'apriltag_detected': tag is not None,
                'apriltag_id': int(tag.tag_id) if tag is not None else None
            })

        msg_out = String()
        msg_out.data = json.dumps(detections)
        self.detection_pub.publish(msg_out)

        self.get_logger().info(
            f'Publishing YOLO + AprilTag detections: {len(detections)}'
        )

        cv2.imshow('YOLO + AprilTag Plane', annotated_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = YoloAprilTagPlaneNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()