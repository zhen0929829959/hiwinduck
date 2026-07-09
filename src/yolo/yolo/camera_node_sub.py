#!/usr/bin/env python3

import json

import cv2
import rclpy
import torch

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()

        # ====================================================
        # 相機內參
        # ====================================================

        # self.fx = 1362.7784437304256
        # self.fy = 1361.8357978878923
        # self.cx = 962.1412542551096
        # self.cy = 538.5980610605569
        self.fx = 923.5084228515625
        self.fy = 923.7492065429688
        self.cx = 641.0979614257812
        self.cy = 364.9132385253906

        self.yolo_plane_offset_m = -0.115

        # 最新 AprilTag Z 深度，單位：公尺
        self.latest_tag_z_m = None

        # ====================================================
        # YOLO 裝置
        # ====================================================

        if torch.cuda.is_available():
            self.device = 0

            self.get_logger().info(
                f'Using GPU: {torch.cuda.get_device_name(0)}'
            )
        else:
            self.device = 'cpu'

            self.get_logger().warn(
                'CUDA not available, using CPU'
            )

        self.model = YOLO(
            'src/yolo/best3.pt'
        )

        # ====================================================
        # 中心點平滑
        # ====================================================

        self.smooth_centers = {}
        self.alpha = 0.25

        # ====================================================
        # ROS subscribers
        # ====================================================

        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.apriltag_pose_sub = self.create_subscription(
            PoseStamped,
            '/apriltag/pose_camera',
            self.apriltag_pose_callback,
            10
        )

        # ====================================================
        # ROS publisher
        # ====================================================

        self.detection_pub = self.create_publisher(
            String,
            '/yolo/detections',
            10
        )

        self.get_logger().info(
            'YOLO node started, using AprilTag Z'
        )

    # ========================================================
    # 接收 AprilTag Pose
    # ========================================================

    def apriltag_pose_callback(self, msg):
        tag_z = float(
            msg.pose.position.z
        )

        if tag_z <= 0.0:
            self.get_logger().warn(
                f'Invalid AprilTag Z: {tag_z}'
            )
            return

        self.latest_tag_z_m = tag_z

        self.get_logger().info(
            f'Updated AprilTag Z: '
            f'{self.latest_tag_z_m:.4f} m'
        )

    # ========================================================
    # 清理 segmentation mask
    # ========================================================

    def clean_mask(self, mask, w_img, h_img):
        mask = (
            mask * 255
        ).astype('uint8')

        mask = cv2.resize(
            mask,
            (w_img, h_img)
        )

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

    # ========================================================
    # 從 mask 找中心
    # ========================================================

    def get_center_from_mask(self, binary_mask):
        contours, _ = cv2.findContours(
            binary_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            return None, None, None, 0.0

        largest_contour = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(
            largest_contour
        )

        if area < 50:
            return None, None, None, area

        moments = cv2.moments(
            largest_contour
        )

        if moments['m00'] == 0:
            return None, None, None, area

        u = int(
            moments['m10']
            / moments['m00']
        )

        v = int(
            moments['m01']
            / moments['m00']
        )

        rect = cv2.minAreaRect(
            largest_contour
        )

        angle = rect[2]

        return u, v, angle, area

    # ========================================================
    # 中心點平滑
    # ========================================================

    def smooth_center(self, track_key, u, v):
        if track_key in self.smooth_centers:
            old_u, old_v = (
                self.smooth_centers[track_key]
            )

            u = int(
                self.alpha * u
                + (1.0 - self.alpha) * old_u
            )

            v = int(
                self.alpha * v
                + (1.0 - self.alpha) * old_v
            )

        self.smooth_centers[track_key] = (
            u,
            v
        )

        return u, v

    # ========================================================
    # 像素 + AprilTag Z 轉相機座標
    # ========================================================

    def pixel_to_camera_xyz(self, u, v):
        # if self.latest_tag_z_m is None:
        #     return None

        # z_m = (
        #     self.latest_tag_z_m
        #     + self.yolo_plane_offset_m
        # )

        z_m = 0.17

        x_m = (
            (float(u) - self.cx)
            * z_m
            / self.fx
        )

        y_m = (
            (float(v) - self.cy)
            * z_m
            / self.fy 
        )

        return [
            float(x_m),
            float(y_m),
            float(z_m)
        ]

    # ========================================================
    # 排序同類物件
    # ========================================================

    def make_sorted_items(self, boxes):
        items = []

        for original_i, box in enumerate(boxes):
            confidence = float(
                box.conf[0].item()
            )

            bbox = (
                box.xyxy[0]
                .tolist()
            )

            class_id = int(
                box.cls[0].item()
            )

            x_min, y_min, x_max, y_max = map(
                int,
                bbox
            )

            cx_box = int(
                (x_min + x_max) / 2
            )

            cy_box = int(
                (y_min + y_max) / 2
            )

            items.append({
                'original_i': original_i,
                'bbox': bbox,
                'class_id': class_id,
                'confidence': confidence,
                'cx_box': cx_box,
                'cy_box': cy_box
            })

        items.sort(
            key=lambda item: (
                item['class_id'],
                item['cx_box']
            )
        )

        class_count = {}

        for item in items:
            class_id = item['class_id']

            if class_id not in class_count:
                class_count[class_id] = 0

            item['stable_id'] = (
                class_count[class_id]
            )

            class_count[class_id] += 1

        return items

    # ========================================================
    # RGB callback
    # ========================================================

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

        except Exception as exc:
            self.get_logger().error(
                f'Image cv_bridge error: {exc}'
            )
            return

        h_img, w_img = image.shape[:2]

        results = self.model(
            image,
            conf=0.6,
            device=self.device,
            verbose=False
        )

        result = results[0]

        annotated_image = result.plot()

        boxes = result.boxes
        masks = result.masks

        detections = []

        sorted_items = self.make_sorted_items(
            boxes
        )

        for item in sorted_items:
            original_i = item['original_i']
            bbox = item['bbox']
            confidence = item['confidence']
            class_id = item['class_id']
            stable_id = item['stable_id']

            class_name = self.model.names[
                class_id
            ]

            x_min, y_min, x_max, y_max = map(
                int,
                bbox
            )

            x_min = max(
                0,
                min(x_min, w_img - 1)
            )

            x_max = max(
                0,
                min(x_max, w_img - 1)
            )

            y_min = max(
                0,
                min(y_min, h_img - 1)
            )

            y_max = max(
                0,
                min(y_max, h_img - 1)
            )

            # 預設使用 bounding box 中心
            u = int(
                (x_min + x_max) / 2
            )

            v = int(
                (y_min + y_max) / 2
            )

            angle = 0.0
            area = 0.0
            center_source = 'box'

            # 有 mask 時優先使用 mask 重心
            if (
                masks is not None
                and original_i < len(masks.data)
            ):
                raw_mask = (
                    masks.data[original_i]
                    .cpu()
                    .numpy()
                )

                binary_mask = self.clean_mask(
                    raw_mask,
                    w_img,
                    h_img
                )

                (
                    mask_u,
                    mask_v,
                    mask_angle,
                    mask_area
                ) = self.get_center_from_mask(
                    binary_mask
                )

                if mask_u is not None:
                    u = mask_u
                    v = mask_v
                    angle = mask_angle
                    area = mask_area
                    center_source = 'seg_binary'

            track_key = (
                f'{class_name}_{stable_id}'
            )

            u, v = self.smooth_center(
                track_key,
                u,
                v
            )

            # 使用 AprilTag Z 計算 YOLO 點相機座標
            camera_xyz_m = (
                self.pixel_to_camera_xyz(
                    u,
                    v
                )
            )

            if camera_xyz_m is not None:
                camera_xyz = [
                    camera_xyz_m[0] * 1000.0,
                    camera_xyz_m[1] * 1000.0,
                    camera_xyz_m[2] * 1000.0
                ]
            else:
                camera_xyz = [
                    None,
                    None,
                    None
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
                track_key,
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
                    (
                        f'x={camera_xyz[0]:.1f}, '
                        f'y={camera_xyz[1]:.1f}, '
                        f'z={camera_xyz[2]:.1f} mm'
                    ),
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
                'bbox': [
                    float(value)
                    for value in bbox
                ],
                'confidence': confidence,
                'class_id': class_id,
                'angle': float(angle),
                'mask_area': float(area),
                'pixel_center': [
                    int(u),
                    int(v)
                ],
                'center_source': center_source,

                # 單位：毫米
                'camera_xyz': camera_xyz,

                # 單位：公尺
                'camera_xyz_m': camera_xyz_m,

                'xyz_source': (
                    'apriltag_z'
                    if camera_xyz_m is not None
                    else 'none'
                )
            })

        msg_out = String()

        msg_out.data = json.dumps(
            detections
        )

        self.detection_pub.publish(
            msg_out
        )

        self.get_logger().info(
            f'Publishing YOLO detections: '
            f'{len(detections)}'
        )

        cv2.imshow(
            'YOLO Detection',
            annotated_image
        )

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()