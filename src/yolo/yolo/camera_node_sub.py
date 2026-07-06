#!/usr/bin/env python3

import json

import cv2
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_sub')

        self.bridge = CvBridge()
        self.depth_image = None

        # 中心點平滑用
        self.smooth_centers = {}
        self.alpha = 0.25

        if torch.cuda.is_available():
            self.device = 0
            self.get_logger().info(
                f'Using GPU: {torch.cuda.get_device_name(0)}'
            )
        else:
            self.device = 'cpu'
            self.get_logger().warn('CUDA not available, using CPU')

        self.model = YOLO('src/yolo/best3.pt')

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            'yolo_detections',
            10
        )

        self.get_logger().info('YOLO node started')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(f'Depth cv_bridge error: {e}')

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

    def get_xyz_from_depth(self, u, v):
        if self.depth_image is None:
            return None, None, None

        dh, dw = self.depth_image.shape[:2]

        if not (0 <= u < dw and 0 <= v < dh):
            return None, None, None

        depth_raw = self.depth_image[v, u]

        if depth_raw == 0:
            return None, None, None

        if self.depth_image.dtype == 'uint16':
            depth_m = float(depth_raw) * 0.001
        else:
            depth_m = float(depth_raw)

        fx = 909.77309
        fy = 909.173606
        cx = 645.06746
        cy = 366.54143

        X = (u - cx) * depth_m / fx
        Y = (v - cy) * depth_m / fy
        Z = depth_m

        return X * 1000.0, Y * 1000.0, Z * 1000.0

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

        # 先依 class 排，再依照 x 座標排
        # 這樣同一個 class 左邊永遠比較前面，右邊永遠比較後面
        items.sort(key=lambda item: (item['class_id'], item['cx_box']))

        # 幫每個 class 重新編 stable_id
        class_count = {}

        for item in items:
            class_id = item['class_id']

            if class_id not in class_count:
                class_count[class_id] = 0

            item['stable_id'] = class_count[class_id]
            class_count[class_id] += 1

        return items

    def rgb_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'RGB cv_bridge error: {e}')
            return

        results = self.model(
            image,
            conf=0.6,
            device=self.device,
            verbose=False
        )

        detections = []
        annotated_image = results[0].plot()

        boxes = results[0].boxes
        masks = results[0].masks

        h_img, w_img = image.shape[:2]

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

            # 預設：box 中心
            u = int((x_min + x_max) / 2)
            v = int((y_min + y_max) / 2)
            angle = 0.0
            area = 0.0
            center_source = 'box'

            # 優先：segmentation mask + 二值化 + morphology
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

            # 重點：不要用 original_i 平滑
            # 用 class_name + stable_id
            # 例如：RJ45_0、RJ45_1
            track_key = f'{class_name}_{stable_id}'
            u, v = self.smooth_center(track_key, u, v)

            X_mm, Y_mm, Z_mm = self.get_xyz_from_depth(u, v)

            cv2.circle(
                annotated_image,
                (u, v),
                6,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                annotated_image,
                f'{class_name}_{stable_id} ',
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

            if X_mm is not None:
                cv2.putText(
                    annotated_image,
                    f'x={X_mm:.1f}, y={Y_mm:.1f}, z={Z_mm:.1f}',
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
                'pixel_center': [u, v],
                'center_source': center_source,
                'camera_xyz': [X_mm, Y_mm, Z_mm]
            })

        msg_out = String()
        msg_out.data = json.dumps(detections)
        self.publisher.publish(msg_out)

        self.get_logger().info(
            f'Publishing detections: {len(detections)}'
        )

        cv2.imshow('Detection Results', annotated_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()