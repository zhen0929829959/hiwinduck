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

        for i, box in enumerate(boxes):
            confidence = float(box.conf[0].item())
            bbox = box.xyxy[0].tolist()
            class_id = int(box.cls[0].item())

            x_min, y_min, x_max, y_max = map(int, bbox)

            x_min = max(0, min(x_min, w_img - 1))
            x_max = max(0, min(x_max, w_img - 1))
            y_min = max(0, min(y_min, h_img - 1))
            y_max = max(0, min(y_max, h_img - 1))

            # 預設：用 bbox 中心
            u = int((x_min + x_max) / 2)
            v = int((y_min + y_max) / 2)
            angle = 0.0
            center_source = 'box'

            # 優先：用 segmentation mask 中心
            if masks is not None and i < len(masks.data):
                mask = masks.data[i].cpu().numpy()
                mask = (mask * 255).astype('uint8')
                mask = cv2.resize(mask, (w_img, h_img))

                contours, _ = cv2.findContours(
                    mask,
                    cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE
                )

                if len(contours) > 0:
                    largest_contour = max(contours, key=cv2.contourArea)

                    if cv2.contourArea(largest_contour) > 50:
                        rect = cv2.minAreaRect(largest_contour)
                        (center_x, center_y), (rect_w, rect_h), angle = rect

                        u = int(center_x)
                        v = int(center_y)
                        center_source = 'seg'

            X_mm, Y_mm, Z_mm = self.get_xyz_from_depth(u, v)

            cv2.circle(
                annotated_image,
                (u, v),
                5,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                annotated_image,
                f'{center_source} center',
                (u + 10, v - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

            if X_mm is not None:
                cv2.putText(
                    annotated_image,
                    f'x={X_mm:.2f}, y={Y_mm:.2f}, z={Z_mm:.2f}',
                    (u + 10, v + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

            detections.append({
                'bbox': bbox,
                'confidence': confidence,
                'class_id': class_id,
                'angle': float(angle),
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