#!/usr/bin/env python3

import json
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        # ====================================================
        # ROS 參數
        # ====================================================
        self.declare_parameter('left_detection_topic', '/camera_left/yolo/detections')
        self.declare_parameter('right_detection_topic', '/camera_right/yolo/detections')
        self.declare_parameter('depth_topic', '/stereo/depth')
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('max_time_difference_sec', 0.08)
        self.declare_parameter('max_reprojection_error_px', 25.0)
        self.declare_parameter('left_camera_matrix', [0.0] * 9)
        self.declare_parameter('left_dist_coeffs', [0.0] * 5)
        self.declare_parameter('right_camera_matrix', [0.0] * 9)
        self.declare_parameter('right_dist_coeffs', [0.0] * 5)
        self.declare_parameter('stereo_rotation', [0.0] * 9)
        self.declare_parameter('stereo_translation', [0.0] * 3)

        # ====================================================
        # 讀取參數
        # ====================================================
        self.left_detection_topic = self.get_parameter('left_detection_topic').value
        self.right_detection_topic = self.get_parameter('right_detection_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.max_time_difference_sec = float(
            self.get_parameter('max_time_difference_sec').value
        )
        self.max_reprojection_error_px = float(
            self.get_parameter('max_reprojection_error_px').value
        )
        self.k_left = np.array(
            self.get_parameter('left_camera_matrix').value, dtype=np.float64
        ).reshape(3, 3)
        self.d_left = np.array(
            self.get_parameter('left_dist_coeffs').value, dtype=np.float64
        ).reshape(-1, 1)
        self.k_right = np.array(
            self.get_parameter('right_camera_matrix').value, dtype=np.float64
        ).reshape(3, 3)
        self.d_right = np.array(
            self.get_parameter('right_dist_coeffs').value, dtype=np.float64
        ).reshape(-1, 1)
        self.stereo_r = np.array(
            self.get_parameter('stereo_rotation').value, dtype=np.float64
        ).reshape(3, 3)
        self.stereo_t = np.array(
            self.get_parameter('stereo_translation').value, dtype=np.float64
        ).reshape(3, 1)

        self._validate_parameters()

        # ====================================================
        # 投影矩陣
        #
        # X_right = R * X_left + T
        #
        # 使用 undistortPoints 後的歸一化座標，
        # 所以這裡不乘 K。
        # ====================================================
        self.projection_left = np.hstack(
            (np.eye(3, dtype=np.float64), np.zeros((3, 1), dtype=np.float64))
        )
        self.projection_right = np.hstack((self.stereo_r, self.stereo_t))

        # ====================================================
        # 最新左右偵測
        # ====================================================
        self.latest_left: list[dict[str, Any]] = []
        self.latest_right: list[dict[str, Any]] = []

        # ====================================================
        # ROS
        # ====================================================
        self.left_sub = self.create_subscription(
            String, self.left_detection_topic, self.left_callback, 10
        )
        self.right_sub = self.create_subscription(
            String, self.right_detection_topic, self.right_callback, 10
        )
        self.depth_pub = self.create_publisher(String, self.depth_topic, 10)

        baseline_m = float(np.linalg.norm(self.stereo_t))
        self.get_logger().info(
            f'Stereo triangulation started | all YOLO targets | baseline={baseline_m:.6f} m'
        )

    # ========================================================
    # 檢查參數
    # ========================================================
    def _validate_parameters(self):
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError(f'Invalid image size: {self.image_width}x{self.image_height}')

        if self.k_left[0, 0] <= 0.0:
            raise ValueError('Invalid left camera matrix')

        if self.k_right[0, 0] <= 0.0:
            raise ValueError('Invalid right camera matrix')

        baseline_m = float(np.linalg.norm(self.stereo_t))
        if baseline_m <= 1e-6:
            raise ValueError('Invalid stereo translation: baseline is zero')

        rotation_check = self.stereo_r.T @ self.stereo_r
        if not np.allclose(rotation_check, np.eye(3), atol=1e-3):
            self.get_logger().warn(
                'Stereo rotation matrix is not perfectly orthonormal'
            )

        determinant = float(np.linalg.det(self.stereo_r))
        if abs(determinant - 1.0) > 1e-3:
            self.get_logger().warn(
                f'Stereo rotation determinant is {determinant:.6f}, expected 1.0'
            )

    # ========================================================
    # 收左右 YOLO detections
    # ========================================================
    def left_callback(self, msg: String):
        detections = self.extract_detections(msg.data, expected_camera_id='left')
        if detections is None:
            return
        self.latest_left = detections
        self.try_triangulate()

    def right_callback(self, msg: String):
        detections = self.extract_detections(msg.data, expected_camera_id='right')
        if detections is None:
            return
        self.latest_right = detections
        self.try_triangulate()

    # ========================================================
    # 載入 detections JSON
    # ========================================================
    def load_detections(self, json_data: str) -> Optional[list[dict[str, Any]]]:
        try:
            detections = json.loads(json_data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid detection JSON: {exc}')
            return None

        if not isinstance(detections, list):
            self.get_logger().warn('Detection JSON is not a list')
            return None

        return [detection for detection in detections if isinstance(detection, dict)]

    # ========================================================
    # 檢查 detection 格式
    # ========================================================
    def valid_detection(
        self, detection: dict[str, Any], expected_camera_id: str
    ) -> bool:
        camera_id = detection.get('camera_id')
        if camera_id is not None and camera_id != expected_camera_id:
            return False

        pixel_center = detection.get('pixel_center')
        if not isinstance(pixel_center, list) or len(pixel_center) != 2:
            return False

        try:
            u = float(pixel_center[0])
            v = float(pixel_center[1])
        except (TypeError, ValueError):
            return False

        if not (np.isfinite(u) and np.isfinite(v)):
            return False

        if 'stamp_sec' not in detection or 'stamp_nanosec' not in detection:
            return False

        return True

    # ========================================================
    # 保留所有有效 detection；相同 track_key 只留 confidence 最高者
    # ========================================================
    def extract_detections(
        self, json_data: str, expected_camera_id: str
    ) -> Optional[list[dict[str, Any]]]:
        detections = self.load_detections(json_data)
        if detections is None:
            return None

        best_by_track_key: dict[str, dict[str, Any]] = {}
        for detection in detections:
            if not self.valid_detection(detection, expected_camera_id):
                continue
            track_key = detection.get('track_key')
            if not isinstance(track_key, str) or not track_key:
                continue
            previous = best_by_track_key.get(track_key)
            if previous is None or float(detection.get('confidence', 0.0)) > float(previous.get('confidence', 0.0)):
                best_by_track_key[track_key] = detection

        return list(best_by_track_key.values())

    # ========================================================
    # Detection timestamp
    # ========================================================
    @staticmethod
    def detection_time_sec(detection: dict[str, Any]) -> float:
        sec = float(detection['stamp_sec'])
        nanosec = float(detection['stamp_nanosec'])
        return sec + nanosec * 1e-9

    # ========================================================
    # 原始像素 -> 去畸變歸一化座標
    # ========================================================
    @staticmethod
    def normalize_pixel(
        pixel_center: list[float],
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> np.ndarray:
        point = np.array(
            [[[float(pixel_center[0]), float(pixel_center[1])]]], dtype=np.float64
        )
        normalized = cv2.undistortPoints(
            src=point, cameraMatrix=camera_matrix, distCoeffs=dist_coeffs
        )
        return normalized.reshape(2)

    # ========================================================
    # 3D 相機座標 -> 原始像素
    # ========================================================
    @staticmethod
    def project_camera_point(
        point_camera: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> np.ndarray:
        image_point, _ = cv2.projectPoints(
            objectPoints=np.asarray(point_camera, dtype=np.float64).reshape(1, 1, 3),
            rvec=np.zeros((3, 1), dtype=np.float64),
            tvec=np.zeros((3, 1), dtype=np.float64),
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
        )
        return image_point.reshape(2)

    # ========================================================
    # 三角測量
    # ========================================================
    def triangulate(
        self,
        left_detection: dict[str, Any],
        right_detection: dict[str, Any],
    ) -> Optional[dict[str, Any]]:
        # ====================================================
        # 時間差
        # ====================================================
        left_time = self.detection_time_sec(left_detection)
        right_time = self.detection_time_sec(right_detection)
        time_difference = abs(left_time - right_time)

        if time_difference > self.max_time_difference_sec:
            return None

        # ====================================================
        # 原始像素
        # ====================================================
        left_raw = left_detection['pixel_center']
        right_raw = right_detection['pixel_center']

        # ====================================================
        # 去畸變
        # ====================================================
        left_normalized = self.normalize_pixel(
            pixel_center=left_raw,
            camera_matrix=self.k_left,
            dist_coeffs=self.d_left,
        )
        right_normalized = self.normalize_pixel(
            pixel_center=right_raw,
            camera_matrix=self.k_right,
            dist_coeffs=self.d_right,
        )
        left_point_2d = np.array(
            [[left_normalized[0]], [left_normalized[1]]], dtype=np.float64
        )
        right_point_2d = np.array(
            [[right_normalized[0]], [right_normalized[1]]], dtype=np.float64
        )

        # ====================================================
        # triangulation
        # ====================================================
        homogeneous_point = cv2.triangulatePoints(
            projMatr1=self.projection_left,
            projMatr2=self.projection_right,
            projPoints1=left_point_2d,
            projPoints2=right_point_2d,
        )

        scale = float(homogeneous_point[3, 0])
        if abs(scale) < 1e-12:
            return None

        point_left = homogeneous_point[:3, 0] / scale
        if not np.all(np.isfinite(point_left)):
            return None

        # ====================================================
        # 左 -> 右相機座標
        # ====================================================
        point_right = (
            self.stereo_r @ point_left.reshape(3, 1) + self.stereo_t
        ).reshape(3)
        left_z_m = float(point_left[2])
        right_z_m = float(point_right[2])

        if left_z_m <= 0.0 or right_z_m <= 0.0:
            return None

        # ====================================================
        # 重投影
        # ====================================================
        left_reprojected = self.project_camera_point(
            point_camera=point_left,
            camera_matrix=self.k_left,
            dist_coeffs=self.d_left,
        )
        right_reprojected = self.project_camera_point(
            point_camera=point_right,
            camera_matrix=self.k_right,
            dist_coeffs=self.d_right,
        )
        left_raw_array = np.array(left_raw, dtype=np.float64)
        right_raw_array = np.array(right_raw, dtype=np.float64)
        left_error_px = float(np.linalg.norm(left_reprojected - left_raw_array))
        right_error_px = float(np.linalg.norm(right_reprojected - right_raw_array))
        mean_error_px = (left_error_px + right_error_px) / 2.0

        return {
            'left_raw': left_raw,
            'right_raw': right_raw,
            'left_normalized': left_normalized,
            'right_normalized': right_normalized,
            'point_left': point_left,
            'point_right': point_right,
            'left_z_m': left_z_m,
            'right_z_m': right_z_m,
            'left_error_px': left_error_px,
            'right_error_px': right_error_px,
            'mean_error_px': mean_error_px,
            'time_difference': time_difference,
        }

    # ========================================================
    # 左右 detection 直接配對
    # ========================================================
    def try_triangulate(self):
        if not self.latest_left:
            return
        if not self.latest_right:
            return
        right_by_track_key = {detection['track_key']: detection for detection in self.latest_right}
        output_detections = []

        for left_detection in self.latest_left:
            track_key = left_detection['track_key']
            right_detection = right_by_track_key.get(track_key)
            if right_detection is None:
                continue

            left_class_name = left_detection.get('class_name')
            right_class_name = right_detection.get('class_name')
            if left_class_name is not None and right_class_name is not None and left_class_name != right_class_name:
                continue

            result = self.triangulate(left_detection, right_detection)
            if result is None:
                continue

            mean_error_px = float(result['mean_error_px'])
            left_error_px = float(result['left_error_px'])
            right_error_px = float(result['right_error_px'])
            if mean_error_px > self.max_reprojection_error_px:
                self.get_logger().warn(
                    f'{track_key} reprojection error too large | left={left_error_px:.3f}px, '
                    f'right={right_error_px:.3f}px, mean={mean_error_px:.3f}px',
                    throttle_duration_sec=1.0,
                )
                continue

            point_left = result['point_left']
            x_m, y_m, z_m = map(float, point_left)
            distance_m = float(np.linalg.norm(point_left))
            left_raw = result['left_raw']
            right_raw = result['right_raw']
            left_normalized = result['left_normalized']
            right_normalized = result['right_normalized']

            output_detections.append({
                'track_key': track_key,
                'class_id': left_detection.get('class_id'),
                'class_name': left_class_name,
                'confidence': float(left_detection.get('confidence', 0.0)),
                'pixel_center': [float(left_raw[0]), float(left_raw[1])],
                'left_pixel_raw': [float(left_raw[0]), float(left_raw[1])],
                'right_pixel_raw': [float(right_raw[0]), float(right_raw[1])],
                'left_normalized': [float(left_normalized[0]), float(left_normalized[1])],
                'right_normalized': [float(right_normalized[0]), float(right_normalized[1])],
                'time_difference_sec': float(result['time_difference']),
                'reprojection_error_left_px': left_error_px,
                'reprojection_error_right_px': right_error_px,
                'reprojection_error_mean_px': mean_error_px,
                'detection_camera_position_m': [x_m, y_m, z_m],
                'detection_camera_position_mm': [x_m * 1000.0, y_m * 1000.0, z_m * 1000.0],
                'distance_m': distance_m,
                'right_camera_z_m': float(result['right_z_m']),
            })

            self.get_logger().info(
                f'{track_key} | XYZ_left=({x_m:.4f}, {y_m:.4f}, {z_m:.4f}) m | '
                f'reproj=({left_error_px:.2f}, {right_error_px:.2f}) px'
            )

        if not output_detections:
            return

        output = {
            'coordinate_frame': 'left_camera',
            'detections': output_detections,
        }

        output_msg = String()
        output_msg.data = json.dumps(output)
        self.depth_pub.publish(output_msg)

        self.get_logger().info(f'Published {len(output_detections)} stereo detections')


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()

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
