#!/usr/bin/env python3
import os

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class StereoAprilTagCalibrator(Node):
    def __init__(self):
        super().__init__('stereo_apriltag_calibrator')

        self.declare_parameter('left_image_topic', '/camera_left/camera_left/color/image_raw')
        self.declare_parameter('right_image_topic', '/camera_right/camera_right/color/image_raw')
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('tag_size_m', 0.05)
        self.declare_parameter('process_rate_hz', 8.0)
        self.declare_parameter('max_preview_dt_sec', 0.08)
        self.declare_parameter('max_capture_dt_sec', 0.02)
        self.declare_parameter('minimum_samples', 25)
        self.declare_parameter('output_file', 'src/yolo/yolo/config/stereo_apriltag_extrinsics.yaml')

        p = lambda name: self.get_parameter(name).value
        self.left_topic = str(p('left_image_topic'))
        self.right_topic = str(p('right_image_topic'))
        self.tag_id = int(p('tag_id'))
        self.tag_size = float(p('tag_size_m'))
        self.process_rate_hz = max(1.0, float(p('process_rate_hz')))
        self.max_preview_dt = float(p('max_preview_dt_sec'))
        self.max_capture_dt = float(p('max_capture_dt_sec'))
        self.minimum_samples = int(p('minimum_samples'))
        self.output_file = os.path.abspath(os.path.expanduser(str(p('output_file'))))

        self.detector = Detector(
            families=str(p('tag_family')),
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )
        self.bridge = CvBridge()

        # 左相機 camera6，1920x1080，0813 單目標定結果
        self.k_left = np.array([
            [1370.481966698142, 0.0, 957.8839678123309],
            [0.0, 1368.2282506378035, 541.0615417682238],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self.d_left = np.array([
            0.18970955702068168,
            -0.5620843081215285,
            0.0014242865497269136,
            -0.002233374919288368,
            0.4613305846866371,
        ], dtype=np.float64).reshape(-1, 1)

        # 右相機 camera2，1920x1080，0818 單目標定結果
        self.k_right = np.array([
            [1376.2993702005285, 0.0, 966.4965648414827],
            [0.0, 1374.3873733700716, 554.7627465081724],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self.d_right = np.array([
            0.17302852476860411,
            -0.558236486919957,
            0.0018853562229434549,
            0.001071749355689855,
            0.5066598285552415,
        ], dtype=np.float64).reshape(-1, 1)

        self.image_size = None
        self.latest_result = None
        self.latest_messages = {'left': None, 'right': None}
        self.receive_count = {'left': 0, 'right': 0}
        self.process_count = 0
        self.last_pair_key = None
        self.busy = False
        self.samples_object = []
        self.samples_left = []
        self.samples_right = []

        self.window_name = 'Stereo AprilTag calibration'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1500, 540)
        self.show_waiting('Waiting for left and right images...')

        self.left_sub = self.create_subscription(
            Image, self.left_topic, self.left_callback, qos_profile_sensor_data
        )
        self.right_sub = self.create_subscription(
            Image, self.right_topic, self.right_callback, qos_profile_sensor_data
        )
        self.create_timer(1.0 / self.process_rate_hz, self.process_latest_pair)
        self.create_timer(0.03, self.gui_callback)
        self.create_timer(2.0, self.watchdog_callback)

        self.get_logger().info(
            '使用 camera6/camera2 離線 K、D；'
            f'處理頻率={self.process_rate_hz:.1f}Hz，'
            f'取樣時間差限制={self.max_capture_dt * 1000.0:.1f}ms；'
            '視窗中按 s 取樣、q 計算，終端 Ctrl+C 離開。'
        )

    def show_waiting(self, text):
        image = np.zeros((540, 1500, 3), dtype=np.uint8)
        cv2.putText(
            image, text, (70, 260), cv2.FONT_HERSHEY_SIMPLEX,
            1.2, (0, 255, 255), 3, cv2.LINE_AA
        )
        cv2.imshow(self.window_name, image)
        cv2.waitKey(1)

    def left_callback(self, msg):
        self.receive_count['left'] += 1
        self.latest_messages['left'] = msg

    def right_callback(self, msg):
        self.receive_count['right'] += 1
        self.latest_messages['right'] = msg

    @staticmethod
    def stamp_seconds(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def gui_callback(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self.capture_sample()
        elif key == ord('q'):
            self.calibrate_and_save()

    def watchdog_callback(self):
        self.get_logger().info(
            f'WATCHDOG | received L/R={self.receive_count["left"]}/'
            f'{self.receive_count["right"]} | processed pairs={self.process_count} | '
            f'samples={len(self.samples_object)}'
        )

    def process_latest_pair(self):
        if self.busy:
            return
        left_msg = self.latest_messages['left']
        right_msg = self.latest_messages['right']
        if left_msg is None or right_msg is None:
            return

        pair_key = (
            left_msg.header.stamp.sec,
            left_msg.header.stamp.nanosec,
            right_msg.header.stamp.sec,
            right_msg.header.stamp.nanosec,
        )
        if pair_key == self.last_pair_key:
            return

        self.last_pair_key = pair_key
        self.busy = True
        try:
            left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
            if left.shape[:2] != right.shape[:2]:
                self.get_logger().error('左右影像解析度不同，不能標定。')
                return
            if left.shape[1] != 1920 or left.shape[0] != 1080:
                self.get_logger().error(
                    f'目前影像為 {left.shape[1]}x{left.shape[0]}，'
                    '但填入的 K、D 僅適用於 1920x1080。'
                )
                return

            dt_sec = abs(self.stamp_seconds(left_msg) - self.stamp_seconds(right_msg))
            dt_ms = dt_sec * 1000.0
            self.image_size = (left.shape[1], left.shape[0])
            corners_left = self.find_tag(left)
            corners_right = self.find_tag(right)
            self.latest_result = (corners_left, corners_right, dt_ms)
            self.process_count += 1
            self.draw_preview(left, right, corners_left, corners_right, dt_ms)
        except Exception as exc:
            self.get_logger().error(f'處理左右影像失敗：{exc}')
        finally:
            self.busy = False

    def find_tag(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray, estimate_tag_pose=False)
        matches = [d for d in detections if int(d.tag_id) == self.tag_id]
        if not matches:
            return None
        detection = max(
            matches,
            key=lambda d: abs(cv2.contourArea(np.asarray(d.corners, dtype=np.float32))),
        )
        corners = np.asarray(detection.corners, dtype=np.float32).reshape(4, 2)
        if abs(cv2.contourArea(corners)) < 400.0:
            return None
        return corners

    def draw_preview(self, left, right, corners_left, corners_right, dt_ms):
        for image, corners, label in (
            (left, corners_left, 'LEFT'),
            (right, corners_right, 'RIGHT'),
        ):
            color = (0, 255, 0) if corners is not None else (0, 0, 255)
            if corners is not None:
                integer_corners = np.rint(corners).astype(np.int32)
                cv2.polylines(image, [integer_corners], True, color, 3)
                for index, point in enumerate(integer_corners):
                    cv2.putText(
                        image, str(index), tuple(point), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 0, 255), 2
                    )
            cv2.putText(
                image,
                f'{label}: {"FOUND" if corners is not None else "NOT FOUND"}',
                (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2
            )

        preview = np.hstack((left, right))
        scale = min(0.5, 1500.0 / preview.shape[1])
        preview = cv2.resize(
            preview, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
        )
        dt_color = (
            (0, 255, 0) if dt_ms <= self.max_capture_dt * 1000.0
            else (0, 165, 255) if dt_ms <= self.max_preview_dt * 1000.0
            else (0, 0, 255)
        )
        status = (
            f'samples={len(self.samples_object)}  dt={dt_ms:.1f}ms '
            f'(save limit={self.max_capture_dt * 1000.0:.0f}ms)  '
            '[s] save  [q] calibrate  [Ctrl+C] exit'
        )
        cv2.putText(
            preview, status, (20, preview.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, dt_color, 2
        )
        cv2.imshow(self.window_name, preview)

    def capture_sample(self):
        if self.latest_result is None:
            self.get_logger().warning('尚未取得可處理的左右影像。')
            return
        corners_left, corners_right, dt_ms = self.latest_result
        if corners_left is None or corners_right is None:
            self.get_logger().warning('左右相機必須同時找到指定 Tag，這組未儲存。')
            return
        if dt_ms > self.max_capture_dt * 1000.0:
            self.get_logger().warning(
                f'時間差 {dt_ms:.1f}ms 過大；'
                f'取樣必須小於 {self.max_capture_dt * 1000.0:.1f}ms。'
            )
            return

        half = self.tag_size / 2.0
        object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        self.samples_object.append(object_points)
        self.samples_left.append(corners_left.copy())
        self.samples_right.append(corners_right.copy())
        self.get_logger().info(
            f'已儲存第 {len(self.samples_object)} 組，左右時間差 {dt_ms:.1f}ms。'
        )

    @staticmethod
    def epipolar_errors(points_left, points_right, fundamental, k_left, d_left, k_right, d_right):
        errors = []
        for distorted_left, distorted_right in zip(points_left, points_right):
            left = cv2.undistortPoints(
                distorted_left.reshape(-1, 1, 2), k_left, d_left, P=k_left
            ).reshape(-1, 2)
            right = cv2.undistortPoints(
                distorted_right.reshape(-1, 1, 2), k_right, d_right, P=k_right
            ).reshape(-1, 2)
            left_h = np.column_stack((left, np.ones(len(left))))
            right_h = np.column_stack((right, np.ones(len(right))))
            lines_right = (fundamental @ left_h.T).T
            lines_left = (fundamental.T @ right_h.T).T
            numerators = np.abs(np.sum(right_h * lines_right, axis=1))
            distance_right = numerators / np.maximum(
                np.linalg.norm(lines_right[:, :2], axis=1), 1e-12
            )
            distance_left = numerators / np.maximum(
                np.linalg.norm(lines_left[:, :2], axis=1), 1e-12
            )
            errors.extend(((distance_left + distance_right) * 0.5).tolist())
        return np.asarray(errors)

    def calibrate_and_save(self):
        count = len(self.samples_object)
        if count < self.minimum_samples:
            self.get_logger().warning(
                f'目前只有 {count} 組，至少需要 {self.minimum_samples} 組。'
            )
            return
        if self.image_size is None:
            self.get_logger().error('沒有有效影像尺寸，無法標定。')
            return

        self.get_logger().info('開始 stereoCalibrate，視窗短暫停止更新屬正常現象。')
        flags = cv2.CALIB_FIX_INTRINSIC
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            300,
            1e-9,
        )
        (
            rms, _, _, _, _, rotation, translation, essential, fundamental
        ) = cv2.stereoCalibrate(
            self.samples_object,
            self.samples_left,
            self.samples_right,
            self.k_left.copy(),
            self.d_left.copy(),
            self.k_right.copy(),
            self.d_right.copy(),
            self.image_size,
            criteria=criteria,
            flags=flags,
        )
        epipolar = self.epipolar_errors(
            self.samples_left,
            self.samples_right,
            fundamental,
            self.k_left,
            self.d_left,
            self.k_right,
            self.d_right,
        )
        rotation_angle_deg = np.degrees(
            np.arccos(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
        )
        result = {
            'definition': 'X_right = R * X_left + T',
            'frame': 'left_color_optical_frame_to_right_color_optical_frame',
            'image_width': int(self.image_size[0]),
            'image_height': int(self.image_size[1]),
            'tag_family': str(self.get_parameter('tag_family').value),
            'tag_id': self.tag_id,
            'tag_size_m': self.tag_size,
            'intrinsics_source': 'left=camera6_0813, right=camera2_0818, 1920x1080',
            'sample_count': count,
            'maximum_capture_dt_ms': self.max_capture_dt * 1000.0,
            'stereo_rms_px': float(rms),
            'symmetric_epipolar_mean_px': float(np.mean(epipolar)),
            'symmetric_epipolar_median_px': float(np.median(epipolar)),
            'symmetric_epipolar_max_px': float(np.max(epipolar)),
            'rotation_angle_deg': float(rotation_angle_deg),
            'baseline_m': float(np.linalg.norm(translation)),
            'stereo_rotation': rotation.tolist(),
            'stereo_translation': translation.reshape(3).tolist(),
            'essential_matrix': essential.tolist(),
            'fundamental_matrix': fundamental.tolist(),
            'left_camera_matrix': self.k_left.tolist(),
            'left_distortion': self.d_left.reshape(-1).tolist(),
            'right_camera_matrix': self.k_right.tolist(),
            'right_distortion': self.d_right.reshape(-1).tolist(),
        }

        output_directory = os.path.dirname(self.output_file)
        if output_directory:
            os.makedirs(output_directory, exist_ok=True)
        with open(self.output_file, 'w', encoding='utf-8') as file:
            yaml.safe_dump(result, file, sort_keys=False, allow_unicode=True)

        self.get_logger().info(
            f'完成：RMS={rms:.3f}px，極線平均={np.mean(epipolar):.3f}px，'
            f'中位數={np.median(epipolar):.3f}px，'
            f'baseline={np.linalg.norm(translation):.4f}m，'
            f'旋轉角={rotation_angle_deg:.2f}deg'
        )
        self.get_logger().info(f'結果已寫入 {self.output_file}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoAprilTagCalibrator()
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
