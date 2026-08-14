#!/usr/bin/env python3

import json

import numpy as np


class YoloDetectionMixin:

    def yolo_detections_base_callback(self, msg):
        try:
            data = json.loads(msg.data)

        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Failed to decode YOLO detections base: {exc}'
            )
            return

        except Exception as exc:
            self.get_logger().error(
                f'Failed to parse YOLO detections base: {exc}'
            )
            return

        self.update_base_camera_rotation(data)

        detections = data.get('detections', [])

        if not isinstance(detections, list):
            self.get_logger().warn(
                'YOLO detections must be a list'
            )
            return

        selected = self.find_target_detection(
            detections
        )

        if selected is None:
            self.get_logger().warn(
                f'Target {self.TARGET_RJ45_TRACK_KEY} '
                f'was not found'
            )
            return

        self.update_yolo_target(selected)

    def update_base_camera_rotation(self, data):
        r_base_camera = data.get(
            'r_base_camera'
        )

        if r_base_camera is None:
            self.get_logger().warn(
                'R_base_camera is missing '
                'from YOLO base message'
            )
            return False

        matrix = np.asarray(
            r_base_camera,
            dtype=float
        )

        if matrix.shape != (3, 3):
            self.get_logger().warn(
                f'Invalid R_base_camera '
                f'shape: {matrix.shape}'
            )
            return False

        if not np.all(np.isfinite(matrix)):
            self.get_logger().warn(
                'R_base_camera contains invalid numbers'
            )
            return False

        self.latest_r_base_camera = matrix
        return True

    def find_target_detection(self, detections):
        for detection in detections:
            if (
                detection.get('track_key')
                == self.TARGET_RJ45_TRACK_KEY
            ):
                return detection

        return None

    def update_yolo_target(self, selected):
        position_m = selected.get(
            'detection_base_position_m'
        )

        if (
            position_m is None
            or len(position_m) != 3
            or any(value is None for value in position_m)
        ):
            self.get_logger().warn(
                f'{self.TARGET_RJ45_TRACK_KEY} '
                f'has invalid base position'
            )
            return False

        self.latest_yolo_detection_base_m = [
            float(position_m[0]),
            float(position_m[1]),
            float(position_m[2])
        ]

        self.latest_yolo_detection_info = selected
        self.has_yolo_detection = True

        x, y, z = (
            self.latest_yolo_detection_base_m
        )

        self.get_logger().info(
            f'Selected {self.TARGET_RJ45_TRACK_KEY}: '
            f'x={x * 1000.0:.3f} mm, '
            f'y={y * 1000.0:.3f} mm, '
            f'z={z * 1000.0:.3f} mm'
        )

        pixel_center = selected.get(
            'pixel_center'
        )

        if (
            pixel_center is not None
            and len(pixel_center) == 2
        ):
            self.latest_hole_center_px = [
                float(pixel_center[0]),
                float(pixel_center[1])
            ]

            self.get_logger().info(
                f'YOLO hole center: '
                f'u={self.latest_hole_center_px[0]:.2f}, '
                f'v={self.latest_hole_center_px[1]:.2f}'
            )

        return True