#!/usr/bin/env python3

import json
import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String


class AprilTagBaseNode(Node):
    def __init__(self):
        super().__init__('apriltag_base_node')

        self.latest_tool_pose = None

        self.create_subscription(
            PoseStamped,
            '/apriltag/pose_camera',
            self.apriltag_callback,
            10
        )

        self.create_subscription(
            String,
            '/hiwin/tool_pose',
            self.tool_pose_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            String,
            '/apriltag/pose_base',
            10
        )

        self.yolo_base_pub = self.create_publisher(
            String,
            '/yolo/detections_base',
            10
        )

        self.latest_center_error = None
        self.latest_yolo_detection = None

        self.create_subscription(
            String,
            '/yolo/detections',
            self.yolo_detections_callback,
            10
        )

        self.create_subscription(
            String,
            '/apriltag/center_error',
            self.center_error_callback,
            10
        )

        self.get_logger().info('AprilTag matrix node started')
    
    def publish_yolo_detections_base(self, detections):
        T_base_tool7 = self.get_T_base_tool7()

        if T_base_tool7 is None:
            self.get_logger().warn(
                'No Tool7 pose yet, cannot transform YOLO detections'
            )
            return False

        T_base_camera = (
            T_base_tool7
            @ self.get_T_tool7_camera_optical()
        )

        output_detections = []

        for det in detections:
            # 只轉 RJ45，不轉 RJ45_card
            if det.get('class_name') != 'RJ45':
                continue

            camera_xyz = det.get('camera_xyz')

            if (
                camera_xyz is None
                or len(camera_xyz) != 3
                or any(v is None for v in camera_xyz)
            ):
                continue

            x_mm, y_mm, z_mm = camera_xyz

            P_camera = np.array([
                float(x_mm) / 1000.0,
                float(y_mm) / 1000.0,
                float(z_mm) / 1000.0,
                1.0
            ])

            P_base = T_base_camera @ P_camera

            output_detections.append({
                'class_name': det.get('class_name'),
                'stable_id': det.get('stable_id'),
                'track_key': det.get('track_key'),
                'confidence': det.get('confidence'),
                'class_id': det.get('class_id'),
                'angle': det.get('angle'),
                'pixel_center': det.get('pixel_center'),
                'center_source': det.get('center_source'),
                'camera_xyz_mm': camera_xyz,
                'detection_base_position_m': P_base[:3].tolist()
            })

        if not output_detections:
            return False

        output = String()
        output.data = json.dumps({
            'frame': 'base',
            'detections': output_detections
        })

        self.yolo_base_pub.publish(output)

        self.get_logger().info(
            f'Published {len(output_detections)} YOLO detections in base frame'
        )

        return True

    def tool_pose_callback(self, msg):
        try:
            data = json.loads(msg.data)

            if data.get('child_frame') != 'tool7':
                self.get_logger().warn(
                    f'Unexpected frame: {data.get("child_frame")}'
                )
                return

            self.latest_tool_pose = data

        except Exception as exc:
            self.get_logger().error(
                f'Failed to parse tool pose: {exc}'
            )

    def center_error_callback(self, msg):
        try:
            self.latest_center_error = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to parse center error: {exc}'
            )

    def yolo_detections_callback(self, msg):
        try:
            data = json.loads(msg.data)

            if isinstance(data, dict):
                detections = data.get('detections', [])
            elif isinstance(data, list):
                detections = data
            else:
                self.get_logger().warn(
                    'YOLO detections format is not list or dict'
                )
                return

            self.publish_yolo_detections_base(
                detections
            )

        except Exception as exc:
            self.get_logger().error(
                f'Failed to parse YOLO detections: {exc}'
            )

    def get_T_base_tool7(self):
        if self.latest_tool_pose is None:
            return None

        x_mm, y_mm, z_mm = self.latest_tool_pose['position_mm']
        rx, ry, rz = self.latest_tool_pose['euler_deg']

        T = np.eye(4)
        T[:3, :3] = R.from_euler(
            'xyz',
            [rx, ry, rz],
            degrees=True
        ).as_matrix()

        T[:3, 3] = [
            x_mm / 1000.0,
            y_mm / 1000.0,
            z_mm / 1000.0
        ]

        return T

    def get_T_flange_tool7(self):
        T = np.eye(4)
        T[:3, 3] = [
            0.005087,
            -0.005525,
            0.191133
        ]
        return T

    def get_T_flange_camera_optical(self):
        T = np.eye(4)

        T[:3, :3] = R.from_quat([
            0.002874781996797597,
            -0.3562352693857901,
            0.9341205845947474,
            0.022514482238590865
        ]).as_matrix()

        T[:3, 3] = [
            0.03577462614985271,
            0.10816222386275594,
            0.09982944540467038
        ]

        return T

    def get_T_tool7_camera_optical(self):
        return (
            np.linalg.inv(self.get_T_flange_tool7())
            @ self.get_T_flange_camera_optical()
        )

    # def calculate_camera_aligned_tool7_pose(
    #     self,
    #     T_base_tag
    # ):
    #     """
    #     計算相機正對 AprilTag 時，需要的 Base → Tool7。

    #     目標軸向：
    #         Camera X =  Tag X
    #         Camera Y = -Tag Y
    #         Camera Z = -Tag Z

    #     也就是 Camera 相對 Tag X 軸旋轉 180 度。
    #     """

    #     if T_base_tag.shape != (4, 4):
    #         raise ValueError(
    #             'T_base_tag must be a 4x4 matrix'
    #         )

    #     # ------------------------------------------------
    #     # Tag → 期望 Camera
    #     # ------------------------------------------------

    #     T_tag_camera_desired = np.eye(4)

    #     T_tag_camera_desired[:3, :3] = (
    #         R.from_euler(
    #             'x',
    #             180.0,
    #             degrees=True
    #         ).as_matrix()
    #     )

    #     # ------------------------------------------------
    #     # Base → 期望 Camera
    #     # ------------------------------------------------

    #     T_base_camera_target = (
    #         T_base_tag
    #         @ T_tag_camera_desired
    #     )

    #     # ------------------------------------------------
    #     # 已知 Tool7 → Camera optical
    #     # ------------------------------------------------

    #     T_tool7_camera = (
    #         self.get_T_tool7_camera_optical()
    #     )

    #     # ------------------------------------------------
    #     # 反推出 Base → Tool7
    #     #
    #     # Base→Camera
    #     # = Base→Tool7 × Tool7→Camera
    #     #
    #     # 所以：
    #     # Base→Tool7
    #     # = Base→Camera × Camera→Tool7
    #     # ------------------------------------------------

    #     T_base_tool7_target = (
    #         T_base_camera_target
    #         @ np.linalg.inv(
    #             T_tool7_camera
    #         )
    #     )

    #     tool_position_m = (
    #         T_base_tool7_target[:3, 3]
    #     )

    #     tool_rotation = R.from_matrix(
    #         T_base_tool7_target[:3, :3]
    #     )

    #     tool_quat = tool_rotation.as_quat()

    #     tool_euler_deg = tool_rotation.as_euler(
    #         'xyz',
    #         degrees=True
    #     )

    #     return {
    #         'position_m': (
    #             tool_position_m.tolist()
    #         ),
    #         'orientation_quat_xyzw': (
    #             tool_quat.tolist()
    #         ),
    #         'euler_deg': (
    #             tool_euler_deg.tolist()
    #         )
    #     }

    def pose_msg_to_matrix(self, msg):
        quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ], dtype=float)

        norm = np.linalg.norm(quat)

        if norm < 1e-12:
            raise ValueError('Quaternion norm is zero')

        quat /= norm

        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

        return T
    
    def camera_xyz_mm_to_base(self, camera_xyz_mm):
        T_base_tool7 = self.get_T_base_tool7()

        if T_base_tool7 is None:
            self.get_logger().warn(
                'No Tool7 pose yet, cannot transform YOLO target'
            )
            return None

        T_base_camera = (
            T_base_tool7
            @ self.get_T_tool7_camera_optical()
        )

        x_mm, y_mm, z_mm = camera_xyz_mm

        P_camera = np.array([
            float(x_mm) / 1000.0,
            float(y_mm) / 1000.0,
            float(z_mm) / 1000.0,
            1.0
        ])

        P_base = T_base_camera @ P_camera

        return P_base[:3]
        
    def get_latest_yolo_detection_base(self):
        if self.latest_yolo_detection is None:
            self.get_logger().warn(
                'No valid YOLO detection yet'
            )
            return None

        camera_xyz_mm = self.latest_yolo_detection.get(
            'camera_xyz'
        )

        if (
            camera_xyz_mm is None
            or len(camera_xyz_mm) != 3
            or any(v is None for v in camera_xyz_mm)
        ):
            self.get_logger().warn(
                'Latest YOLO detection has invalid camera_xyz'
            )
            return None

        detection_base = self.camera_xyz_mm_to_base(
            camera_xyz_mm
        )

        if detection_base is None:
            return None

        return {
            'position_m': detection_base.tolist(),
            'class_name': self.latest_yolo_detection.get(
                'class_name'
            ),
            'stable_id': self.latest_yolo_detection.get(
                'stable_id'
            ),
            'track_key': self.latest_yolo_detection.get(
                'track_key'
            ),
            'confidence': self.latest_yolo_detection.get(
                'confidence'
            ),
            'class_id': self.latest_yolo_detection.get(
                'class_id'
            ),
            'angle': self.latest_yolo_detection.get(
                'angle'
            ),
            'pixel_center': self.latest_yolo_detection.get(
                'pixel_center'
            ),
            'center_source': self.latest_yolo_detection.get(
                'center_source'
            ),
            'camera_xyz_mm': camera_xyz_mm
        }

    def apriltag_callback(self, msg):
        if msg.header.frame_id != 'camera_color_optical_frame':
            self.get_logger().warn(
                f'Unexpected AprilTag frame: {msg.header.frame_id}'
            )
            return

        T_base_tool7 = self.get_T_base_tool7()

        if T_base_tool7 is None:
            self.get_logger().warn('No Tool7 pose yet')
            return

        try:
            T_base_tag = (
                T_base_tool7
                @ self.get_T_tool7_camera_optical()
                @ self.pose_msg_to_matrix(msg)
            )
            T_base_camera = (
                T_base_tool7
                @ self.get_T_tool7_camera_optical()
            )

            centered = False
            center_error_px = [9999.0, 9999.0]
            center_threshold_px = 8.0
            correction_base_m = [0.0, 0.0, 0.0]

            if self.latest_center_error is not None:
                centered = bool(
                    self.latest_center_error.get('centered', False)
                )

                center_error_px = self.latest_center_error.get(
                    'error_px',
                    [9999.0, 9999.0]
                )

                center_threshold_px = float(
                    self.latest_center_error.get(
                        'center_threshold_px',
                        8.0
                    )
                )

                correction_camera_m = np.array(
                    self.latest_center_error.get(
                        'correction_camera_m',
                        [0.0, 0.0, 0.0]
                    ),
                    dtype=float
                )

                correction_base = (
                    T_base_camera[:3, :3]
                    @ correction_camera_m
                )

                correction_base_m = correction_base.tolist()
        except Exception as exc:
            self.get_logger().error(
                f'Transform failed: {exc}'
            )
            return

        P_tag_rj45 = np.array([
            0.017976,   
            -0.179544,
            0.0,
            1.0
        ])

        P_base_rj45 = T_base_tag @ P_tag_rj45

        P_tag_board_center = np.array([
            0.027976,
            -0.129544,
            0.0,
            1.0
        ])

        P_base_board_center = T_base_tag @ P_tag_board_center

        tag_position = T_base_tag[:3, 3]
        tag_quat = R.from_matrix(
            T_base_tag[:3, :3]
        ).as_quat()

        # data = {
        #     'frame': 'base',
        #     'position_m': tag_position.tolist(),
        #     'rj45_position_m': P_base_rj45[:3].tolist(),
        #     'orientation_quat_xyzw': tag_quat.tolist(),

        #     'centered': centered,
        #     'center_error_px': center_error_px,
        #     'center_threshold_px': center_threshold_px,

        #     # 手臂在 Base 座標需要修正的 XYZ
        #     'center_correction_base_m': correction_base_m
        # }


        data = {
            'frame': 'base',
            'position_m': tag_position.tolist(),
            'board_center_base_position_m': P_base_board_center[:3].tolist(),
            'rj45_position_m': P_base_rj45[:3].tolist(),
            'orientation_quat_xyzw': tag_quat.tolist(),

            'centered': centered,
            'center_error_px': center_error_px,
            'center_threshold_px': center_threshold_px,

            # 手臂在 Base 座標需要修正的 XYZ
            'center_correction_base_m': correction_base_m
        }

        output = String()
        output.data = json.dumps(data)
        self.pose_pub.publish(output)

        self.get_logger().info(
            f'tag base position: 'f'({tag_position[0]:.6f}, 'f'{tag_position[1]:.6f}, 'f'{tag_position[2]:.6f}) m'
            f'RJ45 base position: 'f'({P_base_rj45[0]:.6f}, 'f'{P_base_rj45[1]:.6f}, 'f'{P_base_rj45[2]:.6f}) m'
            f'board center base position: 'f'({P_base_board_center[0]:.6f}, 'f'{P_base_board_center[1]:.6f}, 'f'{P_base_board_center[2]:.6f}) m'
        )





def main(args=None):
    rclpy.init(args=args)
    node = AprilTagBaseNode()

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