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
        self.latest_center_error = None

        self.create_subscription(
            String,
            '/apriltag/center_error',
            self.center_error_callback,
            10
        )

        self.get_logger().info('AprilTag matrix node started')

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

        tag_position = T_base_tag[:3, 3]
        tag_quat = R.from_matrix(
            T_base_tag[:3, :3]
        ).as_quat()

        data = {
            'frame': 'base',
            'position_m': tag_position.tolist(),
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