#!/usr/bin/env python3

import json

from std_msgs.msg import String


class ToolPoseMixin:

    # ========================================================
    # 取得並發布目前 UR Active TCP Pose
    #
    # 為了讓 camera_flange_matrix2.py 暫時不用改，
    # topic payload 仍保留舊名稱：
    #
    #   child_frame = "tool7"
    #
    # 但這裡的 tool7 實際意義已經是：
    #   UR Teach Pendant Active TCP
    # ========================================================

    def update_and_publish_tool_pose(
        self,
        save_photo_orientation=False
    ):
        current_pose = self.get_current_robot_pose()

        if current_pose is None:
            self.get_logger().error(
                'Cannot get current UR TCP pose'
            )
            return False

        if len(current_pose) < 6:
            self.get_logger().error(
                'Current TCP pose needs 6 values'
            )
            return False

        x, y, z, rx, ry, rz = [
            float(value)
            for value in current_pose[:6]
        ]

        if save_photo_orientation:
            self.photo_orientation_deg = [
                rx,
                ry,
                rz
            ]

        data = {
            # 保留舊格式，讓 camera_flange_matrix2.py
            # 可以繼續直接使用。
            'frame': 'base_link',
            'child_frame': 'tool7',
            'position_mm': [
                x,
                y,
                z
            ],
            'euler_deg': [
                rx,
                ry,
                rz
            ],
            'stamp_ns': (
                self.get_clock().now().nanoseconds
            )
        }

        message = String()
        message.data = json.dumps(data)

        self.tool_pose_pub.publish(message)

        self.get_logger().info(
            f'Published Base_link -> TCP: '
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'z={z:.3f}, '
            f'rx={rx:.3f}, '
            f'ry={ry:.3f}, '
            f'rz={rz:.3f}'
        )

        return True