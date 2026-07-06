import json

from hiwin_interfaces.srv import RobotCommand
from std_msgs.msg import String


ACTIVE_TOOL = 7
ACTIVE_BASE = 0


class ToolPoseMixin:

    # ========================================================
    # 取得並發布 Tool7 位姿
    # ========================================================

    def update_and_publish_tool_pose(
        self,
        save_photo_orientation=False
    ):
        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.CHECK_POSE,
            tool=ACTIVE_TOOL,
            base=ACTIVE_BASE,
            holding=True
        )

        response = self.call_hiwin(request)

        if response is None:
            self.get_logger().error(
                'CHECK_POSE returned no response'
            )
            return False

        if not response.current_position:
            self.get_logger().error(
                'CHECK_POSE returned empty current_position'
            )
            return False

        if len(response.current_position) < 6:
            self.get_logger().error(
                'current_position needs at least 6 values'
            )
            return False

        x, y, z, rx, ry, rz = [
            float(value)
            for value in response.current_position[:6]
        ]

        if save_photo_orientation:
            self.photo_orientation_deg = [
                rx,
                ry,
                rz
            ]

        data = {
            'frame': 'base',
            'child_frame': 'tool7',
            'tool': ACTIVE_TOOL,
            'base': ACTIVE_BASE,
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
            'stamp_ns': self.get_clock().now().nanoseconds
        }

        message = String()
        message.data = json.dumps(data)

        self.tool_pose_pub.publish(message)

        self.get_logger().info(
            f'Published Base -> Tool7: '
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'z={z:.3f}, '
            f'rx={rx:.3f}, '
            f'ry={ry:.3f}, '
            f'rz={rz:.3f}'
        )

        return True