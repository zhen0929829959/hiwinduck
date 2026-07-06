from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R


class Rj45TargetMixin:
    """
    負責計算移動到 RJ45 上方的目標 Pose。

    不負責：
    - 發送 HIWIN 指令
    - 切換狀態
    """

    def calculate_rj45_target_pose(self):
        """
        根據最新的 RJ45 位置與 AprilTag 姿態，
        計算手臂移動到 RJ45 上方的目標 Pose。

        成功：
            回傳 Twist

        失敗：
            回傳 None
        """

        if (
            self.latest_tag_rj45_position_m
            is None
        ):
            self.get_logger().error(
                'Second RJ45 localization '
                'data is missing'
            )
            return None

        if self.latest_tag_quat is None:
            self.get_logger().error(
                'Second AprilTag orientation '
                'data is missing'
            )
            return None

        if not self.latest_centered:
            self.get_logger().error(
                'Refusing to calculate RJ45 target '
                'because AprilTag is not centered'
            )
            return None

        rj45_x, rj45_y, rj45_z = (
            self.latest_tag_rj45_position_m
        )

        qx, qy, qz, qw = (
            self.latest_tag_quat
        )

        try:
            rx, ry, rz = R.from_quat(
                [qx, qy, qz, qw]
            ).as_euler(
                'xyz',
                degrees=True
            )

        except Exception as exc:
            self.get_logger().error(
                f'Quaternion conversion failed: '
                f'{exc}'
            )
            return None

        x_raw = (
            float(rj45_x)
            * 1000.0
        )

        y_raw = (
            float(rj45_y)
            * 1000.0
        )

        z_raw = (
            float(rj45_z)
            * 1000.0
        )

        x_corrected, y_corrected = (
            self.apply_rj45_xy_correction(
                x_raw=x_raw,
                y_raw=y_raw
            )
        )

        pose = Twist()

        pose.linear.x = x_corrected
        pose.linear.y = y_corrected

        pose.linear.z = (
            z_raw
            + self.RJ45_APPROACH_Z_MM
        )

        pose.angular.x = float(rx)
        pose.angular.y = float(ry)

        pose.angular.z = (
            float(rz)
            - self.TARGET_RZ_OFFSET
        )

        self.log_rj45_target_pose(
            pose
        )

        return pose

    def apply_rj45_xy_correction(
        self,
        x_raw,
        y_raw
    ):
        """
        套用實驗得到的 XY 仿射補償公式。
        """

        x_corrected = (
            0.95064356 * x_raw
            - 0.04529201 * y_raw
            + 23.15631050
        )

        y_corrected = (
            0.02028272 * x_raw
            + 1.00011377 * y_raw
            - 1.67192112
        )

        self.get_logger().info(
            f'RJ45 raw position: '
            f'x={x_raw:.3f}, '
            f'y={y_raw:.3f}'
        )

        self.get_logger().info(
            f'RJ45 corrected position: '
            f'x={x_corrected:.3f}, '
            f'y={y_corrected:.3f}'
        )

        return x_corrected, y_corrected

    def log_rj45_target_pose(
        self,
        pose
    ):
        """
        顯示最後計算出的 RJ45 目標位置。
        """

        self.get_logger().info(
            f'Target above RJ45: '
            f'x={pose.linear.x:.3f}, '
            f'y={pose.linear.y:.3f}, '
            f'z={pose.linear.z:.3f}, '
            f'rx={pose.angular.x:.3f}, '
            f'ry={pose.angular.y:.3f}, '
            f'rz={pose.angular.z:.3f}'
        )