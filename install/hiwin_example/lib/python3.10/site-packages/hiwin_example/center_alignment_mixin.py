import numpy as np

from geometry_msgs.msg import Twist


class CenterAlignmentMixin:
    """
    負責 AprilTag 畫面置中的計算。

    只負責：
    1. 檢查修正資料
    2. 將公尺轉成毫米
    3. 限制單次最大修正量
    4. 計算手臂目標 Pose

    不負責：
    1. 發送 HIWIN 指令
    2. 切換 State
    """

    def calculate_center_alignment_pose(
        self,
        current_pose
    ):
        """
        根據 AprilTag 的修正量，
        計算手臂置中後的目標 Pose。

        成功：
            回傳 Twist

        失敗：
            回傳 None
        """

        if (
            self.latest_center_correction_base_m
            is None
        ):
            self.get_logger().error(
                'Center correction data is missing'
            )
            return None

        if self.latest_center_error_px is None:
            self.get_logger().error(
                'Center pixel error is missing'
            )
            return None

        if current_pose is None:
            self.get_logger().error(
                'Current robot pose is missing'
            )
            return None

        (
            current_x,
            current_y,
            current_z,
            current_rx,
            current_ry,
            current_rz
        ) = current_pose

        correction_mm = (
            np.array(
                self.latest_center_correction_base_m,
                dtype=float
            )
            * 1000.0
            * self.CENTER_CORRECTION_SIGN
        )

        if correction_mm.shape != (3,):
            self.get_logger().error(
                f'Invalid correction shape: '
                f'{correction_mm.shape}'
            )
            return None

        if not np.all(
            np.isfinite(correction_mm)
        ):
            self.get_logger().error(
                'Center correction contains '
                'invalid numerical values'
            )
            return None

        correction_norm = float(
            np.linalg.norm(
                correction_mm
            )
        )

        error_u, error_v = (
            self.latest_center_error_px
        )

        self.get_logger().info(
            f'Before alignment: '
            f'u={error_u:.2f}px, '
            f'v={error_v:.2f}px'
        )

        self.get_logger().info(
            f'Raw Base correction: '
            f'dx={correction_mm[0]:.3f}mm, '
            f'dy={correction_mm[1]:.3f}mm, '
            f'dz={correction_mm[2]:.3f}mm, '
            f'norm={correction_norm:.3f}mm'
        )

        if (
            correction_norm
            < self.MIN_CENTER_CORRECTION_MM
        ):
            self.get_logger().error(
                f'Correction is too small '
                f'({correction_norm:.3f} mm), '
                f'but AprilTag is not centered'
            )
            return None

        correction_mm = (
            self.limit_center_correction(
                correction_mm
            )
        )

        if correction_mm is None:
            return None

        target_pose = (
            self.create_center_alignment_pose(
                current_pose=current_pose,
                correction_mm=correction_mm
            )
        )

        self.log_center_alignment_target(
            target_pose
        )

        return target_pose

    def limit_center_correction(
        self,
        correction_mm
    ):
        """
        限制單次最大修正距離。
        """

        correction_norm = float(
            np.linalg.norm(
                correction_mm
            )
        )

        if correction_norm <= 0.0:
            self.get_logger().error(
                'Center correction norm is zero'
            )
            return None

        if (
            correction_norm
            <= self.MAX_CENTER_CORRECTION_MM
        ):
            return correction_mm

        limited_correction_mm = (
            correction_mm
            / correction_norm
            * self.MAX_CENTER_CORRECTION_MM
        )

        limited_norm = float(
            np.linalg.norm(
                limited_correction_mm
            )
        )

        self.get_logger().warn(
            f'Correction limited to '
            f'{limited_norm:.3f} mm'
        )

        return limited_correction_mm

    def create_center_alignment_pose(
        self,
        current_pose,
        correction_mm
    ):
        """
        將置中修正量加到目前手臂位置。

        只修改 XYZ。
        RX、RY、RZ 保持不變。
        """

        (
            current_x,
            current_y,
            current_z,
            current_rx,
            current_ry,
            current_rz
        ) = current_pose

        pose = Twist()

        pose.linear.x = (
            current_x
            + float(correction_mm[0])
        )

        pose.linear.y = (
            current_y
            + float(correction_mm[1])
        )

        pose.linear.z = (
            current_z
            + float(correction_mm[2])
        )

        pose.angular.x = current_rx
        pose.angular.y = current_ry
        pose.angular.z = current_rz

        return pose

    def log_center_alignment_target(
        self,
        pose
    ):
        """
        顯示最後計算出的置中目標位置。
        """

        self.get_logger().info(
            f'Alignment target: '
            f'x={pose.linear.x:.3f}, '
            f'y={pose.linear.y:.3f}, '
            f'z={pose.linear.z:.3f}, '
            f'rx={pose.angular.x:.3f}, '
            f'ry={pose.angular.y:.3f}, '
            f'rz={pose.angular.z:.3f}'
        )