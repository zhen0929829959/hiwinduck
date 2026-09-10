import numpy as np

from geometry_msgs.msg import Twist


class CenterAlignmentMixin:
    """
    負責 AprilTag 畫面置中的計算。

    ALIGN_APRILTAG_CENTER 階段：
    1. 只修正 X / Y
    2. Z 保持不變
    3. RX / RY / RZ 保持不變

    AprilTag 的最終高度，
    交給 strategy 在真正置中後重新決定。
    """

    def calculate_center_alignment_pose(
        self,
        current_pose
    ):
        """
        根據 AprilTag 的修正量，
        計算手臂置中後的目標 Pose。

        只修正 XY。

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

        # ============================================================
        # AprilTag 畫面置中只修 XY
        #
        # 不讓 PnP Z 的誤差造成手臂上下移動。
        # Z 高度等真正置中之後再重新計算。
        # ============================================================

        correction_mm[2] = 0.0

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
            f'XY Base correction: '
            f'dx={correction_mm[0]:.3f}mm, '
            f'dy={correction_mm[1]:.3f}mm, '
            f'dz=0.000mm, '
            f'norm={correction_norm:.3f}mm'
        )

        if (
            correction_norm
            < self.MIN_CENTER_CORRECTION_MM
        ):
            self.get_logger().error(
                f'XY correction is too small '
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
        限制單次最大 XY 修正距離。
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

        # 確保 Z 永遠不動
        limited_correction_mm[2] = 0.0

        limited_norm = float(
            np.linalg.norm(
                limited_correction_mm
            )
        )

        self.get_logger().warning(
            f'XY correction limited to '
            f'{limited_norm:.3f} mm'
        )

        return limited_correction_mm

    def create_center_alignment_pose(
        self,
        current_pose,
        correction_mm
    ):
        """
        AprilTag 畫面置中。

        只修改：
            X
            Y

        完全保持：
            Z
            RX
            RY
            RZ
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

        # X 修正
        pose.linear.x = (
            current_x
            + float(correction_mm[0])
        )

        # Y 修正
        pose.linear.y = (
            current_y
            + float(correction_mm[1])
        )

        # Z 完全不動
        pose.linear.z = current_z

        # 姿態完全不動
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