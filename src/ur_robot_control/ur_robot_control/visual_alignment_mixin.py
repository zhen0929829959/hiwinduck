import math

import numpy as np
from geometry_msgs.msg import Twist


class VisualAlignmentMixin:
    """
    負責孔中心與插頭中心的視覺微調計算。

    只負責：
    1. 讀取孔中心與插頭中心
    2. 計算像素誤差
    3. 將像素誤差轉成 mm
    4. 限制單次修正量
    5. 建立新的手臂目標 Pose

    不負責：
    1. 訂閱 YOLO
    2. 發送 HIWIN 指令
    3. 切換狀態
    """

    def calculate_visual_alignment_pose(
        self,
        current_pose
    ):
        """
        成功：
            回傳 Twist

        已經對準：
            回傳 False

        資料錯誤：
            回傳 None
        """

        if self.latest_hole_center_px is None:
            self.get_logger().error(
                'Hole center pixel is missing'
            )
            return None

        if current_pose is None:
            self.get_logger().error(
                'Current robot pose is missing'
            )
            return None

        hole_u, hole_v = (
            self.latest_hole_center_px
        )

        # 插頭中心先固定寫死
        # 之後再依照實際畫面調整
        plug_u = self.PLUG_CENTER_U
        plug_v = self.PLUG_CENTER_V

        # 孔中心減掉插頭中心
        error_u = float(hole_u - plug_u)
        error_v = float(hole_v - plug_v)

        self.get_logger().info(
            f'Visual alignment error: '
            f'u={error_u:.2f}px, '
            f'v={error_v:.2f}px'
        )

        # 兩方向都在門檻內，代表已對準
        if (
            abs(error_u)
            <= self.VISUAL_ALIGN_THRESHOLD_PX
            and
            abs(error_v)
            <= self.VISUAL_ALIGN_THRESHOLD_PX
        ):
            self.get_logger().info(
                'Plug and hole are aligned'
            )

            return False

        # ----------------------------------------------------
        # Pixel 轉成相機座標 mm
        # ----------------------------------------------------

        dx_camera_mm = (
            error_u
            * self.VISUAL_ALIGN_DEPTH_M
            / self.CAMERA_FX
            * 1000.0
        )

        dy_camera_mm = (
            error_v
            * self.VISUAL_ALIGN_DEPTH_M
            / self.CAMERA_FY
            * 1000.0
        )

        # 調整實際移動方向
        dx_camera_mm *= (
            self.VISUAL_ALIGN_X_SIGN
        )

        dy_camera_mm *= (
            self.VISUAL_ALIGN_Y_SIGN
        )

        # 比例控制，不一次移完整誤差
        dx_camera_mm *= self.VISUAL_ALIGN_KP
        dy_camera_mm *= self.VISUAL_ALIGN_KP

        correction_camera_mm = np.array(
            [
                dx_camera_mm,
                dy_camera_mm,
                0.0
            ],
            dtype=float
        )

        # ----------------------------------------------------
        # 相機座標轉成 Base 座標
        # ----------------------------------------------------

        if self.latest_r_base_camera is None:
            self.get_logger().error(
                'R_base_camera is missing'
            )
            return None

        r_base_camera = np.asarray(
            self.latest_r_base_camera,
            dtype=float
        )

        if r_base_camera.shape != (3, 3):
            self.get_logger().error(
                'R_base_camera must be 3x3'
            )
            return None

        correction_base_mm = (
            r_base_camera
            @ correction_camera_mm
        )

        # 微調階段只移動 Base XY
        correction_base_mm[2] = 0.0

        correction_base_mm = (
            self.limit_visual_alignment_correction(
                correction_base_mm
            )
        )

        if correction_base_mm is None:
            return None

        self.get_logger().info(
            f'Visual Base correction: '
            f'dx={correction_base_mm[0]:.3f}mm, '
            f'dy={correction_base_mm[1]:.3f}mm'
        )

        return self.create_visual_alignment_pose(
            current_pose=current_pose,
            correction_mm=correction_base_mm
        )

    def limit_visual_alignment_correction(
        self,
        correction_mm
    ):
        """
        限制單次最大 XY 修正距離。
        """

        xy_norm = float(
            math.hypot(
                correction_mm[0],
                correction_mm[1]
            )
        )

        if xy_norm <= 0.0:
            self.get_logger().error(
                'Visual correction is zero'
            )
            return None

        if (
            xy_norm
            <= self.MAX_VISUAL_ALIGN_STEP_MM
        ):
            return correction_mm

        scale = (
            self.MAX_VISUAL_ALIGN_STEP_MM
            / xy_norm
        )

        limited = correction_mm.copy()

        limited[0] *= scale
        limited[1] *= scale
        limited[2] = 0.0

        self.get_logger().warning(
            f'Visual correction limited to '
            f'{self.MAX_VISUAL_ALIGN_STEP_MM:.3f} mm'
        )

        return limited

    def create_visual_alignment_pose(
        self,
        current_pose,
        correction_mm
    ):
        """
        Visual fine alignment：

        X、Y：根據視覺誤差修正
        Z、Rx、Ry、Rz：固定為剛進入
        VISUAL_FINE_ALIGN 時的姿態
        """

        if self.visual_align_reference_pose is None:
            self.get_logger().error(
                'Visual align reference pose is missing'
            )
            return None

        current_x = current_pose[0]
        current_y = current_pose[1]

        fixed_z = self.visual_align_reference_pose[2]
        fixed_rx = self.visual_align_reference_pose[3]
        fixed_ry = self.visual_align_reference_pose[4]
        fixed_rz = self.visual_align_reference_pose[5]

        pose = Twist()

        # 只有 XY 可以改
        pose.linear.x = (
            current_x
            + float(correction_mm[0])
        )

        pose.linear.y = (
            current_y
            + float(correction_mm[1])
        )

        # Z 鎖死
        pose.linear.z = float(fixed_z)

        # 姿態全部鎖死
        pose.angular.x = float(fixed_rx)
        pose.angular.y = float(fixed_ry)
        pose.angular.z = float(fixed_rz)

        return pose