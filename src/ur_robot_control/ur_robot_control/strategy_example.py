

#!/usr/bin/env python3

import math
import time
from enum import Enum
from threading import Thread

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool, String

from ur_robot_control.apriltag_sampling_mixin import AprilTagSamplingMixin
from ur_robot_control.center_alignment_mixin import CenterAlignmentMixin
from ur_robot_control.force_insertion_mixin import ForceInsertionMixin
from ur_robot_control.rj45_target_mixin import Rj45TargetMixin
from ur_robot_control.strategy_motion_mixin import StrategyMotionMixin
from ur_robot_control.tool_pose_mixin import ToolPoseMixin
from ur_robot_control.ur_robot_mixin import UrRobotMixin
from ur_robot_control.visual_alignment_mixin import VisualAlignmentMixin
from ur_robot_control.yolo_detection_mixin import YoloDetectionMixin


# ============================================================
# UR 基本設定
# ============================================================

DEFAULT_VELOCITY = 30
DEFAULT_ACCELERATION = 30


# ============================================================
# AprilTag 取樣設定
# ============================================================

DISCARD_SAMPLE_COUNT = 10
MEDIAN_SAMPLE_COUNT = 5
TAG_TIMEOUT_SEC = 20.0

APRILTAG_TOPIC = '/camera_left/apriltag/pose_base'
YOLO_DETECTIONS_BASE_TOPIC = '/yolo/detections_base'

TARGET_RJ45_TRACK_KEY = 'RJ45_0'


# ============================================================
# AprilTag 置中設定
# ============================================================

MAX_CENTER_ALIGN_COUNT = 5

CENTER_ALIGN_VELOCITY = 10
CENTER_ALIGN_ACCELERATION = 10

MAX_CENTER_CORRECTION_MM = 20.0
MIN_CENTER_CORRECTION_MM = 0.5

DEFAULT_CENTER_THRESHOLD_PX = 15.0
CENTER_CORRECTION_SIGN = 1.0


# ============================================================
# YOLO 視覺微調設定
# ============================================================

PLUG_CENTER_U = 984.0
PLUG_CENTER_V = 718.0

CAMERA_FX = 1362.7784437304256
CAMERA_FY = 1361.8357978878923

VISUAL_ALIGN_DEPTH_M = 0.11
VISUAL_ALIGN_KP = 2.3
VISUAL_ALIGN_THRESHOLD_PX = 5.0

MAX_VISUAL_ALIGN_STEP_MM = 3.0

VISUAL_ALIGN_X_SIGN = 1.0
VISUAL_ALIGN_Y_SIGN = 1.0

MAX_VISUAL_ALIGN_COUNT = 25


# ============================================================
# 插入重試設定
# ============================================================

MAX_INSERTION_RETRY_COUNT = 3

# ============================================================
# 力覺搜尋設定（實驗功能，可一鍵關閉）
# ============================================================

# False = 完全維持原本流程：碰撞後回 PHOTO_POSE 重新定位
# True  = 碰撞後先在原始插入點附近做離散環狀搜尋，再重新插入
ENABLE_FORCE_SEARCH = True

# 只改 XY，不改 Z / 姿態。單位：mm / deg
FORCE_SEARCH_RADIUS_STEP_MM = 2.0
FORCE_SEARCH_MAX_RADIUS_MM = 5.0
FORCE_SEARCH_ANGLE_STEP_DEG = 45.0
FORCE_SEARCH_VELOCITY = 2
FORCE_SEARCH_ACCELERATION = 2


# ============================================================
# PHOTO_POSE 找不到 AprilTag 時的搜尋設定
# ============================================================

# 只用於 URSim / mock 驗證。
# 真機不要直接自動做 360 度搜尋動作。
APRILTAG_SEARCH_SIMULATION_ONLY = True

# 把第六軸一圈拆成 4 次，每次 90 度；
# 每轉一次就重新等待 AprilTag，而不是一次轉完整圈後才看。
APRILTAG_SEARCH_STEP_DEG = 90.0
APRILTAG_SEARCH_MAX_STEPS = 4


# ============================================================
# 手臂位置設定
# ============================================================

PHOTO_POSE = [
    72.18,
    -109.52,
    64.26,
    -80.53,
    -78.70,
    -105.88,
]

TAG_APPROACH_Z_MM = 200.0
RJ45_APPROACH_Z_MM = 20.0

YOLO_OFFSET_X_MM = -4.0
YOLO_OFFSET_Y_MM = -4.0

TARGET_RZ_OFFSET = -180.0
CAMERA_RX_OFFSET_DEG = 30.0


# ============================================================
# 狀態機
# ============================================================

class States(Enum):
    INIT = 0
    MOVE_TO_PHOTO_POSE = 1
    WAIT_FIRST_APRILTAG = 2
    MOVE_ABOVE_APRILTAG = 3
    MOVE_ABOVE_APRILTAG_TEST = 18
    PREPARE_SECOND_LOCALIZATION = 4
    WAIT_SECOND_APRILTAG = 5
    ALIGN_APRILTAG_CENTER = 6
    MOVE_ABOVE_BOARD_CENTER = 7
    WAIT_YOLO_DETECTION = 8
    MOVE_ABOVE_YOLO_TARGET = 9
    VISUAL_FINE_ALIGN = 10
    CHECK_POSE = 11
    PREPARE_INSERTION = 12
    RUN_INSERTION = 13
    CHECK_INSERTION_RESULT = 14
    SEARCH_INSERTION_POSITION = 17

    # PHOTO_POSE 看不到 AprilTag 時，第六軸搜尋
    SEARCH_FIRST_APRILTAG = 15

    FINISH = 16


class ExampleStrategy(
    AprilTagSamplingMixin,
    UrRobotMixin,
    ToolPoseMixin,
    CenterAlignmentMixin,
    Rj45TargetMixin,
    VisualAlignmentMixin,
    YoloDetectionMixin,
    StrategyMotionMixin,
    ForceInsertionMixin,
    Node
):
    TARGET_RJ45_TRACK_KEY = TARGET_RJ45_TRACK_KEY

    CENTER_CORRECTION_SIGN = CENTER_CORRECTION_SIGN
    MIN_CENTER_CORRECTION_MM = MIN_CENTER_CORRECTION_MM
    MAX_CENTER_CORRECTION_MM = MAX_CENTER_CORRECTION_MM

    RJ45_APPROACH_Z_MM = RJ45_APPROACH_Z_MM
    TARGET_RZ_OFFSET = TARGET_RZ_OFFSET

    PLUG_CENTER_U = PLUG_CENTER_U
    PLUG_CENTER_V = PLUG_CENTER_V

    CAMERA_FX = CAMERA_FX
    CAMERA_FY = CAMERA_FY

    VISUAL_ALIGN_DEPTH_M = VISUAL_ALIGN_DEPTH_M
    VISUAL_ALIGN_KP = VISUAL_ALIGN_KP
    VISUAL_ALIGN_THRESHOLD_PX = VISUAL_ALIGN_THRESHOLD_PX
    MAX_VISUAL_ALIGN_STEP_MM = MAX_VISUAL_ALIGN_STEP_MM
    VISUAL_ALIGN_X_SIGN = VISUAL_ALIGN_X_SIGN
    VISUAL_ALIGN_Y_SIGN = VISUAL_ALIGN_Y_SIGN

    def __init__(self):
        super().__init__(
            'example_strategy'
        )

        self.init_ur_robot()
        self.init_force_insertion()

        self.tool_pose_pub = self.create_publisher(
            String,
            '/hiwin/tool_pose',
            10
        )

        self.freeze_pnp_z_pub = self.create_publisher(
            Bool,
            '/apriltag/freeze_pnp_z',
            10
        )

        self.tag_sub = self.create_subscription(
            String,
            APRILTAG_TOPIC,
            self.tag_pose_callback,
            10
        )

        self.yolo_detection_sub = self.create_subscription(
            String,
            YOLO_DETECTIONS_BASE_TOPIC,
            self.yolo_detections_base_callback,
            10
        )

        self.waiting_for_tag = False
        self.has_tag = False
        self.received_sample_count = 0

        self.tag_samples = []
        self.rj45_samples = []
        self.quat_samples = []

        self.center_error_samples = []
        self.center_correction_samples = []
        self.center_threshold_samples = []

        self.latest_tag_position_m = None
        self.latest_tag_rj45_position_m = None
        self.latest_board_center_position_m = None
        self.latest_tag_quat = None
        self.latest_camera_aligned_tool7_pose_base = None
        self.tag_above_z_mm = None

        self.latest_yolo_detection_base_m = None
        self.latest_yolo_detection_info = None
        self.has_yolo_detection = False

        self.latest_centered = False
        self.latest_center_error_px = None
        self.latest_center_correction_base_m = None
        self.latest_center_threshold_px = (
            DEFAULT_CENTER_THRESHOLD_PX
        )

        self.photo_orientation_deg = None
        self.center_align_count = 0

        self.latest_hole_center_px = None
        self.latest_r_base_camera = None
        self.visual_align_count = 0
        self.visual_align_reference_pose = None

        # ============================================================
        # 插入重試
        # ============================================================

        self.insertion_retry_count = 0
        self.max_insertion_retry_count = (
            MAX_INSERTION_RETRY_COUNT
        )

        # 力覺搜尋：origin 永遠保存「這一輪視覺對位完成後」的中心點，
        # 每次搜尋都相對這個中心算，避免位置逐次漂移。
        self.force_search_origin_pose = None
        self.force_search_attempt_count = 0

        # ============================================================
        # PHOTO_POSE AprilTag 搜尋
        # ============================================================

        self.apriltag_search_step = 0

        self.get_logger().info(
            'Example strategy node started'
        )

    # ================================================================
    # Helpers
    # ================================================================

    def freeze_current_pnp_z(self):
        msg = Bool()
        msg.data = True

        self.freeze_pnp_z_pub.publish(
            msg
        )

        self.get_logger().info(
            'Published freeze PnP Z command'
        )

    def reset_retry_data(self):
        """
        碰撞後，準備回 PHOTO_POSE 重新跑整套流程前，
        清掉上一輪的 AprilTag / YOLO / visual alignment 資料。
        """

        self.reset_tag_sampling()

        self.waiting_for_tag = False

        # AprilTag / board
        self.latest_tag_position_m = None
        self.latest_tag_rj45_position_m = None
        self.latest_board_center_position_m = None
        self.latest_tag_quat = None

        self.latest_centered = False
        self.latest_center_error_px = None
        self.latest_center_correction_base_m = None
        self.latest_center_threshold_px = (
            DEFAULT_CENTER_THRESHOLD_PX
        )

        self.center_align_count = 0

        # YOLO
        self.latest_yolo_detection_base_m = None
        self.latest_yolo_detection_info = None
        self.has_yolo_detection = False

        # visual alignment
        self.latest_hole_center_px = None
        self.latest_r_base_camera = None
        self.visual_align_count = 0
        self.visual_align_reference_pose = None

        # insertion / force search 也不要沿用
        self.insertion_start_pose = None
        self.force_search_origin_pose = None
        self.insertion_reference_z_mm = None
        self.force_search_attempt_count = 0

        self.get_logger().info(
            'Retry data reset complete'
        )

    def get_next_force_search_offset(self):
        """
        產生下一個離散環狀搜尋點。

        第一次碰撞後：r=0.5 mm，角度 0/45/90/...
        繞完一圈後：r=1.0 mm，再繞一圈。

        回傳：dx_mm, dy_mm, radius_mm, angle_deg
        搜尋範圍用完則回傳 None。
        """
        points_per_ring = max(1, int(round(360.0 / FORCE_SEARCH_ANGLE_STEP_DEG)))
        ring_index = self.force_search_attempt_count // points_per_ring + 1
        point_index = self.force_search_attempt_count % points_per_ring
        radius_mm = ring_index * FORCE_SEARCH_RADIUS_STEP_MM

        if radius_mm > FORCE_SEARCH_MAX_RADIUS_MM + 1e-9:
            return None

        angle_deg = point_index * FORCE_SEARCH_ANGLE_STEP_DEG
        angle_rad = math.radians(angle_deg)
        dx_mm = radius_mm * math.cos(angle_rad)
        dy_mm = radius_mm * math.sin(angle_rad)
        return dx_mm, dy_mm, radius_mm, angle_deg

    def move_to_next_force_search_position(self):
        """
        從原始視覺對位中心，移到下一個 XY 搜尋點。
        不改 Z，也不改 Rx/Ry/Rz。
        """
        if self.force_search_origin_pose is None:
            self.get_logger().error('Force-search origin pose is missing')
            return 'ERROR'

        offset = self.get_next_force_search_offset()
        if offset is None:
            return 'EXHAUSTED'

        dx_mm, dy_mm, radius_mm, angle_deg = offset

        current_pose = self.get_current_robot_pose()

        if current_pose is None:
            self.get_logger().error('Cannot read current pose before force-search move')
            return 'ERROR'

        target = list(current_pose)
        target[0] = self.force_search_origin_pose[0] + dx_mm
        target[1] = self.force_search_origin_pose[1] + dy_mm

        pose = self.create_pose(target[0], target[1], target[2], target[3], target[4], target[5])

        self.get_logger().warning(
            f'Force-search point {self.force_search_attempt_count + 1}: '
            f'r={radius_mm:.2f} mm, angle={angle_deg:.1f} deg, '
            f'dx={dx_mm:+.3f} mm, dy={dy_mm:+.3f} mm'
        )

        if not self.move_pose_lin(pose, FORCE_SEARCH_VELOCITY, FORCE_SEARCH_ACCELERATION, holding=True):
            self.get_logger().error('Move to force-search point failed')
            return 'ERROR'

        # 下一次插入要從這個新位置開始。
        self.insertion_start_pose = list(target)
        self.force_search_attempt_count += 1
        time.sleep(0.3)
        return 'OK'

    def reset_first_apriltag_search(self):
        self.apriltag_search_step = 0

    def search_first_apriltag_mock(self):
        """
        PHOTO_POSE 找不到 AprilTag 時的搜尋邏輯。

        概念：
            PHOTO_POSE
                ↓
            等 AprilTag timeout
                ↓
            第六軸 +90 deg
                ↓
            再偵測
                ↓
            最多 4 次 = 一圈

        這裡刻意只允許 mock / URSim。
        真機自動大角度旋轉要另外確認關節限制、治具與線材干涉。
        """

        if self.apriltag_search_step >= APRILTAG_SEARCH_MAX_STEPS:
            self.get_logger().error(
                'AprilTag search completed one full turn '
                'but tag is still not detected'
            )
            return False

        self.apriltag_search_step += 1

        search_angle = (
            APRILTAG_SEARCH_STEP_DEG
            * self.apriltag_search_step
        )

        target_joints = list(PHOTO_POSE)
        target_joints[5] = (
            PHOTO_POSE[5] + search_angle
        )

        self.get_logger().warning(
            'PHOTO_POSE AprilTag not found -> '
            f'search step '
            f'{self.apriltag_search_step}/'
            f'{APRILTAG_SEARCH_MAX_STEPS}, '
            f'J6 target offset={search_angle:.1f} deg'
        )

        if not APRILTAG_SEARCH_SIMULATION_ONLY:
            self.get_logger().error(
                'Real-hardware automatic J6 search is disabled '
                'in this example'
            )
            return False

        # ------------------------------------------------------------
        # URSim / mock:
        # 若你的 mock 環境允許真正規劃，可在受控模擬環境裡
        # 用現有 move_joints() 驗證 target_joints。
        #
        # 這個版本不直接對真機送這個搜尋動作。
        # ------------------------------------------------------------

        self.get_logger().warning(
            f'URSim/mock search target joints = {target_joints}'
        )

        # 搜尋後重新準備收 AprilTag
        self.reset_tag_sampling()
        self.waiting_for_tag = True

        if not self.update_and_publish_tool_pose():
            self.waiting_for_tag = False
            return False

        return True

    # ================================================================
    # State machine
    # ================================================================

    def _state_machine(self, state):

        # ============================================================
        # INIT
        # ============================================================

        if state == States.INIT:
            self.get_logger().info(
                'INIT'
            )

            self.reset_tag_sampling()

            self.photo_orientation_deg = None
            self.center_align_count = 0

            self.latest_yolo_detection_base_m = None
            self.latest_yolo_detection_info = None
            self.has_yolo_detection = False

            self.insertion_retry_count = 0
            self.reset_first_apriltag_search()

            return States.MOVE_TO_PHOTO_POSE
            # return States.WAIT_FIRST_APRILTAG


        # ============================================================
        # MOVE TO PHOTO POSE
        # ============================================================

        if state == States.MOVE_TO_PHOTO_POSE:
            self.get_logger().info(
                'MOVE_TO_PHOTO_POSE'
            )

            # 每次真的回 PHOTO_POSE，都重新開始搜尋計數
            self.reset_first_apriltag_search()

            if not self.move_joints(
                PHOTO_POSE,
                DEFAULT_VELOCITY,
                DEFAULT_ACCELERATION,
                holding=True
            ):
                self.get_logger().error(
                    'Move to photo pose failed'
                )

                return States.FINISH

            time.sleep(1.0)

            self.reset_tag_sampling()
            self.waiting_for_tag = True

            if not self.update_and_publish_tool_pose(
                save_photo_orientation=True
            ):
                self.waiting_for_tag = False

                return States.FINISH

            return States.WAIT_FIRST_APRILTAG

        # ============================================================
        # WAIT FIRST APRILTAG
        # ============================================================

        if state == States.WAIT_FIRST_APRILTAG:
            if not self.wait_for_tag_result(
                'WAIT_FIRST_APRILTAG'
            ):
                self.get_logger().warning(
                    'First AprilTag timeout at PHOTO_POSE -> '
                    'start J6 search'
                )

                return States.SEARCH_FIRST_APRILTAG

            self.reset_first_apriltag_search()

            return States.MOVE_ABOVE_APRILTAG
            # return States.MOVE_ABOVE_APRILTAG_TEST

        # ============================================================
        # SEARCH FIRST APRILTAG
        # ============================================================

        if state == States.SEARCH_FIRST_APRILTAG:
            self.get_logger().warning(
                'SEARCH_FIRST_APRILTAG'
            )

            if not self.search_first_apriltag_mock():
                return States.FINISH

            return States.WAIT_FIRST_APRILTAG


        # ============================================================
        # MOVE ABOVE APRILTAG
        # ============================================================

        if state == States.MOVE_ABOVE_APRILTAG:
            self.get_logger().info(
                'MOVE_ABOVE_APRILTAG'
            )

            if self.latest_tag_position_m is None:
                self.get_logger().error(
                    'First AprilTag position is missing'
                )

                return States.FINISH

            if (
                self.latest_camera_aligned_tool7_pose_base
                is None
            ):
                self.get_logger().error(
                    'Camera-aligned Tool7 pose is missing'
                )

                return States.FINISH

            tag_x, tag_y, tag_z = (
                self.latest_tag_position_m
            )

            rx, ry, rz = (
                self.latest_camera_aligned_tool7_pose_base[
                    'euler_deg'
                ]
            )

            pose = self.create_pose(
                tag_x * 1000.0,
                tag_y * 1000.0,
                tag_z * 1000.0
                + TAG_APPROACH_Z_MM,
                rx,
                ry,
                rz - TARGET_RZ_OFFSET
            )

            if not self.move_pose(
                pose,
                DEFAULT_VELOCITY,
                DEFAULT_ACCELERATION,
                holding=True
            ):
                self.get_logger().error(
                    'Move above AprilTag failed'
                )

                return States.FINISH

            time.sleep(1.0)

            current_pose = (
                self.get_current_robot_pose()
            )

            if current_pose is None:
                self.get_logger().error(
                    'Cannot read pose above AprilTag'
                )

                return States.FINISH

            # ============================================================
            # 這裡先不要保存最終 Z
            #
            # 因為此時 AprilTag 還沒有真正置中。
            # 最終固定高度會等 AprilTag 中心對準後，
            # 使用最新的 Tag Z 再重新建立。
            # ============================================================
            self.freeze_current_pnp_z()
            self.tag_above_z_mm = None

            self.center_align_count = 0

            # return States.PREPARE_SECOND_LOCALIZATION
            return States.MOVE_ABOVE_BOARD_CENTER

        # ============================================================
        # PREPARE SECOND LOCALIZATION
        # ============================================================

        if state == States.PREPARE_SECOND_LOCALIZATION:
            self.get_logger().info(
                'PREPARE_SECOND_LOCALIZATION'
            )

            self.reset_tag_sampling()
            self.waiting_for_tag = True

            if not self.update_and_publish_tool_pose():
                self.waiting_for_tag = False

                return States.FINISH

            return States.WAIT_SECOND_APRILTAG

        # ============================================================
        # WAIT SECOND APRILTAG
        # ============================================================

        if state == States.WAIT_SECOND_APRILTAG:

            if not self.wait_for_tag_result(
                'WAIT_SECOND_APRILTAG'
            ):
                self.get_logger().warning(
                    'Second AprilTag timeout -> '
                    'return to PHOTO_POSE and restart localization'
                )

                self.reset_retry_data()

                return States.MOVE_TO_PHOTO_POSE

            if self.latest_center_error_px is None:
                self.get_logger().error(
                    'Center error data is missing'
                )
                return States.FINISH

            # ========================================================
            # AprilTag 已經真正置中
            # ========================================================

            if self.latest_centered:

                if self.latest_tag_position_m is None:
                    self.get_logger().error(
                        'Centered AprilTag position is missing'
                    )

                    return States.FINISH

                tag_x, tag_y, tag_z = (
                    self.latest_tag_position_m
                )

                # ----------------------------------------------------
                # 現在 AprilTag 已經在畫面中心。
                #
                # 使用「此時重新量到」的 Tag Z，
                # 建立固定的拍攝高度。
                #
                # 之後移到 Board Center 時，
                # 都使用同一個 Z。
                # ----------------------------------------------------

                self.tag_above_z_mm = (
                    tag_z * 1000.0
                    + TAG_APPROACH_Z_MM
                )

                error_u, error_v = (
                    self.latest_center_error_px
                )

                self.get_logger().info(
                    'AprilTag centered'
                )

                self.get_logger().info(
                    f'Final center error: '
                    f'u={error_u:.2f}px, '
                    f'v={error_v:.2f}px'
                )

                self.get_logger().info(
                    f'Centered Tag position: '
                    f'x={tag_x * 1000.0:.3f}mm, '
                    f'y={tag_y * 1000.0:.3f}mm, '
                    f'z={tag_z * 1000.0:.3f}mm'
                )

                self.get_logger().info(
                    f'Fixed board observation Z: '
                    f'{self.tag_above_z_mm:.3f}mm '
                    f'(Tag Z + {TAG_APPROACH_Z_MM:.3f}mm)'
                )

                # ----------------------------------------------------
                # 如果之後真的需要固定 PnP Z，
                # 可以在「已置中」之後才 freeze。
                #
                # 不要在還沒置中之前 freeze。
                # ----------------------------------------------------

                self.freeze_current_pnp_z()
                time.sleep(0.5)

                return States.MOVE_ABOVE_BOARD_CENTER

            # ========================================================
            # 還沒有置中
            # ========================================================

            if (
                self.center_align_count
                >= MAX_CENTER_ALIGN_COUNT
            ):
                self.get_logger().error(
                    'AprilTag center alignment '
                    'exceeded maximum count'
                )

                return States.FINISH

            self.get_logger().info(
                f'AprilTag not centered -> '
                f'run XY alignment '
                f'{self.center_align_count + 1}/'
                f'{MAX_CENTER_ALIGN_COUNT}'
            )

            return States.ALIGN_APRILTAG_CENTER

        # ============================================================
        # ALIGN APRILTAG CENTER
        # ============================================================

        if state == States.ALIGN_APRILTAG_CENTER:
            self.get_logger().info(
                'ALIGN_APRILTAG_CENTER'
            )

            current_pose = (
                self.get_current_robot_pose()
            )

            if current_pose is None:
                return States.FINISH

            pose = (
                self.calculate_center_alignment_pose(
                    current_pose=current_pose
                )
            )

            if pose is None:
                return States.FINISH

            if not self.move_pose(
                pose,
                CENTER_ALIGN_VELOCITY,
                CENTER_ALIGN_ACCELERATION,
                holding=True
            ):
                self.get_logger().error(
                    'Center alignment movement failed'
                )

                return States.FINISH

            self.center_align_count += 1

            time.sleep(1.0)

            return States.PREPARE_SECOND_LOCALIZATION

        # ============================================================
        # MOVE ABOVE BOARD CENTER
        # ============================================================

        if state == States.MOVE_ABOVE_BOARD_CENTER:
            self.get_logger().info(
                'MOVE_ABOVE_BOARD_CENTER'
            )

            if (
                self.latest_board_center_position_m
                is None
            ):
                self.get_logger().error(
                    'Board center position is missing'
                )

                return States.FINISH

            if (
                self.latest_camera_aligned_tool7_pose_base
                is None
            ):
                self.get_logger().error(
                    'Camera-aligned Tool7 pose is missing'
                )

                return States.FINISH

            # if self.tag_above_z_mm is None:
            #     self.get_logger().error(
            #         'Fixed board observation Z is missing'
            #     )

                return States.FINISH

            board_x, board_y, board_z = (
                self.latest_board_center_position_m
            )

            rx, ry, rz = (
                self.latest_camera_aligned_tool7_pose_base[
                    'euler_deg'
                ]
            )

            self.get_logger().info(
                f'Move to board center: '
                f'x={board_x * 1000.0:.3f}mm, '
                f'y={board_y * 1000.0:.3f}mm, '
                f'z={board_z * 1000.0 + TAG_APPROACH_Z_MM:.3f}mm'
            )

            pose = self.create_pose(
                board_x * 1000.0,
                board_y * 1000.0,
                board_z * 1000.0
                + TAG_APPROACH_Z_MM,
                rx,
                ry,
                rz - TARGET_RZ_OFFSET
            )

            if not self.move_pose(
                pose,
                DEFAULT_VELOCITY,
                DEFAULT_ACCELERATION,
                holding=True
            ):
                self.get_logger().error(
                    'Move above board center failed'
                )

                return States.FINISH

            if not self.update_and_publish_tool_pose():
                return States.FINISH

            time.sleep(1.0)

            self.latest_yolo_detection_base_m = None
            self.latest_yolo_detection_info = None
            self.has_yolo_detection = False

            return States.WAIT_YOLO_DETECTION

        # ============================================================
        # WAIT YOLO DETECTION
        # ============================================================

        if state == States.WAIT_YOLO_DETECTION:
            self.get_logger().info(
                'WAIT_YOLO_DETECTION'
            )

            # time.sleep(1.0)

            start_time = time.time()

            while (
                rclpy.ok()
                and time.time() - start_time
                < 15.0
            ):
                if self.has_yolo_detection:
                    return States.MOVE_ABOVE_YOLO_TARGET

                time.sleep(0.05)

            self.get_logger().error(
                'YOLO detection timeout'
            )

            return States.FINISH

        # ============================================================
        # MOVE ABOVE YOLO TARGET
        # ============================================================

        if state == States.MOVE_ABOVE_YOLO_TARGET:
            self.get_logger().info(
                'MOVE_ABOVE_YOLO_TARGET'
            )

            if (
                self.latest_yolo_detection_base_m
                is None
            ):
                self.get_logger().error(
                    'YOLO target base position is missing'
                )

                return States.FINISH

            if self.latest_tag_quat is None:
                self.get_logger().error(
                    'Second AprilTag orientation is missing'
                )

                return States.FINISH

            target_x, target_y, target_z = (
                self.latest_yolo_detection_base_m
            )

            try:
                rx, ry, rz = (
                    R.from_quat(
                        self.latest_tag_quat
                    ).as_euler(
                        'xyz',
                        degrees=True
                    )
                )

            except Exception as exc:
                self.get_logger().error(
                    f'Quaternion conversion failed: '
                    f'{exc}'
                )

                return States.FINISH

            pose = self.create_pose(
                # target_x * 1000.0+ YOLO_OFFSET_X_MM,
                # target_y * 1000.0+ YOLO_OFFSET_Y_MM,
                target_x * 1000.0,
                target_y * 1000.0,
                target_z * 1000.0
                + RJ45_APPROACH_Z_MM,
                rx,
                ry,
                rz - 90
            )

            if not self.move_pose(
                pose,
                DEFAULT_VELOCITY,
                DEFAULT_ACCELERATION,
                holding=True
            ):
                self.get_logger().error(
                    'Move above YOLO target failed'
                )

                return States.FINISH

            if not self.update_and_publish_tool_pose():
                return States.FINISH

            time.sleep(2.0)

            self.visual_align_count = 0
            self.latest_hole_center_px = None
            self.visual_align_reference_pose = None

            self.latest_yolo_detection_base_m = None
            self.latest_yolo_detection_info = None
            self.has_yolo_detection = False

            return States.VISUAL_FINE_ALIGN


        # ============================================================
        # SECOND YOLO ALIGNMENT
        # ============================================================

        if state == States.VISUAL_FINE_ALIGN:
            self.get_logger().info(
                'SECOND_YOLO_ALIGNMENT'
            )
            # time.sleep(5.0)
            start_time = time.time()

            while (
                rclpy.ok()
                and time.time() - start_time < 15.0
            ):
                if (
                    self.has_yolo_detection
                    and self.latest_yolo_detection_base_m is not None
                ):
                    break

                time.sleep(0.05)

            if self.latest_yolo_detection_base_m is None:
                self.get_logger().error(
                    'Second YOLO detection timeout'
                )

                return States.FINISH

            if self.latest_tag_quat is None:
                self.get_logger().error(
                    'Second AprilTag orientation is missing'
                )

                return States.FINISH

            target_x, target_y, target_z = (
                self.latest_yolo_detection_base_m
            )

            try:
                rx, ry, rz = (
                    R.from_quat(
                        self.latest_tag_quat
                    ).as_euler(
                        'xyz',
                        degrees=True
                    )
                )

            except Exception as exc:
                self.get_logger().error(
                    f'Quaternion conversion failed: {exc}'
                )

                return States.FINISH

            self.get_logger().info(
                f'Second YOLO target: '
                f'x={target_x * 1000.0:.3f}mm, '
                f'y={target_y * 1000.0:.3f}mm, '
                f'z={target_z * 1000.0:.3f}mm'
            )

            pose = self.create_pose(
                target_x * 1000.0 + YOLO_OFFSET_X_MM,
                target_y * 1000.0 + YOLO_OFFSET_Y_MM,
                target_z * 1000.0 + RJ45_APPROACH_Z_MM,
                rx,
                ry,
                rz - 90
            )

            if not self.move_pose(
                pose,
                DEFAULT_VELOCITY,
                DEFAULT_ACCELERATION,
                holding=True
            ):
                self.get_logger().error(
                    'Second YOLO alignment movement failed'
                )

                return States.FINISH

            if not self.update_and_publish_tool_pose():
                return States.FINISH

            time.sleep(1.0)

            return States.CHECK_POSE


        # # ============================================================
        # # VISUAL FINE ALIGN
        # # ============================================================

        # if state == States.VISUAL_FINE_ALIGN:
        #     self.get_logger().info(
        #         'VISUAL_FINE_ALIGN'
        #     )

        #     if (
        #         self.visual_align_count
        #         >= MAX_VISUAL_ALIGN_COUNT
        #     ):
        #         self.get_logger().error(
        #             'Visual alignment exceeded '
        #             'maximum count'
        #         )

        #         return States.FINISH

        #     self.latest_hole_center_px = None

        #     start_time = time.time()

        #     while (
        #         rclpy.ok()
        #         and time.time() - start_time
        #         < 5.0
        #     ):
        #         if (
        #             self.latest_hole_center_px
        #             is not None
        #         ):
        #             break

        #         time.sleep(0.05)

        #     if self.latest_hole_center_px is None:
        #         self.get_logger().error(
        #             'Waiting for new hole center timeout'
        #         )

        #         return States.FINISH

        #     current_pose = (
        #         self.get_current_robot_pose()
        #     )

        #     if current_pose is None:
        #         return States.FINISH

        #     if self.visual_align_reference_pose is None:
        #         self.visual_align_reference_pose = list(
        #             current_pose
        #         )

        #         self.get_logger().info(
        #             'Visual align reference pose saved: '
        #             f'X={self.visual_align_reference_pose[0]:.3f}, '
        #             f'Y={self.visual_align_reference_pose[1]:.3f}, '
        #             f'Z={self.visual_align_reference_pose[2]:.3f}, '
        #             f'Rx={self.visual_align_reference_pose[3]:.3f}, '
        #             f'Ry={self.visual_align_reference_pose[4]:.3f}, '
        #             f'Rz={self.visual_align_reference_pose[5]:.3f}'
        #         )

        #     pose = (
        #         self.calculate_visual_alignment_pose(
        #             current_pose=current_pose
        #         )
        #     )

        #     if pose is False:
        #         return States.CHECK_POSE

        #     if pose is None:
        #         return States.FINISH

        #     if not self.move_pose_lin(
        #         pose,
        #         5,
        #         5,
        #         holding=True
        #     ):
        #         self.get_logger().error(
        #             'Visual alignment movement failed'
        #         )

        #         return States.FINISH

        #     self.visual_align_count += 1

        #     time.sleep(1.0)

        #     if not self.update_and_publish_tool_pose():
        #         return States.FINISH

        #     time.sleep(0.5)

        #     return States.VISUAL_FINE_ALIGN

        # ============================================================
        # INSERTION
        # ============================================================

        if state == States.CHECK_POSE:
            self.get_logger().info(
                'CHECK_POSE'
            )

            current_pose = (
                self.get_current_robot_pose()
            )

            if current_pose is None:
                self.get_logger().error(
                    'Cannot read final alignment pose'
                )

                return States.FINISH

            self.insertion_start_pose = list(
                current_pose
            )

            # 整輪力覺搜尋的固定深度基準，只在視覺對位完成時設定一次
            self.insertion_reference_z_mm = float(current_pose[2])

            # 每次完成一輪視覺對位，都重新建立力覺搜尋中心。
            self.force_search_origin_pose = list(current_pose)
            self.force_search_attempt_count = 0

            self.get_logger().info(
                f'Final alignment pose: '
                f'{self.insertion_start_pose}'
            )

            return States.PREPARE_INSERTION

        if state == States.PREPARE_INSERTION:
            self.get_logger().info(
                'PREPARE_INSERTION'
            )

            if not self.prepare_force_insertion():
                return States.FINISH

            return States.RUN_INSERTION

        if state == States.RUN_INSERTION:
            self.get_logger().info(
                'RUN_INSERTION'
            )

            if not self.run_force_insertion():
                return States.FINISH

            return States.CHECK_INSERTION_RESULT

        # ============================================================
        # FORCE-GUIDED SEARCH NEXT POSITION
        # ============================================================

        if state == States.SEARCH_INSERTION_POSITION:
            self.get_logger().info('SEARCH_INSERTION_POSITION')

            result = self.move_to_next_force_search_position()

            if result == 'OK':
                return States.PREPARE_INSERTION

            if result == 'EXHAUSTED':
                self.get_logger().warning(
                    'Force-search range exhausted -> fall back to original '
                    'PHOTO_POSE relocalization flow'
                )

                self.insertion_retry_count += 1

                if self.insertion_retry_count > self.max_insertion_retry_count:
                    self.get_logger().error('Insertion retry limit reached')
                    return States.FINISH

                self.reset_retry_data()
                return States.MOVE_TO_PHOTO_POSE

            return States.FINISH

        # ============================================================
        # CHECK INSERTION RESULT
        # ============================================================

        if state == States.CHECK_INSERTION_RESULT:
            self.get_logger().info(
                'CHECK_INSERTION_RESULT'
            )

            ok = self.check_force_insertion_result()

            if not ok:
                self.get_logger().error(
                    'Insertion result handling failed'
                )

                return States.FINISH

            # --------------------------------------------------------
            # SUCCESS
            # --------------------------------------------------------

            if self.insertion_result == 'SUCCESS':
                self.get_logger().info(
                    'Insertion success -> FINISH'
                )

                return States.FINISH

            # --------------------------------------------------------
            # COLLISION
            # --------------------------------------------------------

            if self.insertion_result == 'COLLISION':

                # check_force_insertion_result() 在 COLLISION 時已經先退回
                # 這次的 insertion_start_pose，所以這裡只要決定下一步。
                if ENABLE_FORCE_SEARCH:
                    self.get_logger().warning(
                        'Insertion collision -> try nearby force-search point'
                    )
                    return States.SEARCH_INSERTION_POSITION

                # 關掉 ENABLE_FORCE_SEARCH 時，以下完全走原本流程。
                self.insertion_retry_count += 1

                self.get_logger().warning(
                    f'Insertion collision -> return to PHOTO_POSE. '
                    f'Retry {self.insertion_retry_count}/'
                    f'{self.max_insertion_retry_count}'
                )

                if self.insertion_retry_count > self.max_insertion_retry_count:
                    self.get_logger().error('Insertion retry limit reached')
                    return States.FINISH

                self.reset_retry_data()
                return States.MOVE_TO_PHOTO_POSE

            self.get_logger().error(
                f'Unknown insertion result: '
                f'{self.insertion_result}'
            )

            return States.FINISH

        return States.FINISH

    # ================================================================
    # Main loop
    # ================================================================

    def _main_loop(self):
        state = States.INIT

        while (
            rclpy.ok()
            and state != States.FINISH
        ):
            state = self._state_machine(
                state
            )

        self.get_logger().info(
            'FINISH'
        )

    def start_main_loop_thread(self):
        self.main_loop_thread = Thread(
            target=self._main_loop,
            daemon=True
        )

        self.main_loop_thread.start()


def main(args=None):
    rclpy.init(
        args=args
    )

    strategy = ExampleStrategy()

    strategy.start_main_loop_thread()

    try:
        while (
            rclpy.ok()
            and strategy.main_loop_thread.is_alive()
        ):
            rclpy.spin_once(
                strategy,
                timeout_sec=0.1
            )

    except KeyboardInterrupt:
        strategy.get_logger().warning(
            'Example strategy interrupted'
        )

    finally:
        strategy.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

