#!/usr/bin/env python3

import time
from enum import Enum
from threading import Thread

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from hiwin_interfaces.srv import RobotCommand
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String

from hiwin_example.apriltag_sampling_mixin import AprilTagSamplingMixin
from hiwin_example.hiwin_robot_mixin import HiwinRobotMixin
from hiwin_example.tool_pose_mixin import ToolPoseMixin
from hiwin_example.center_alignment_mixin import CenterAlignmentMixin
from hiwin_example.rj45_target_mixin import Rj45TargetMixin

# ============================================================
# HIWIN 基本設定
# ============================================================

DEFAULT_VELOCITY = 20
DEFAULT_ACCELERATION = 15

ACTIVE_TOOL = 7
ACTIVE_BASE = 0


# ============================================================
# AprilTag 取樣設定
# ============================================================

# 每次開始取樣時，先丟掉前面幾筆不穩定資料
DISCARD_SAMPLE_COUNT = 10

# 正式收集幾筆資料後取中位數
MEDIAN_SAMPLE_COUNT = 20

# 等待 AprilTag 資料的最長時間
TAG_TIMEOUT_SEC = 10.0

APRILTAG_TOPIC = '/apriltag/pose_base'


# ============================================================
# AprilTag 置中設定
# ============================================================

# 最多允許修正幾次
MAX_CENTER_ALIGN_COUNT = 5

# 置中時使用較低速度
CENTER_ALIGN_VELOCITY = 5
CENTER_ALIGN_ACCELERATION = 5

# 單次最多修正 20 mm，避免辨識錯誤造成大幅移動
MAX_CENTER_CORRECTION_MM = 20.0

# 修正量太小時不移動，避免手臂抖動
MIN_CENTER_CORRECTION_MM = 0.5

# 預設允許的畫面中心誤差
DEFAULT_CENTER_THRESHOLD_PX = 8.0

# 修正方向控制
#
# 正常先使用 1.0。
# 如果實際移動後 AprilTag 反而離畫面中心更遠，
# 將這裡改成 -1.0。
CENTER_CORRECTION_SIGN = 1.0


# ============================================================
# 手臂位置設定
# ============================================================

PHOTO_POSE = [
    -3.583,
    13.158,
    20.430,
    -0.430,
    -80.445,
    -92.938
]

# 第一次定位後，移至 AprilTag 上方的距離
TAG_APPROACH_Z_MM = 140.0

# 最後移至 RJ45 上方的距離
RJ45_APPROACH_Z_MM = 150.0

TARGET_RZ_OFFSET = 180.0


# ============================================================
# 狀態機
# ============================================================

class States(Enum):
    INIT = 0

    MOVE_TO_PHOTO_POSE = 1
    WAIT_FIRST_APRILTAG = 2
    MOVE_ABOVE_APRILTAG = 3

    PREPARE_SECOND_LOCALIZATION = 4
    WAIT_SECOND_APRILTAG = 5
    ALIGN_APRILTAG_CENTER = 6

    MOVE_ABOVE_RJ45 = 7
    CHECK_POSE = 8
    FINISH = 9


class ExampleStrategy(
    AprilTagSamplingMixin,
    HiwinRobotMixin,
    ToolPoseMixin,
    CenterAlignmentMixin,
    Rj45TargetMixin,
    Node
):
    CENTER_CORRECTION_SIGN = CENTER_CORRECTION_SIGN
    MIN_CENTER_CORRECTION_MM = MIN_CENTER_CORRECTION_MM
    MAX_CENTER_CORRECTION_MM = MAX_CENTER_CORRECTION_MM

    RJ45_APPROACH_Z_MM = RJ45_APPROACH_Z_MM
    TARGET_RZ_OFFSET = TARGET_RZ_OFFSET

    def __init__(self):
        super().__init__('example_strategy')

        # ----------------------------------------------------
        # HIWIN service client
        # ----------------------------------------------------

        self.hiwin_client = self.create_client(
            RobotCommand,
            'hiwinmodbus_service'
        )

        # 發布目前 Tool7 位姿，讓矩陣轉換節點使用
        self.tool_pose_pub = self.create_publisher(
            String,
            '/hiwin/tool_pose',
            10
        )

        # 訂閱轉換成 Base 座標後的 AprilTag 資料
        self.tag_sub = self.create_subscription(
            String,
            APRILTAG_TOPIC,
            self.tag_pose_callback,
            10
        )

        # ----------------------------------------------------
        # AprilTag 等待狀態
        # ----------------------------------------------------

        self.waiting_for_tag = False
        self.has_tag = False

        self.received_sample_count = 0

        # ----------------------------------------------------
        # AprilTag 取樣資料
        # ----------------------------------------------------

        self.tag_samples = []
        self.rj45_samples = []
        self.quat_samples = []

        self.center_error_samples = []
        self.center_correction_samples = []
        self.center_threshold_samples = []

        # ----------------------------------------------------
        # AprilTag 最終結果
        # ----------------------------------------------------

        self.latest_tag_position_m = None
        self.latest_tag_rj45_position_m = None
        self.latest_tag_quat = None

        self.latest_centered = False
        self.latest_center_error_px = None
        self.latest_center_correction_base_m = None
        self.latest_center_threshold_px = (
            DEFAULT_CENTER_THRESHOLD_PX
        )

        # 拍照點的手臂姿態
        self.photo_orientation_deg = None

        # 已執行幾次畫面置中
        self.center_align_count = 0

        self.get_logger().info(
            'Example strategy node started'
        )

    # ========================================================
    # 狀態機
    # ========================================================

    def _state_machine(self, state):

        # ----------------------------------------------------
        # INIT
        # ----------------------------------------------------

        if state == States.INIT:
            self.get_logger().info('INIT')

            self.reset_tag_sampling()

            self.photo_orientation_deg = None
            self.center_align_count = 0

            return States.MOVE_TO_PHOTO_POSE

        # ----------------------------------------------------
        # 移動到第一階段拍照位置
        # ----------------------------------------------------

        if state == States.MOVE_TO_PHOTO_POSE:
            self.get_logger().info(
                'MOVE_TO_PHOTO_POSE'
            )

            request = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                joints=PHOTO_POSE,
                tool=ACTIVE_TOOL,
                base=ACTIVE_BASE,
                holding=True
            )

            response = self.call_hiwin(request)

            if response is None:
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

        # ----------------------------------------------------
        # 等待第一次 AprilTag 定位
        # ----------------------------------------------------

        if state == States.WAIT_FIRST_APRILTAG:
            if not self.wait_for_tag_result(
                'WAIT_FIRST_APRILTAG'
            ):
                return States.FINISH

            return States.MOVE_ABOVE_APRILTAG

        # ----------------------------------------------------
        # 根據第一次定位移至 AprilTag 上方
        # ----------------------------------------------------

        if state == States.MOVE_ABOVE_APRILTAG:
            self.get_logger().info(
                'MOVE_ABOVE_APRILTAG'
            )

            if self.latest_tag_position_m is None:
                self.get_logger().error(
                    'First AprilTag position is missing'
                )
                return States.FINISH

            if self.photo_orientation_deg is None:
                self.get_logger().error(
                    'Photo orientation is missing'
                )
                return States.FINISH

            tag_x, tag_y, tag_z = (
                self.latest_tag_position_m
            )

            rx, ry, rz = (
                self.photo_orientation_deg
            )

            pose = Twist()

            pose.linear.x = (
                float(tag_x) * 1000.0
            )

            pose.linear.y = (
                float(tag_y) * 1000.0
            )

            pose.linear.z = (
                float(tag_z) * 1000.0
                + TAG_APPROACH_Z_MM
            )

            # 第一階段靠近時保持原本拍照姿態
            pose.angular.x = float(rx)
            pose.angular.y = float(ry)
            pose.angular.z = float(rz)

            self.get_logger().info(
                f'Target above AprilTag: '
                f'x={pose.linear.x:.3f}, '
                f'y={pose.linear.y:.3f}, '
                f'z={pose.linear.z:.3f}, '
                f'rx={pose.angular.x:.3f}, '
                f'ry={pose.angular.y:.3f}, '
                f'rz={pose.angular.z:.3f}'
            )

            request = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                tool=ACTIVE_TOOL,
                base=ACTIVE_BASE,
                pose=pose,
                holding=True
            )

            response = self.call_hiwin(request)

            if response is None:
                self.get_logger().error(
                    'Move above AprilTag failed'
                )
                return States.FINISH

            # 接下來才開始計算第二階段置中次數
            self.center_align_count = 0

            time.sleep(1.0)

            return States.PREPARE_SECOND_LOCALIZATION

        # ----------------------------------------------------
        # 準備第二階段近距離定位
        # ----------------------------------------------------

        if state == States.PREPARE_SECOND_LOCALIZATION:
            self.get_logger().info(
                'PREPARE_SECOND_LOCALIZATION'
            )

            self.reset_tag_sampling()
            self.waiting_for_tag = True

            # 更新目前 Tool7 位姿，矩陣節點才能用正確位姿
            if not self.update_and_publish_tool_pose():
                self.waiting_for_tag = False
                return States.FINISH

            return States.WAIT_SECOND_APRILTAG

        # ----------------------------------------------------
        # 等待第二階段 AprilTag 資料
        # ----------------------------------------------------

        if state == States.WAIT_SECOND_APRILTAG:
            if not self.wait_for_tag_result(
                'WAIT_SECOND_APRILTAG'
            ):
                return States.FINISH

            if self.latest_center_error_px is None:
                self.get_logger().error(
                    'Center error data is missing'
                )
                return States.FINISH

            error_u, error_v = (
                self.latest_center_error_px
            )

            self.get_logger().info(
                f'Second localization result: '
                f'u={error_u:.2f}px, '
                f'v={error_v:.2f}px, '
                f'centered={self.latest_centered}, '
                f'align count='
                f'{self.center_align_count}/'
                f'{MAX_CENTER_ALIGN_COUNT}'
            )

            # 已經在畫面中心，可以繼續前往 RJ45
            if self.latest_centered:
                self.get_logger().info(
                    'AprilTag is centered'
                )
                return States.MOVE_ABOVE_RJ45

            # 超過最大修正次數就停止
            if (self.center_align_count>= MAX_CENTER_ALIGN_COUNT):
                self.get_logger().error(
                    'AprilTag center alignment '
                    'exceeded maximum count'
                )
                return States.FINISH

            return States.ALIGN_APRILTAG_CENTER

        # ----------------------------------------------------
        # 修正手臂位置，讓 AprilTag 靠近畫面中心
        # ----------------------------------------------------

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

            request = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                velocity=CENTER_ALIGN_VELOCITY,
                acceleration=CENTER_ALIGN_ACCELERATION,
                tool=ACTIVE_TOOL,
                base=ACTIVE_BASE,
                pose=pose,
                holding=True
            )

            response = self.call_hiwin(request)

            if response is None:
                self.get_logger().error(
                    'Center alignment movement failed'
                )
                return States.FINISH

            self.center_align_count += 1

            self.get_logger().info(
                f'Center alignment movement '
                f'{self.center_align_count}/'
                f'{MAX_CENTER_ALIGN_COUNT} completed'
            )

            # 等手臂和畫面穩定後重新取樣
            time.sleep(1.0)

            return States.PREPARE_SECOND_LOCALIZATION

        # ----------------------------------------------------
        # 移動到 RJ45 上方
        # ----------------------------------------------------

        if state == States.MOVE_ABOVE_RJ45:
            self.get_logger().info(
                'MOVE_ABOVE_RJ45'
            )

            pose = (
                self.calculate_rj45_target_pose()
            )

            if pose is None:
                return States.FINISH

            request = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                tool=ACTIVE_TOOL,
                base=ACTIVE_BASE,
                pose=pose,
                holding=True
            )

            response = self.call_hiwin(request)

            if response is None:
                self.get_logger().error(
                    'Move above RJ45 failed'
                )
                return States.FINISH

            time.sleep(3.0)

            return States.CHECK_POSE

        # ----------------------------------------------------
        # 確認最後位置
        # ----------------------------------------------------

        if state == States.CHECK_POSE:
            self.get_logger().info(
                'CHECK_POSE'
            )

            current_pose = (
                self.get_current_robot_pose()
            )

            if current_pose is not None:
                self.get_logger().info(
                    f'Current position: '
                    f'{current_pose}'
                )

            return States.FINISH

        return States.FINISH

    # ========================================================
    # 主流程
    # ========================================================

    def _main_loop(self):
        state = States.INIT

        while (
            rclpy.ok()
            and state != States.FINISH
        ):
            state = self._state_machine(
                state
            )

        self.get_logger().info('FINISH')

    def start_main_loop_thread(self):
        self.main_loop_thread = Thread(
            target=self._main_loop,
            daemon=True
        )

        self.main_loop_thread.start()


def main(args=None):
    rclpy.init(args=args)

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
        pass

    finally:
        strategy.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()