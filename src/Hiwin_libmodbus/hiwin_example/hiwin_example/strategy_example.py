#!/usr/bin/env python3
import json
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
from std_msgs.msg import Bool, String

from hiwin_example.apriltag_sampling_mixin import AprilTagSamplingMixin
from hiwin_example.hiwin_robot_mixin import HiwinRobotMixin
from hiwin_example.tool_pose_mixin import ToolPoseMixin
from hiwin_example.center_alignment_mixin import CenterAlignmentMixin
from hiwin_example.rj45_target_mixin import Rj45TargetMixin

# ============================================================
# HIWIN 基本設定
# ============================================================

DEFAULT_VELOCITY = 10
DEFAULT_ACCELERATION = 10

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
YOLO_DETECTIONS_BASE_TOPIC = '/yolo/detections_base'

TARGET_RJ45_TRACK_KEY = 'RJ45_1'

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
TAG_APPROACH_Z_MM = 160.0

# 最後移至 RJ45 上方的距離
RJ45_APPROACH_Z_MM = 15.0

TARGET_RZ_OFFSET = 180.0
CAMERA_RX_OFFSET_DEG = 30.0

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

    MOVE_ABOVE_BOARD_CENTER = 7
    WAIT_YOLO_DETECTION = 8
    MOVE_ABOVE_YOLO_TARGET = 9

    CHECK_POSE = 10
    FINISH = 11


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

        self.freeze_pnp_z_pub = self.create_publisher(
            Bool,
            '/apriltag/freeze_pnp_z',
            10
        )

        # 訂閱轉換成 Base 座標後的 AprilTag 資料
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
        self.latest_center_threshold_px = DEFAULT_CENTER_THRESHOLD_PX

        # 拍照點的手臂姿態
        self.photo_orientation_deg = None

        # 已執行幾次畫面置中
        self.center_align_count = 0

        self.get_logger().info( 'Example strategy node started' )

    def yolo_detections_base_callback(self, msg):
        try:
            data = json.loads(msg.data)

            detections = data.get('detections', [])

            if not isinstance(detections, list):
                self.get_logger().warn( 'YOLO detections must be a list' )
                return

            selected = None

            for det in detections:
                if det.get('track_key') == TARGET_RJ45_TRACK_KEY:
                    selected = det
                    break

            if selected is None:
                self.get_logger().warn( f'Target {TARGET_RJ45_TRACK_KEY} was not found' )
                return

            position_m = selected.get('detection_base_position_m')

            if (
                position_m is None
                or len(position_m) != 3
                or any(value is None for value in position_m)
            ):
                self.get_logger().warn( f'{TARGET_RJ45_TRACK_KEY} has invalid base position' )
                return

            self.latest_yolo_detection_base_m = [
                float(position_m[0]),
                float(position_m[1]),
                float(position_m[2])
            ]

            self.latest_yolo_detection_info = selected
            self.has_yolo_detection = True

            x, y, z = self.latest_yolo_detection_base_m

            self.get_logger().info(
                f'Selected {TARGET_RJ45_TRACK_KEY}: '
                f'x={x * 1000.0:.3f} mm, '
                f'y={y * 1000.0:.3f} mm, '
                f'z={z * 1000.0:.3f} mm'
            )

        except json.JSONDecodeError as exc:
            self.get_logger().error( f'Failed to decode YOLO detections base: {exc}' )

        except Exception as exc:
            self.get_logger().error( f'Failed to parse YOLO detections base: {exc}' )

    def freeze_current_pnp_z(self):
        msg = Bool()
        msg.data = True

        self.freeze_pnp_z_pub.publish(msg)

        self.get_logger().info('Published freeze PnP Z command')

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

            self.latest_yolo_detection_base_m = None
            self.latest_yolo_detection_info = None
            self.has_yolo_detection = False

            return States.MOVE_TO_PHOTO_POSE

        # ----------------------------------------------------
        # 移動到第一階段拍照位置
        # ----------------------------------------------------

        if state == States.MOVE_TO_PHOTO_POSE:
            self.get_logger().info( 'MOVE_TO_PHOTO_POSE' )

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
                self.get_logger().error( 'Move to photo pose failed' )
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
            self.get_logger().info( 'MOVE_ABOVE_APRILTAG' )

            if self.latest_tag_position_m is None:
                self.get_logger().error( 'First AprilTag position is missing' )
                return States.FINISH

            if self.photo_orientation_deg is None:
                self.get_logger().error( 'Photo orientation is missing' )
                return States.FINISH

            #會有問題嗎？？？
            # tag_x, tag_y, tag_z = self.latest_camera_aligned_tool7_pose_base['position_m']
            tag_x, tag_y, tag_z = self.latest_tag_position_m
            rx, ry, rz = self.latest_camera_aligned_tool7_pose_base['euler_deg']

            pose = Twist()

            pose.linear.x = ( float(tag_x) * 1000.0 )
            pose.linear.y = ( float(tag_y) * 1000.0 )

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
                self.get_logger().error( 'Move above AprilTag failed' )
                return States.FINISH
            
            #存z
            time.sleep(1.0)

            current_pose = self.get_current_robot_pose()
            if current_pose is None:
                self.get_logger().error(
                    'Cannot read pose above AprilTag'
                )
                return States.FINISH

            # current_pose 預期為：
            # [x, y, z, rx, ry, rz]
            self.tag_above_z_mm = float(
                current_pose[2]
            )

            self.get_logger().info(
                f'Saved Tag-above Z: '
                f'{self.tag_above_z_mm:.3f} mm'
            )

            # 接下來才開始計算第二階段置中次數
            self.center_align_count = 0

            time.sleep(1.0)

            return States.PREPARE_SECOND_LOCALIZATION

        # ----------------------------------------------------
        # 準備第二階段近距離定位
        # ----------------------------------------------------

        if state == States.PREPARE_SECOND_LOCALIZATION:
            self.get_logger().info( 'PREPARE_SECOND_LOCALIZATION' )

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
                self.get_logger().error( 'Center error data is missing' )
                return States.FINISH

            self.freeze_current_pnp_z()
            time.sleep(3)

            error_u, error_v = self.latest_center_error_px

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
                self.get_logger().info( 'AprilTag is centered' )
                return States.MOVE_ABOVE_BOARD_CENTER

            # 超過最大修正次數就停止
            if (self.center_align_count>= MAX_CENTER_ALIGN_COUNT):
                self.get_logger().error( 'AprilTag center alignment exceeded maximum count' )
                return States.FINISH

            # return States.ALIGN_APRILTAG_CENTER
            return States.MOVE_ABOVE_BOARD_CENTER

        # ----------------------------------------------------
        # 修正手臂位置，讓 AprilTag 靠近畫面中心
        # ----------------------------------------------------

        if state == States.ALIGN_APRILTAG_CENTER:
            self.get_logger().info( 'ALIGN_APRILTAG_CENTER' )

            current_pose = self.get_current_robot_pose()

            if current_pose is None:
                return States.FINISH

            pose = self.calculate_center_alignment_pose(
                current_pose=current_pose
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
                self.get_logger().error( 'Center alignment movement failed' )
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
        # 移動到 板中心 上方
        # ----------------------------------------------------

        if state == States.MOVE_ABOVE_BOARD_CENTER:
            self.get_logger().info( 'MOVE_ABOVE_BOARD_CENTER' )

            if self.latest_board_center_position_m is None:
                self.get_logger().error( 'Board center position is missing' )
                return States.FINISH

            if self.latest_camera_aligned_tool7_pose_base is None:
                self.get_logger().error('Camera-aligned Tool7 pose is missing')
                return States.FINISH
            

            if self.tag_above_z_mm is None:
                self.get_logger().error('Saved Tag-above Z is missing')
                return States.FINISH

            board_x, board_y, _ = self.latest_board_center_position_m
            # _, _, tag_z = self.latest_tag_position_m
            rx, ry, rz = self.latest_camera_aligned_tool7_pose_base['euler_deg']

            pose = Twist()

            pose.linear.x = float(board_x) * 1000.0
            pose.linear.y = float(board_y) * 1000.0
            # pose.linear.z = (float(tag_z) * 1000.0 + TAG_APPROACH_Z_MM)
            pose.linear.z = float(self.tag_above_z_mm)

            pose.angular.x = float(rx)
            pose.angular.y = float(ry)
            pose.angular.z = float(rz)

            self.get_logger().info(
                f'Target above board center: '
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
                self.get_logger().error( 'Move above board center failed' )
                return States.FINISH
            
            if not self.update_and_publish_tool_pose():
                return States.FINISH

            time.sleep(1.0)

            self.latest_yolo_detection_base_m = None
            self.latest_yolo_detection_info = None
            self.has_yolo_detection = False

            return States.WAIT_YOLO_DETECTION



        
        # ----------------------------------------------------
        # 等yolo偵測
        # ----------------------------------------------------
        if state == States.WAIT_YOLO_DETECTION:
            self.get_logger().info( 'WAIT_YOLO_DETECTION' )

            self.get_logger().info(
                'Waiting 10 seconds for YOLO to stabilize'
            )

            time.sleep(5.0)

            start_time = time.time()
            timeout_sec = 15.0

            while (
                rclpy.ok()
                and time.time() - start_time < timeout_sec
            ):
                if self.has_yolo_detection:
                    x, y, z = self.latest_yolo_detection_base_m

                    self.get_logger().info(
                        f'YOLO target received: '
                        f'track_key={TARGET_RJ45_TRACK_KEY}, '
                        f'x={x * 1000.0:.3f} mm, '
                        f'y={y * 1000.0:.3f} mm, '
                        f'z={z * 1000.0:.3f} mm'
                    )

                    return States.MOVE_ABOVE_YOLO_TARGET

                time.sleep(0.05)

            self.get_logger().error( 'YOLO detection timeout' )

            return States.FINISH
        
        # ----------------------------------------------------
        # 移到yolo偵測點上方
        # ----------------------------------------------------

        if state == States.MOVE_ABOVE_YOLO_TARGET:
            self.get_logger().info(
                'MOVE_ABOVE_YOLO_TARGET'
            )

            if self.latest_yolo_detection_base_m is None:
                self.get_logger().error(
                    'YOLO target base position is missing'
                )
                return States.FINISH

            if self.latest_tag_quat is None:
                self.get_logger().error(
                    'Second AprilTag orientation is missing'
                )
                return States.FINISH

            # if not self.latest_centered:
            #     self.get_logger().error(
            #         'Refusing to move above YOLO target '
            #         'because AprilTag is not centered'
            #     )
            #     return States.FINISH

            target_x, target_y, target_z = (
                self.latest_yolo_detection_base_m
            )

            qx, qy, qz, qw = self.latest_tag_quat

            try:
                rx, ry, rz = R.from_quat(
                    [qx, qy, qz, qw]
                ).as_euler(
                    'xyz',
                    degrees=True
                )

            except Exception as exc:
                self.get_logger().error(
                    f'Quaternion conversion failed: {exc}'
                )
                return States.FINISH

            x_raw = float(target_x) * 1000.0
            y_raw = float(target_y) * 1000.0
            z_raw = float(target_z) * 1000.0

            # corrected_x_mm, corrected_y_mm = self.correct_xy(
            #     x_raw,
            #     y_raw
            # )

            pose = Twist()

            pose.linear.x = x_raw
            pose.linear.y = y_raw

            # pose.linear.x = corrected_x_mm
            # pose.linear.y = corrected_y_mm

            # YOLO 的 z + 安全高度
            pose.linear.z = (
                z_raw
                + RJ45_APPROACH_Z_MM
            )

            # 姿態使用 AprilTag 的 quaternion
            pose.angular.x = float(rx)
            pose.angular.y = float(ry)

            pose.angular.z = (float(rz)- TARGET_RZ_OFFSET)

            self.get_logger().info(
                f'Target above YOLO RJ45: '
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
                    'Move above YOLO target failed'
                )
                return States.FINISH

            time.sleep(3.0)

            return States.CHECK_POSE


        # ----------------------------------------------------
        # 確認最後位置
        # ----------------------------------------------------

        if state == States.CHECK_POSE:
            self.get_logger().info( 'CHECK_POSE' )

            current_pose = self.get_current_robot_pose()

            if current_pose is not None:
                self.get_logger().info( f'Current position: {current_pose}' )

            return States.FINISH

        return States.FINISH


        # if state == States.CHECK_POSE:
        #     self.get_logger().info(
        #         'CHECK_POSE'
        #     )

        #     current_pose = self.get_current_robot_pose()

        #     if current_pose is not None:
        #         self.get_logger().info(
        #             f'Current position: {current_pose}'
        #         )
        #     else:
        #         self.get_logger().warning(
        #             'Current robot pose is unavailable'
        #         )

        #     if self.latest_tag_quat is not None:
        #         qx, qy, qz, qw = self.latest_tag_quat

        #         self.get_logger().info(
        #             f'Latest AprilTag quaternion: '
        #             f'qx={qx:.6f}, '
        #             f'qy={qy:.6f}, '
        #             f'qz={qz:.6f}, '
        #             f'qw={qw:.6f}'
        #         )

        #         try:
        #             tag_rx, tag_ry, tag_rz = (
        #                 R.from_quat(
        #                     [qx, qy, qz, qw]
        #                 ).as_euler(
        #                     'xyz',
        #                     degrees=True
        #                 )
        #             )

        #             self.get_logger().info(
        #                 f'Latest AprilTag Euler: '
        #                 f'rx={tag_rx:.3f}, '
        #                 f'ry={tag_ry:.3f}, '
        #                 f'rz={tag_rz:.3f}'
        #             )

        #         except Exception as exc:
        #             self.get_logger().error(
        #                 f'Quaternion conversion failed: {exc}'
        #             )

        #     else:
        #         self.get_logger().warning(
        #             'Latest AprilTag quaternion is unavailable'
        #         )

        #     # 避免迴圈印得太快
        #     time.sleep(0.5)

        #     # 不結束，繼續停留在 CHECK_POSE
        #     return States.FINISH

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
