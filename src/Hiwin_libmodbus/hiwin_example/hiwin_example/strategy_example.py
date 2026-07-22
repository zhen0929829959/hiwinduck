#!/usr/bin/env python3

import time
from enum import Enum
from threading import Event, Thread

import rclpy
from hiwin_interfaces.srv import RobotCommand
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool, String

from hiwin_example.apriltag_sampling_mixin import AprilTagSamplingMixin
from hiwin_example.center_alignment_mixin import CenterAlignmentMixin
from hiwin_example.hiwin_robot_mixin import HiwinRobotMixin
from hiwin_example.insertion_mixin import InsertionMixin
from hiwin_example.rj45_target_mixin import Rj45TargetMixin
from hiwin_example.strategy_motion_mixin import StrategyMotionMixin
from hiwin_example.tool_pose_mixin import ToolPoseMixin
from hiwin_example.visual_alignment_mixin import VisualAlignmentMixin
from hiwin_example.yolo_detection_mixin import YoloDetectionMixin


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

DISCARD_SAMPLE_COUNT = 10
MEDIAN_SAMPLE_COUNT = 5
TAG_TIMEOUT_SEC = 10.0

APRILTAG_TOPIC = '/apriltag/pose_base'
YOLO_DETECTIONS_BASE_TOPIC = '/yolo/detections_base'

TARGET_RJ45_TRACK_KEY = 'RJ45_0'


# ============================================================
# AprilTag 置中設定
# ============================================================

MAX_CENTER_ALIGN_COUNT = 5

CENTER_ALIGN_VELOCITY = 5
CENTER_ALIGN_ACCELERATION = 5

MAX_CENTER_CORRECTION_MM = 20.0
MIN_CENTER_CORRECTION_MM = 0.5

DEFAULT_CENTER_THRESHOLD_PX = 8.0
CENTER_CORRECTION_SIGN = 1.0


# ============================================================
# YOLO 視覺微調設定
# ============================================================

PLUG_CENTER_U = 1282.0
PLUG_CENTER_V = 625.0

CAMERA_FX = 1362.7784437304256
CAMERA_FY = 1361.8357978878923

VISUAL_ALIGN_DEPTH_M = 0.20
VISUAL_ALIGN_KP = 0.5
VISUAL_ALIGN_THRESHOLD_PX = 4.0

MAX_VISUAL_ALIGN_STEP_MM = 2.0

VISUAL_ALIGN_X_SIGN = 1.0
VISUAL_ALIGN_Y_SIGN = 1.0

MAX_VISUAL_ALIGN_COUNT = 10


# ============================================================
# 插入設定
# ============================================================

INSERT_DISTANCE_MM = 40.0
SUCCESS_DEPTH_THRESHOLD_MM = 17.0
DEPTH_TOLERANCE_MM = 0.5

INSERT_VELOCITY = 1
INSERT_ACCELERATION = 1

RETURN_VELOCITY = 3
RETURN_ACCELERATION = 3

MONITOR_READY_TIMEOUT_SEC = 5.0
MAX_INSERTION_TIME_SEC = 100.0

STOP_SETTLE_SEC = 0.5
RETURN_SETTLE_SEC = 0.5


# ============================================================
# 手臂位置設定
# ============================================================

PHOTO_POSE = [-3.583, 13.158, 20.430, -0.430, -80.445, -92.938]

TAG_APPROACH_Z_MM = 160.0
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
    VISUAL_FINE_ALIGN = 10
    CHECK_POSE = 11
    PREPARE_INSERTION = 12
    RUN_INSERTION = 13
    CHECK_INSERTION_RESULT = 14
    FINISH = 15


class ExampleStrategy(
    AprilTagSamplingMixin,
    HiwinRobotMixin,
    ToolPoseMixin,
    CenterAlignmentMixin,
    Rj45TargetMixin,
    VisualAlignmentMixin,
    YoloDetectionMixin,
    StrategyMotionMixin,
    InsertionMixin,
    Node
):
    ACTIVE_TOOL = ACTIVE_TOOL
    ACTIVE_BASE = ACTIVE_BASE

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

    INSERT_DISTANCE_MM = INSERT_DISTANCE_MM
    SUCCESS_DEPTH_THRESHOLD_MM = SUCCESS_DEPTH_THRESHOLD_MM
    DEPTH_TOLERANCE_MM = DEPTH_TOLERANCE_MM

    INSERT_VELOCITY = INSERT_VELOCITY
    INSERT_ACCELERATION = INSERT_ACCELERATION

    RETURN_VELOCITY = RETURN_VELOCITY
    RETURN_ACCELERATION = RETURN_ACCELERATION

    MONITOR_READY_TIMEOUT_SEC = MONITOR_READY_TIMEOUT_SEC
    MAX_INSERTION_TIME_SEC = MAX_INSERTION_TIME_SEC

    STOP_SETTLE_SEC = STOP_SETTLE_SEC
    RETURN_SETTLE_SEC = RETURN_SETTLE_SEC

    def __init__(self):
        super().__init__('example_strategy')

        self.hiwin_client = self.create_client(
            RobotCommand,
            'hiwinmodbus_service'
        )

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

        self.insertion_command_pub = self.create_publisher(
            String,
            '/insertion/command',
            10
        )

        self.insertion_status_sub = self.create_subscription(
            String,
            '/insertion/status',
            self.insertion_status_callback,
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
        self.latest_center_threshold_px = DEFAULT_CENTER_THRESHOLD_PX

        self.photo_orientation_deg = None
        self.center_align_count = 0

        self.latest_hole_center_px = None
        self.latest_r_base_camera = None
        self.visual_align_count = 0

        self.insertion_monitor_ready_event = Event()
        self.insertion_stop_event = Event()

        self.insertion_stop_reason = None
        self.latest_insertion_monitor_data = None

        self.insertion_start_pose = None
        self.insertion_target_pose = None
        self.insertion_stopped_pose = None

        self.insertion_depth_mm = 0.0
        self.insertion_result = None

        self.get_logger().info(
            'Example strategy node started'
        )

    def freeze_current_pnp_z(self):
        msg = Bool()
        msg.data = True
        self.freeze_pnp_z_pub.publish(msg)

        self.get_logger().info(
            'Published freeze PnP Z command'
        )

    def _state_machine(self, state):
        if state == States.INIT:
            self.get_logger().info('INIT')

            self.reset_tag_sampling()

            self.photo_orientation_deg = None
            self.center_align_count = 0

            self.latest_yolo_detection_base_m = None
            self.latest_yolo_detection_info = None
            self.has_yolo_detection = False

            return States.MOVE_TO_PHOTO_POSE

        if state == States.MOVE_TO_PHOTO_POSE:
            self.get_logger().info(
                'MOVE_TO_PHOTO_POSE'
            )

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

        if state == States.WAIT_FIRST_APRILTAG:
            if not self.wait_for_tag_result(
                'WAIT_FIRST_APRILTAG'
            ):
                return States.FINISH

            return States.MOVE_ABOVE_APRILTAG

        if state == States.MOVE_ABOVE_APRILTAG:
            self.get_logger().info(
                'MOVE_ABOVE_APRILTAG'
            )

            if self.latest_tag_position_m is None:
                self.get_logger().error(
                    'First AprilTag position is missing'
                )
                return States.FINISH

            if self.latest_camera_aligned_tool7_pose_base is None:
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
                tag_z * 1000.0 + TAG_APPROACH_Z_MM,
                rx,
                ry,
                rz
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

            current_pose = self.get_current_robot_pose()

            if current_pose is None:
                self.get_logger().error(
                    'Cannot read pose above AprilTag'
                )
                return States.FINISH

            self.tag_above_z_mm = float(
                current_pose[2]
            )

            self.center_align_count = 0

            return States.PREPARE_SECOND_LOCALIZATION

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

            self.freeze_current_pnp_z()
            time.sleep(3.0)

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

            if self.latest_centered:
                return States.MOVE_ABOVE_BOARD_CENTER

            if self.center_align_count >= MAX_CENTER_ALIGN_COUNT:
                self.get_logger().error(
                    'AprilTag center alignment '
                    'exceeded maximum count'
                )
                return States.FINISH

            # 若要重新啟用 AprilTag 置中，改成：
            # return States.ALIGN_APRILTAG_CENTER
            return States.MOVE_ABOVE_BOARD_CENTER

        if state == States.ALIGN_APRILTAG_CENTER:
            self.get_logger().info(
                'ALIGN_APRILTAG_CENTER'
            )

            current_pose = self.get_current_robot_pose()

            if current_pose is None:
                return States.FINISH

            pose = self.calculate_center_alignment_pose(
                current_pose=current_pose
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

        if state == States.MOVE_ABOVE_BOARD_CENTER:
            self.get_logger().info(
                'MOVE_ABOVE_BOARD_CENTER'
            )

            if self.latest_board_center_position_m is None:
                self.get_logger().error(
                    'Board center position is missing'
                )
                return States.FINISH

            if self.latest_camera_aligned_tool7_pose_base is None:
                self.get_logger().error(
                    'Camera-aligned Tool7 pose is missing'
                )
                return States.FINISH

            if self.tag_above_z_mm is None:
                self.get_logger().error(
                    'Saved Tag-above Z is missing'
                )
                return States.FINISH

            board_x, board_y, _ = (
                self.latest_board_center_position_m
            )

            rx, ry, rz = (
                self.latest_camera_aligned_tool7_pose_base[
                    'euler_deg'
                ]
            )

            pose = self.create_pose(
                board_x * 1000.0,
                board_y * 1000.0,
                self.tag_above_z_mm,
                rx,
                ry,
                rz
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

        if state == States.WAIT_YOLO_DETECTION:
            self.get_logger().info(
                'WAIT_YOLO_DETECTION'
            )

            time.sleep(5.0)

            start_time = time.time()

            while (
                rclpy.ok()
                and time.time() - start_time < 15.0
            ):
                if self.has_yolo_detection:
                    return States.MOVE_ABOVE_YOLO_TARGET

                time.sleep(0.05)

            self.get_logger().error(
                'YOLO detection timeout'
            )
            return States.FINISH

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

            target_x, target_y, target_z = (
                self.latest_yolo_detection_base_m
            )

            try:
                rx, ry, rz = R.from_quat(
                    self.latest_tag_quat
                ).as_euler(
                    'xyz',
                    degrees=True
                )

            except Exception as exc:
                self.get_logger().error(
                    f'Quaternion conversion failed: {exc}'
                )
                return States.FINISH

            pose = self.create_pose(
                target_x * 1000.0,
                target_y * 1000.0,
                target_z * 1000.0 + RJ45_APPROACH_Z_MM,
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
                    'Move above YOLO target failed'
                )
                return States.FINISH

            if not self.update_and_publish_tool_pose():
                return States.FINISH

            time.sleep(2.0)

            self.visual_align_count = 0
            self.latest_hole_center_px = None

            return States.VISUAL_FINE_ALIGN

        if state == States.VISUAL_FINE_ALIGN:
            self.get_logger().info(
                'VISUAL_FINE_ALIGN'
            )

            if self.visual_align_count >= MAX_VISUAL_ALIGN_COUNT:
                self.get_logger().error(
                    'Visual alignment exceeded '
                    'maximum count'
                )
                return States.FINISH

            self.latest_hole_center_px = None

            start_time = time.time()

            while (
                rclpy.ok()
                and time.time() - start_time < 5.0
            ):
                if self.latest_hole_center_px is not None:
                    break

                time.sleep(0.05)

            if self.latest_hole_center_px is None:
                self.get_logger().error(
                    'Waiting for new hole center timeout'
                )
                return States.FINISH

            current_pose = self.get_current_robot_pose()

            if current_pose is None:
                return States.FINISH

            pose = self.calculate_visual_alignment_pose(
                current_pose=current_pose
            )

            if pose is False:
                return States.CHECK_POSE

            if pose is None:
                return States.FINISH

            if not self.move_pose(
                pose,
                5,
                5,
                holding=True
            ):
                self.get_logger().error(
                    'Visual alignment movement failed'
                )
                return States.FINISH

            self.visual_align_count += 1

            time.sleep(1.0)

            if not self.update_and_publish_tool_pose():
                return States.FINISH

            time.sleep(0.5)

            return States.VISUAL_FINE_ALIGN

        if state == States.CHECK_POSE:
            self.get_logger().info('CHECK_POSE')

            current_pose = self.get_current_robot_pose()

            if current_pose is None:
                self.get_logger().error(
                    'Cannot read final alignment pose'
                )
                return States.FINISH

            self.insertion_start_pose = list(
                current_pose
            )

            self.get_logger().info(
                f'Final alignment pose: '
                f'{self.insertion_start_pose}'
            )

            return States.PREPARE_INSERTION

        if state == States.PREPARE_INSERTION:
            self.get_logger().info(
                'PREPARE_INSERTION'
            )

            if not self.prepare_insertion():
                return States.FINISH

            return States.RUN_INSERTION

        if state == States.RUN_INSERTION:
            self.get_logger().info(
                'RUN_INSERTION'
            )

            if not self.run_insertion():
                return States.FINISH

            return States.CHECK_INSERTION_RESULT

        if state == States.CHECK_INSERTION_RESULT:
            self.get_logger().info(
                'CHECK_INSERTION_RESULT'
            )

            self.check_insertion_result()

            return States.FINISH

        return States.FINISH

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
        strategy.get_logger().warn(
            'Example strategy interrupted'
        )

        strategy.insertion_stop_reason = (
            'USER_INTERRUPT'
        )
        strategy.insertion_stop_event.set()

        if rclpy.ok():
            try:
                strategy.stop_insertion_motion()
            except Exception:
                pass

    finally:
        if rclpy.ok():
            try:
                strategy.publish_insertion_command(
                    'STOP'
                )
            except Exception:
                pass

        strategy.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()