#!/usr/bin/env python3
import json
from collections import deque
from threading import Lock, Thread
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from ur_robot_control.strategy_motion_mixin import StrategyMotionMixin
from ur_robot_control.ur_robot_mixin import UrRobotMixin

HAND_EYE = {
    # T_flange_camera: X_flange = T_flange_camera @ X_camera
    # 'left': {
    #     'translation_m': [0.06605019676959288, -0.0011101957303255375, 0.08656908061796169],
    #     'quaternion_xyzw': [-0.1719769579812515, -0.18369882942628987, 0.671477082986354, 0.6969915300884673],
    # },
    'left': {
        'translation_m': [0.06401128433566425, 0.0017546951302344677, 0.085698423527346],
        'quaternion_xyzw': [-0.16835079110393059, -0.18179580314821783, 0.6751788889608925, 0.6947962039288282],
    },
    'right': {
        'translation_m': [-0.07701936069829121, 0.0009718288741870998, 0.09111079541355217],
        'quaternion_xyzw': [0.17371258203270165, 0.16467844437669157, 0.6844976131363045, 0.6885985524323346],
    },
}

def matrix_from_translation_quaternion(translation, quaternion):
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix

def pose_to_matrix(pose):
    return matrix_from_translation_quaternion(
        [pose.position.x, pose.position.y, pose.position.z],
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
    )

def transform_to_matrix(transform):
    return matrix_from_translation_quaternion(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w],
    )

class AprilTagHandEyeTargetSelector(UrRobotMixin, StrategyMotionMixin, Node):
    """Calculate a hand-eye target and command the robot through MoveIt."""

    def __init__(self):
        super().__init__('apriltag_handeye_target_selector')

        self.declare_parameter('camera', 'left')
        self.declare_parameter('left_pose_topic', '/camera_left/apriltag/pose_camera')
        self.declare_parameter('right_pose_topic', '/camera_right/apriltag/pose_camera')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('flange_frame', 'tool0')
        self.declare_parameter('tcp_frame', 'tool0_controller')
        self.declare_parameter('clearance_m', 0.03)
        self.declare_parameter('sample_count', 30)
        self.declare_parameter('max_position_std_mm', 3.0)
        self.declare_parameter('velocity_percent', 10.0)
        self.declare_parameter('acceleration_percent', 10.0)

        # 固定 TCP 姿態，不再使用「目前姿態 + 180 度」
        self.declare_parameter('target_rx_deg', 180.0)
        self.declare_parameter('target_ry_deg', 0.0)
        self.declare_parameter('target_rz_deg', -135.0)
        # self.declare_parameter('target_rz_deg', -180.0)


        self.camera = str(self.get_parameter('camera').value).lower()
        if self.camera not in HAND_EYE:
            raise ValueError("camera must be 'left' or 'right'")

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.flange_frame = str(self.get_parameter('flange_frame').value)
        self.tcp_frame = str(self.get_parameter('tcp_frame').value)
        self.clearance_m = float(self.get_parameter('clearance_m').value)
        self.sample_count = int(self.get_parameter('sample_count').value)
        self.max_position_std_mm = float(self.get_parameter('max_position_std_mm').value)
        self.velocity_percent = float(self.get_parameter('velocity_percent').value)
        self.acceleration_percent = float(self.get_parameter('acceleration_percent').value)
        self.target_rx_deg = float(self.get_parameter('target_rx_deg').value)
        self.target_ry_deg = float(self.get_parameter('target_ry_deg').value)
        self.target_rz_deg = float(self.get_parameter('target_rz_deg').value)

        if self.sample_count <= 0:
            raise ValueError('sample_count must be greater than zero')
        if self.clearance_m < 0.0:
            raise ValueError('clearance_m must not be negative')

        topic_parameter = f'{self.camera}_pose_topic'
        self.pose_topic = str(self.get_parameter(topic_parameter).value)

        calibration = HAND_EYE[self.camera]

        self.t_handeye_raw = matrix_from_translation_quaternion(
            calibration['translation_m'],
            calibration['quaternion_xyzw']
        )

        self.t_handeye_inverse = np.linalg.inv(self.t_handeye_raw)

        self.init_ur_robot()

        self.samples = deque(maxlen=self.sample_count)
        self.latest_target = None
        self.motion_lock = Lock()
        self.motion_thread = None

        self.create_subscription(PoseStamped, self.pose_topic, self.tag_callback, 10)
        self.target_pub = self.create_publisher(PoseStamped, '~/target_pose', 10)
        self.diagnostic_pub = self.create_publisher(String, '~/diagnostic', 10)
        self.create_service(Trigger, '~/freeze_target', self.freeze_target)
        self.create_service(Trigger, '~/reset', self.reset)

        self.get_logger().info(
            f'camera={self.camera}, tag_topic={self.pose_topic}, '
            f'base={self.base_frame}, flange={self.flange_frame}, tcp={self.tcp_frame}, '
            f'clearance={self.clearance_m:.3f} m; '
            f'velocity={self.velocity_percent:.1f}%, '
            f'acceleration={self.acceleration_percent:.1f}%, '
            f'fixed_target_rpy=[{self.target_rx_deg:.2f}, '
            f'{self.target_ry_deg:.2f}, {self.target_rz_deg:.2f}] deg'
        )

    def lookup_matrix(self, target_frame, source_frame):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.2),
        )
        return transform_to_matrix(transform.transform)

    def tag_callback(self, msg):
        try:
            t_base_flange = self.lookup_matrix(
                self.base_frame,
                self.flange_frame
            )

            t_camera_tag = pose_to_matrix(msg.pose)

            # 假設 HAND_EYE 是 T_flange_camera
            t_base_tag_raw = (
                t_base_flange
                @ self.t_handeye_raw
                @ t_camera_tag
            )

            # 假設 HAND_EYE 其實是 T_camera_flange
            t_base_tag_inv = (
                t_base_flange
                @ self.t_handeye_inverse
                @ t_camera_tag
            )

        except (TransformException, ValueError) as exc:
            self.get_logger().warn(
                f'Cannot transform {self.camera} Tag pose: {exc}'
            )
            return

        raw = t_base_tag_raw[:3, 3]
        inv = t_base_tag_inv[:3, 3]

        self.get_logger().info(
            f'HAND_EYE RAW XYZ(mm)='
            f'[{raw[0]*1000:.2f}, '
            f'{raw[1]*1000:.2f}, '
            f'{raw[2]*1000:.2f}] | '
            f'INVERSE XYZ(mm)='
            f'[{inv[0]*1000:.2f}, '
            f'{inv[1]*1000:.2f}, '
            f'{inv[2]*1000:.2f}]'
        )
        if np.all(np.isfinite(raw)):
            self.samples.append(raw)

    def freeze_target(self, request, response):
        del request

        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'Robot motion is already in progress'
            return response

        if len(self.samples) < self.sample_count:
            response.success = False
            response.message = (
                f'Only {len(self.samples)}/{self.sample_count} valid samples'
            )
            return response

        values = np.stack(self.samples, axis=0)

        tag_position = np.median(values, axis=0)
        std_mm = np.std(values, axis=0) * 1000.0
        max_std_mm = float(np.max(std_mm))

        self.get_logger().warn(
            'TAG BASE MEDIAN: '
            f'XYZ(mm)=[{tag_position[0]*1000.0:.3f}, '
            f'{tag_position[1]*1000.0:.3f}, '
            f'{tag_position[2]*1000.0:.3f}], '
            f'STD(mm)=[{std_mm[0]:.3f}, '
            f'{std_mm[1]:.3f}, '
            f'{std_mm[2]:.3f}]'
        )

        if max_std_mm > self.max_position_std_mm:
            response.success = False
            response.message = (
                f'Pose unstable: std_xyz_mm={std_mm.tolist()}, '
                f'limit={self.max_position_std_mm:.2f} mm'
            )
            return response

        # 固定 TCP 姿態
        tcp_euler_deg = np.array([
            self.target_rx_deg,
            self.target_ry_deg,
            self.target_rz_deg,
        ], dtype=np.float64)

        target_quaternion = Rotation.from_euler(
            'xyz',
            tcp_euler_deg,
            degrees=True
        ).as_quat()

        target_position = tag_position.copy()

        # Tag 上方 clearance，沿 base_link +Z
        target_position[2] += self.clearance_m

        self.get_logger().warn(
            'TARGET TCP: '
            f'XYZ(mm)=[{target_position[0]*1000.0:.3f}, '
            f'{target_position[1]*1000.0:.3f}, '
            f'{target_position[2]*1000.0:.3f}], '
            f'RPY(deg)=[{tcp_euler_deg[0]:.3f}, '
            f'{tcp_euler_deg[1]:.3f}, '
            f'{tcp_euler_deg[2]:.3f}]'
        )

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.base_frame

        target.pose.position.x = float(target_position[0])
        target.pose.position.y = float(target_position[1])
        target.pose.position.z = float(target_position[2])

        target.pose.orientation.x = float(target_quaternion[0])
        target.pose.orientation.y = float(target_quaternion[1])
        target.pose.orientation.z = float(target_quaternion[2])
        target.pose.orientation.w = float(target_quaternion[3])

        self.latest_target = target
        self.target_pub.publish(target)

        diagnostic = {
            'camera': self.camera,
            'definition': (
                'T_base_tag = '
                'T_base_flange @ T_flange_camera @ T_camera_tag'
            ),
            'tag_base_m': tag_position.tolist(),
            'target_base_m': target_position.tolist(),
            'clearance_axis': f'+Z of {self.base_frame}',
            'clearance_m': self.clearance_m,
            'sample_count': len(values),
            'std_xyz_mm': std_mm.tolist(),
            'target_euler_xyz_deg': tcp_euler_deg.tolist(),
            'fixed_orientation': True,
            'motion_commanded': True,
        }

        diagnostic_msg = String()
        diagnostic_msg.data = json.dumps(diagnostic)
        self.diagnostic_pub.publish(diagnostic_msg)

        # 這次使用的 sample 已經結束，先清掉
        # 避免下一次 freeze 混到這次的資料
        self.samples.clear()

        self.motion_thread = Thread(
            target=self.execute_target_motion,
            args=(
                target_position.copy(),
                tcp_euler_deg.copy(),
            ),
            daemon=True,
        )
        self.motion_thread.start()

        response.success = True
        response.message = (
            f'{self.camera} robot motion started: '
            f'({target_position[0]:.6f}, '
            f'{target_position[1]:.6f}, '
            f'{target_position[2]:.6f}) m'
        )

        return response

    def execute_target_motion(
        self,
        target_position,
        target_euler_deg,
    ):
        with self.motion_lock:
            try:
                pose = self.create_pose(
                    target_position[0] * 1000.0,
                    target_position[1] * 1000.0,
                    target_position[2] * 1000.0,
                    target_euler_deg[0],
                    target_euler_deg[1],
                    target_euler_deg[2],
                )

                succeeded = self.move_pose(
                    pose,
                    self.velocity_percent,
                    self.acceleration_percent,
                    holding=True,
                )

                if succeeded:
                    current_pose = self.get_current_robot_pose()

                    self.get_logger().info(
                        'Move above AprilTag completed; '
                        f'current TCP pose={current_pose}'
                    )
                else:
                    self.get_logger().error(
                        'Move above AprilTag failed'
                    )

            except Exception as exc:
                self.get_logger().error(
                    f'Robot motion raised an exception: {exc}'
                )

    def reset(self, request, response):
        del request

        self.samples.clear()
        self.latest_target = None

        response.success = True
        response.message = 'Samples cleared'

        return response

def main(args=None):
    rclpy.init(args=args)

    node = AprilTagHandEyeTargetSelector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()