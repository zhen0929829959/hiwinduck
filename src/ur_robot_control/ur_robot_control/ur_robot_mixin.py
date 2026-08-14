#!/usr/bin/env python3

import math
import time

import numpy as np

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from shape_msgs.msg import SolidPrimitive


class UrRobotMixin:

    def init_ur_moveit(self):
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )

        self.current_plan_goal_handle = None
        self.current_execute_goal_handle = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False
        )

        self.get_logger().info(
            'Waiting for /move_action...'
        )

        if not self.move_group_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                '/move_action server not available'
            )

        self.get_logger().info(
            'Connected to /move_action'
        )

        self.get_logger().info(
            'Waiting for /execute_trajectory...'
        )

        if not self.execute_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                '/execute_trajectory server not available'
            )

        self.get_logger().info(
            'Connected to /execute_trajectory'
        )

    # ========================================================
    # 相容 strategy_example 目前的初始化名稱
    # ========================================================

    def init_ur_robot(self):
        return self.init_ur_moveit()

    def wait_for_future(
        self,
        future,
        timeout_sec=-1.0
    ):
        start_time = time.time()

        while (
            rclpy.ok()
            and not future.done()
        ):
            time.sleep(0.01)

            if (
                timeout_sec > 0.0
                and time.time() - start_time > timeout_sec
            ):
                return False

        return future.done()

    def convert_velocity(self, velocity):
        value = float(velocity) / 100.0

        if value > 1.0:
            value = 1.0

        if value < 0.01:
            value = 0.01

        return value

    def convert_acceleration(self, acceleration):
        value = float(acceleration) / 100.0

        if value > 1.0:
            value = 1.0

        if value < 0.01:
            value = 0.01

        return value

    def configure_request(
        self,
        goal,
        velocity,
        acceleration,
        pipeline_id='ompl',
        planner_id=''
    ):
        request = goal.request

        request.group_name = 'ur_manipulator'
        request.start_state.is_diff = True

        request.pipeline_id = pipeline_id
        request.planner_id = planner_id

        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0

        request.max_velocity_scaling_factor = (
            self.convert_velocity(velocity)
        )

        request.max_acceleration_scaling_factor = (
            self.convert_acceleration(acceleration)
        )

        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        return request

    def create_joint_goal(
        self,
        joints,
        velocity,
        acceleration,
        pipeline_id='pilz_industrial_motion_planner',
        planner_id='PTP'
    ):
        if (
            joints is None
            or len(joints) != 6
        ):
            self.get_logger().error(
                'UR5e requires exactly 6 joints'
            )
            return None

        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        goal = MoveGroup.Goal()

        request = self.configure_request(
            goal,
            velocity,
            acceleration,
            pipeline_id=pipeline_id,
            planner_id=planner_id
        )

        constraints = Constraints()

        for joint_name, joint_degree in zip(
            joint_names,
            joints
        ):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name

            joint_constraint.position = math.radians(
                float(joint_degree)
            )

            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0

            constraints.joint_constraints.append(
                joint_constraint
            )

        request.goal_constraints.append(
            constraints
        )

        return goal

    # ========================================================
    # TF / Matrix 工具
    # ========================================================

    def transform_to_matrix(self, transform):
        """
        geometry_msgs/TransformStamped -> 4x4 homogeneous matrix
        """
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        T = np.eye(4)

        T[:3, :3] = R.from_quat([
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        ]).as_matrix()

        T[:3, 3] = [
            translation.x,
            translation.y,
            translation.z
        ]

        return T

    def get_T_tool0_tcp(self):
        """
        由目前 TF 自動算出 Tool0 -> Active TCP。

        ROS / URDF:
            base_link -> tool0

        UR Controller + Teach Pendant Active TCP:
            base_link -> tool0_controller

        因此：
            T_tool0_tcp
            = inv(T_base_tool0) @ T_base_tcp

        這樣 Teach Pendant 更換 Active TCP 後，
        不需要在 Python 裡另外手改 TCP offset。
        """
        try:
            tf_base_tool0 = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                Time(),
                timeout=Duration(seconds=1.0)
            )

            tf_base_tcp = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0_controller',
                Time(),
                timeout=Duration(seconds=1.0)
            )

        except Exception as exc:
            self.get_logger().error(
                f'Cannot calculate Tool0 -> TCP: {exc}'
            )
            return None

        T_base_tool0 = self.transform_to_matrix(
            tf_base_tool0
        )

        T_base_tcp = self.transform_to_matrix(
            tf_base_tcp
        )

        T_tool0_tcp = (
            np.linalg.inv(T_base_tool0)
            @ T_base_tcp
        )

        tcp_xyz = T_tool0_tcp[:3, 3] * 1000.0

        tcp_rpy = R.from_matrix(
            T_tool0_tcp[:3, :3]
        ).as_euler(
            'xyz',
            degrees=True
        )

        self.get_logger().info(
            'Tool0 -> TCP: '
            f'XYZ(mm)={tcp_xyz}, '
            f'RPY(deg)={tcp_rpy}'
        )

        return T_tool0_tcp

    def tcp_pose_to_tool0_matrix(self, pose):
        """
        Strategy 傳進來的 pose 一律視為 TCP target：

            T_base_tcp_target

        MoveIt 現在控制的 link 是 tool0，
        所以在底層反推出：

            T_base_tool0_target
            = T_base_tcp_target
              @ inv(T_tool0_tcp)

        pose 格式維持原本：
            XYZ = mm
            RPY = degree
        """
        if pose is None:
            self.get_logger().error(
                'TCP target pose is None'
            )
            return None

        T_tool0_tcp = self.get_T_tool0_tcp()

        if T_tool0_tcp is None:
            return None

        x = float(pose.linear.x) / 1000.0
        y = float(pose.linear.y) / 1000.0
        z = float(pose.linear.z) / 1000.0

        rx = float(pose.angular.x)
        ry = float(pose.angular.y)
        rz = float(pose.angular.z)

        try:
            T_base_tcp_target = np.eye(4)

            T_base_tcp_target[:3, :3] = R.from_euler(
                'xyz',
                [rx, ry, rz],
                degrees=True
            ).as_matrix()

            T_base_tcp_target[:3, 3] = [
                x,
                y,
                z
            ]

            T_base_tool0_target = (
                T_base_tcp_target
                @ np.linalg.inv(T_tool0_tcp)
            )

            target_xyz = (
                T_base_tool0_target[:3, 3]
                * 1000.0
            )

            target_rpy = R.from_matrix(
                T_base_tool0_target[:3, :3]
            ).as_euler(
                'xyz',
                degrees=True
            )

            self.get_logger().warn(
                'MOVEIT TOOL0 TARGET: '
                f'XYZ(mm)={target_xyz}, '
                f'RPY(deg)={target_rpy}'
            )

        except Exception as exc:
            self.get_logger().error(
                f'TCP -> Tool0 conversion failed: {exc}'
            )
            return None

        return T_base_tool0_target

    def print_actual_tool0_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                Time(),
                timeout=Duration(seconds=1.0)
            )
        except Exception as exc:
            self.get_logger().error(
                f'Cannot get actual tool0: {exc}'
            )
            return

        t = transform.transform.translation
        q = transform.transform.rotation

        rpy = R.from_quat([
            q.x,
            q.y,
            q.z,
            q.w
        ]).as_euler(
            'xyz',
            degrees=True
        )

        self.get_logger().warn(
            'ACTUAL TOOL0: '
            f'XYZ(mm)='
            f'[{t.x * 1000.0:.3f}, '
            f'{t.y * 1000.0:.3f}, '
            f'{t.z * 1000.0:.3f}], '
            f'RPY(deg)={rpy}'
        )

    def create_pose_goal(
        self,
        pose,
        velocity,
        acceleration,
        pipeline_id='pilz_industrial_motion_planner',
        planner_id='PTP',
        position_tolerance=0.005,
        orientation_tolerance=0.05
    ):
        if pose is None:
            self.get_logger().error(
                'Pose is None'
            )
            return None

        # Strategy 給的是 TCP target。
        # 在這裡才反推成 MoveIt 要控制的 tool0 target。
        T_base_tool0_target = self.tcp_pose_to_tool0_matrix(
            pose
        )

        if T_base_tool0_target is None:
            return None

        x = float(T_base_tool0_target[0, 3])
        y = float(T_base_tool0_target[1, 3])
        z = float(T_base_tool0_target[2, 3])

        try:
            quaternion = R.from_matrix(
                T_base_tool0_target[:3, :3]
            ).as_quat()

            qx = float(quaternion[0])
            qy = float(quaternion[1])
            qz = float(quaternion[2])
            qw = float(quaternion[3])

        except Exception as exc:
            self.get_logger().error(
                f'Tool0 rotation conversion failed: {exc}'
            )
            return None

        goal = MoveGroup.Goal()

        request = self.configure_request(
            goal,
            velocity,
            acceleration,
            pipeline_id=pipeline_id,
            planner_id=planner_id
        )

        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'tool0'
        position_constraint.weight = 1.0

        region = BoundingVolume()

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [
            float(position_tolerance)
        ]

        sphere_pose = Pose()
        sphere_pose.position.x = x
        sphere_pose.position.y = y
        sphere_pose.position.z = z
        sphere_pose.orientation.w = 1.0

        region.primitives.append(
            sphere
        )

        region.primitive_poses.append(
            sphere_pose
        )

        position_constraint.constraint_region = (
            region
        )

        constraints.position_constraints.append(
            position_constraint
        )

        orientation_constraint = (
            OrientationConstraint()
        )

        orientation_constraint.header.frame_id = (
            'base_link'
        )

        orientation_constraint.link_name = (
            'tool0'
        )

        orientation_constraint.orientation.x = qx
        orientation_constraint.orientation.y = qy
        orientation_constraint.orientation.z = qz
        orientation_constraint.orientation.w = qw

        orientation_constraint.absolute_x_axis_tolerance = (
            float(orientation_tolerance)
        )

        orientation_constraint.absolute_y_axis_tolerance = (
            float(orientation_tolerance)
        )

        orientation_constraint.absolute_z_axis_tolerance = (
            float(orientation_tolerance)
        )

        orientation_constraint.weight = 1.0

        constraints.orientation_constraints.append(
            orientation_constraint
        )

        request.goal_constraints.append(
            constraints
        )

        return goal

    def plan_goal(
        self,
        goal,
        goal_name
    ):
        if goal is None:
            return None

        self.get_logger().info(
            '=============================='
        )

        self.get_logger().info(
            f'PLAN - {goal_name}'
        )

        self.get_logger().info(
            f'Pipeline = {goal.request.pipeline_id}'
        )

        self.get_logger().info(
            f'Planner = '
            f'{goal.request.planner_id or "(default)"}'
        )

        send_future = (
            self.move_group_client.send_goal_async(
                goal
            )
        )

        if not self.wait_for_future(
            send_future,
            timeout_sec=10.0
        ):
            self.get_logger().error(
                'Planning goal response TIMEOUT'
            )
            return None

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                'Planning goal handle is None'
            )
            return None

        if not goal_handle.accepted:
            self.get_logger().error(
                'Planning goal rejected'
            )
            return None

        self.current_plan_goal_handle = (
            goal_handle
        )

        result_future = (
            goal_handle.get_result_async()
        )

        if not self.wait_for_future(
            result_future,
            timeout_sec=15.0
        ):
            self.get_logger().error(
                'Planning TIMEOUT'
            )

            cancel_future = (
                goal_handle.cancel_goal_async()
            )

            self.wait_for_future(
                cancel_future,
                timeout_sec=3.0
            )

            self.current_plan_goal_handle = None
            return None

        wrapped_result = (
            result_future.result()
        )

        self.current_plan_goal_handle = None

        if wrapped_result is None:
            self.get_logger().error(
                'Planning result is None'
            )
            return None

        result = wrapped_result.result

        if (
            result.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):
            self.get_logger().error(
                'Planning FAILED. '
                f'error_code='
                f'{result.error_code.val}'
            )
            return None

        self.get_logger().info(
            'Planning SUCCESS'
        )

        return result.planned_trajectory

    def execute_trajectory(
        self,
        trajectory,
        holding=True
    ):
        if trajectory is None:
            self.get_logger().error(
                'No trajectory to execute'
            )
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = (
            self.execute_client.send_goal_async(
                goal
            )
        )

        if not self.wait_for_future(
            send_future,
            timeout_sec=5.0
        ):
            self.get_logger().error(
                'Execute goal response TIMEOUT'
            )
            return False

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                'Execute goal handle is None'
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                'Execute goal rejected'
            )
            return False

        self.current_execute_goal_handle = (
            goal_handle
        )

        if not holding:
            self.get_logger().info(
                'Execute goal accepted '
                '(non-blocking)'
            )
            return True

        result_future = (
            goal_handle.get_result_async()
        )

        if not self.wait_for_future(
            result_future,
            timeout_sec=60.0
        ):
            self.get_logger().error(
                'Execution TIMEOUT'
            )

            cancel_future = (
                goal_handle.cancel_goal_async()
            )

            self.wait_for_future(
                cancel_future,
                timeout_sec=3.0
            )

            self.current_execute_goal_handle = None
            return False

        wrapped_result = (
            result_future.result()
        )

        self.current_execute_goal_handle = None

        if wrapped_result is None:
            self.get_logger().error(
                'Execution result is None'
            )
            return False

        result = wrapped_result.result

        if (
            result.error_code.val
            == MoveItErrorCodes.SUCCESS
        ):
            self.get_logger().info(
                'Execution SUCCESS'
            )
            self.print_actual_tool0_pose()
            return True

        self.get_logger().error(
            'Execution FAILED. '
            f'error_code='
            f'{result.error_code.val}'
        )

        return False

    def ur_move_joints_ptp(
        self,
        joints,
        velocity,
        acceleration,
        holding=True
    ):
        goal = self.create_joint_goal(
            joints,
            velocity,
            acceleration,
            pipeline_id=(
                'pilz_industrial_motion_planner'
            ),
            planner_id='PTP'
        )

        trajectory = self.plan_goal(
            goal,
            'Pilz PTP Joint'
        )

        if trajectory is None:
            return False

        return self.execute_trajectory(
            trajectory,
            holding=holding
        )

    def ur_move_joints_ompl(
        self,
        joints,
        velocity,
        acceleration,
        holding=True
    ):
        goal = self.create_joint_goal(
            joints,
            velocity,
            acceleration,
            pipeline_id='ompl',
            planner_id=''
        )

        trajectory = self.plan_goal(
            goal,
            'OMPL Joint'
        )

        if trajectory is None:
            return False

        return self.execute_trajectory(
            trajectory,
            holding=holding
        )

    def ur_move_pose_ptp(
        self,
        pose,
        velocity,
        acceleration,
        holding=True
    ):
        goal = self.create_pose_goal(
            pose,
            velocity,
            acceleration,
            pipeline_id=(
                'pilz_industrial_motion_planner'
            ),
            planner_id='PTP'
        )

        trajectory = self.plan_goal(
            goal,
            'Pilz PTP Pose'
        )

        if trajectory is None:
            return False

        return self.execute_trajectory(
            trajectory,
            holding=holding
        )

    def ur_move_pose_lin(
        self,
        pose,
        velocity,
        acceleration,
        holding=True
    ):
        goal = self.create_pose_goal(
            pose,
            velocity,
            acceleration,
            pipeline_id=(
                'pilz_industrial_motion_planner'
            ),
            planner_id='LIN'
        )

        trajectory = self.plan_goal(
            goal,
            'Pilz LIN Pose'
        )

        if trajectory is None:
            return False

        return self.execute_trajectory(
            trajectory,
            holding=holding
        )

    def ur_move_pose_ompl(
        self,
        pose,
        velocity,
        acceleration,
        holding=True
    ):
        goal = self.create_pose_goal(
            pose,
            velocity,
            acceleration,
            pipeline_id='ompl',
            planner_id=''
        )

        trajectory = self.plan_goal(
            goal,
            'OMPL Pose'
        )

        if trajectory is None:
            return False

        return self.execute_trajectory(
            trajectory,
            holding=holding
        )


    # ========================================================
    # 取得目前 UR Active TCP Pose
    #
    # 直接讀 UR Controller 發布的 tool0_controller。
    # 這個 frame 會反映 Teach Pendant 的 Active TCP。
    #
    # 回傳格式刻意跟 HIWIN 一樣：
    # [X_mm, Y_mm, Z_mm, Rx_deg, Ry_deg, Rz_deg]
    # ========================================================

    def get_current_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0_controller',
                Time(),
                timeout=Duration(seconds=1.0)
            )

        except Exception as exc:
            self.get_logger().error(
                f'Cannot get current robot pose: {exc}'
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        try:
            euler = R.from_quat([
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w
            ]).as_euler(
                'xyz',
                degrees=True
            )

        except Exception as exc:
            self.get_logger().error(
                f'Quaternion to Euler failed: {exc}'
            )
            return None

        return [
            float(translation.x) * 1000.0,
            float(translation.y) * 1000.0,
            float(translation.z) * 1000.0,
            float(euler[0]),
            float(euler[1]),
            float(euler[2]),
        ]

    def cancel_current_goals(self):
        if (
            self.current_plan_goal_handle
            is not None
        ):
            try:
                cancel_future = (
                    self.current_plan_goal_handle
                    .cancel_goal_async()
                )

                self.wait_for_future(
                    cancel_future,
                    timeout_sec=3.0
                )

            except Exception as exc:
                self.get_logger().error(
                    f'Cancel PLAN failed: {exc}'
                )

            self.current_plan_goal_handle = None

        if (
            self.current_execute_goal_handle
            is not None
        ):
            try:
                cancel_future = (
                    self.current_execute_goal_handle
                    .cancel_goal_async()
                )

                self.wait_for_future(
                    cancel_future,
                    timeout_sec=3.0
                )

            except Exception as exc:
                self.get_logger().error(
                    f'Cancel EXECUTE failed: {exc}'
                )

            self.current_execute_goal_handle = None

        return True

    def stop_motion(self):
        return self.cancel_current_goals()