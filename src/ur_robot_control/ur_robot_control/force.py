#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import (
    WrenchStamped,
    PoseStamped,
)

from moveit_msgs.action import (
    MoveGroup,
    ExecuteTrajectory,
)

from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
)

from shape_msgs.msg import SolidPrimitive

from tf2_ros import (
    Buffer,
    TransformListener,
)
from std_msgs.msg import String


class URForceInsertTest(Node):

    def __init__(self):
        super().__init__(
            'ur_force_insert_test'
        )

        # ============================================================
        # Test parameters
        # ============================================================

        # 沿 base_link -Z 往下多少
        self.insert_distance_m = 0.05
        # 停止後往 base_link +Z 回退
        self.retreat_distance_m = 0.05

        # MoveIt scaling
        self.velocity_scale = 0.06
        self.acceleration_scale = 0.06

        # Force threshold
        self.force_limit_n = 20.0

        # baseline 幾筆
        self.baseline_sample_count = 50

        # 連續幾筆超過 threshold 才停止
        self.force_trigger_count_required = 3
        # stop 後等待多久再 retreat
        self.retreat_delay_sec = 2.0

        self.retreat_timer = None

        # ============================================================
        # Force
        # ============================================================

        self.latest_force = None

        self.force_samples = []
        self.force_baseline = None

        self.force_trigger_count = 0

        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.force_callback,
            10
        )

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # MoveIt PLAN
        # ============================================================

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        # ============================================================
        # MoveIt EXECUTE
        # ============================================================

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        self.trajectory_event_pub = self.create_publisher(
            String,
            '/trajectory_execution_event',
            10
        )

        self.plan_goal_handle = None
        self.execute_goal_handle = None

        self.motion_started = False
        self.motion_finished = False

        self.stop_requested = False
        self.retreat_started = False

        # ============================================================
        # Main timer
        # ============================================================

        self.timer = self.create_timer(
            0.1,
            self.main_loop
        )

        self.get_logger().info(
            '==================================='
        )

        self.get_logger().info(
            'UR Force Insert Test started'
        )

        self.get_logger().info(
            f'Insert distance = '
            f'{self.insert_distance_m * 1000:.1f} mm'
        )

        self.get_logger().info(
            f'Force limit = '
            f'{self.force_limit_n:.1f} N'
        )

        self.get_logger().info(
            '==================================='
        )

    # ================================================================
    # Force
    # ================================================================

    def force_callback(
        self,
        msg
    ):

        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        self.latest_force = [
            fx,
            fy,
            fz
        ]

        # ============================================================
        # Baseline
        # ============================================================

        if self.force_baseline is None:

            self.force_samples.append(
                [fx, fy, fz]
            )

            if (
                len(self.force_samples)
                >= self.baseline_sample_count
            ):

                n = len(
                    self.force_samples
                )

                baseline_fx = (
                    sum(
                        sample[0]
                        for sample
                        in self.force_samples
                    )
                    / n
                )

                baseline_fy = (
                    sum(
                        sample[1]
                        for sample
                        in self.force_samples
                    )
                    / n
                )

                baseline_fz = (
                    sum(
                        sample[2]
                        for sample
                        in self.force_samples
                    )
                    / n
                )

                self.force_baseline = [
                    baseline_fx,
                    baseline_fy,
                    baseline_fz
                ]

                self.get_logger().info(
                    'Force baseline ready'
                )

                self.get_logger().info(
                    f'Baseline = '
                    f'[{baseline_fx:.2f}, '
                    f'{baseline_fy:.2f}, '
                    f'{baseline_fz:.2f}] N'
                )

            return

        # ============================================================
        # 還沒 execute
        # ============================================================

        if self.execute_goal_handle is None:
            return

        if self.motion_finished:
            return
        if self.retreat_started:
            return

        if self.stop_requested:
            return

        # ============================================================
        # Delta force
        # ============================================================

        dfx = (
            fx
            - self.force_baseline[0]
        )

        dfy = (
            fy
            - self.force_baseline[1]
        )

        dfz = (
            fz
            - self.force_baseline[2]
        )

        delta_force = math.sqrt(
            dfx * dfx
            + dfy * dfy
            + dfz * dfz
        )

        self.get_logger().info(
            f'dF = '
            f'[{dfx:+.2f}, '
            f'{dfy:+.2f}, '
            f'{dfz:+.2f}] N | '
            f'|dF| = '
            f'{delta_force:.2f} N'
        )

        # ============================================================
        # Threshold
        # ============================================================

        if (
            abs(dfz)
            > self.force_limit_n
        ):

            self.force_trigger_count += 1

        else:

            self.force_trigger_count = 0

        # ============================================================
        # Stop
        # ============================================================

        if (
            self.force_trigger_count
            >= self.force_trigger_count_required
        ):

            self.get_logger().warning(
                'Force limit exceeded!'
            )

            self.get_logger().warning(
                f'dFx={dfx:.2f}, '
                f'dFy={dfy:.2f}, '
                f'dFz={dfz:.2f}, '
                f'|dF|={delta_force:.2f} N'
            )

            self.stop_trajectory()

    # ================================================================
    # Main
    # ================================================================

    def main_loop(self):

        if self.motion_started:
            return

        if self.force_baseline is None:
            return

        self.motion_started = True

        self.get_logger().info(
            'Baseline complete'
        )

        self.start_plan()

    # ================================================================
    # Current tool0
    # ================================================================

    def get_current_tool0_pose(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'base_link',
                    'tool0',
                    rclpy.time.Time()
                )
            )

        except Exception as exc:

            self.get_logger().error(
                f'TF error: {exc}'
            )

            return None

        pose = PoseStamped()

        pose.header.frame_id = (
            'base_link'
        )

        pose.pose.position.x = (
            transform.transform.translation.x
        )

        pose.pose.position.y = (
            transform.transform.translation.y
        )

        pose.pose.position.z = (
            transform.transform.translation.z
        )

        pose.pose.orientation = (
            transform.transform.rotation
        )

        return pose

    # ================================================================
    # PLAN
    # ================================================================

    def start_plan(self):

        current_pose = (
            self.get_current_tool0_pose()
        )

        if current_pose is None:

            self.motion_started = False
            return

        target_pose = PoseStamped()

        target_pose.header.frame_id = (
            'base_link'
        )

        target_pose.pose.position.x = (
            current_pose.pose.position.x
        )

        target_pose.pose.position.y = (
            current_pose.pose.position.y
        )

        target_pose.pose.position.z = (
            current_pose.pose.position.z
            - self.insert_distance_m
        )

        target_pose.pose.orientation = (
            current_pose.pose.orientation
        )

        self.get_logger().info(
            'Current tool0: '
            f'X={current_pose.pose.position.x:.4f}, '
            f'Y={current_pose.pose.position.y:.4f}, '
            f'Z={current_pose.pose.position.z:.4f}'
        )

        self.get_logger().info(
            'Target tool0: '
            f'X={target_pose.pose.position.x:.4f}, '
            f'Y={target_pose.pose.position.y:.4f}, '
            f'Z={target_pose.pose.position.z:.4f}'
        )

        self.send_plan_goal(
            target_pose
        )

    # ================================================================
    # Create PLAN goal
    # ================================================================

    def send_plan_goal(
        self,
        target_pose
    ):

        if not (
            self.move_group_client
            .wait_for_server(
                timeout_sec=5.0
            )
        ):

            self.get_logger().error(
                '/move_action unavailable'
            )

            return

        goal = MoveGroup.Goal()

        request = goal.request

        request.group_name = (
            'ur_manipulator'
        )

        request.pipeline_id = (
            'pilz_industrial_motion_planner'
        )

        request.planner_id = 'LIN'

        request.num_planning_attempts = 1

        request.allowed_planning_time = (
            5.0
        )

        request.max_velocity_scaling_factor = (
            self.velocity_scale
        )

        request.max_acceleration_scaling_factor = (
            self.acceleration_scale
        )

        request.start_state.is_diff = True

        # ============================================================
        # Position
        # ============================================================

        position_constraint = (
            PositionConstraint()
        )

        position_constraint.header.frame_id = (
            'base_link'
        )

        position_constraint.link_name = (
            'tool0'
        )

        position_constraint.weight = 1.0

        box = SolidPrimitive()

        box.type = (
            SolidPrimitive.BOX
        )

        box.dimensions = [
            0.001,
            0.001,
            0.001
        ]

        region = BoundingVolume()

        region.primitives.append(
            box
        )

        region.primitive_poses.append(
            target_pose.pose
        )

        position_constraint.constraint_region = (
            region
        )

        # ============================================================
        # Orientation
        # ============================================================

        orientation_constraint = (
            OrientationConstraint()
        )

        orientation_constraint.header.frame_id = (
            'base_link'
        )

        orientation_constraint.link_name = (
            'tool0'
        )

        orientation_constraint.orientation = (
            target_pose.pose.orientation
        )

        orientation_constraint.absolute_x_axis_tolerance = (
            0.01
        )

        orientation_constraint.absolute_y_axis_tolerance = (
            0.01
        )

        orientation_constraint.absolute_z_axis_tolerance = (
            0.01
        )

        orientation_constraint.weight = 1.0

        # ============================================================
        # Constraints
        # ============================================================

        constraints = Constraints()

        constraints.position_constraints.append(
            position_constraint
        )

        constraints.orientation_constraints.append(
            orientation_constraint
        )

        request.goal_constraints.append(
            constraints
        )

        # ============================================================
        # IMPORTANT
        #
        # /move_action 只 PLAN
        # ============================================================

        goal.planning_options.plan_only = (
            True
        )

        goal.planning_options.look_around = (
            False
        )

        goal.planning_options.replan = (
            False
        )

        self.get_logger().info(
            'Sending Pilz LIN PLAN...'
        )

        future = (
            self.move_group_client
            .send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.plan_goal_callback
        )

    # ================================================================
    # PLAN accepted
    # ================================================================

    def plan_goal_callback(
        self,
        future
    ):

        self.plan_goal_handle = (
            future.result()
        )

        if not (
            self.plan_goal_handle.accepted
        ):

            self.get_logger().error(
                'Planning goal rejected'
            )

            self.motion_finished = True

            return

        self.get_logger().info(
            'Planning goal accepted'
        )

        result_future = (
            self.plan_goal_handle
            .get_result_async()
        )

        result_future.add_done_callback(
            self.plan_result_callback
        )

    # ================================================================
    # PLAN result
    # ================================================================

    def plan_result_callback(
        self,
        future
    ):

        wrapped_result = (
            future.result()
        )

        result = (
            wrapped_result.result
        )

        self.plan_goal_handle = None

        if (
            result.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):

            self.get_logger().error(
                'Planning failed: '
                f'{result.error_code.val}'
            )

            self.motion_finished = True

            return

        self.get_logger().info(
            'Pilz LIN planning SUCCESS'
        )

        trajectory = (
            result.planned_trajectory
        )

        self.start_execute(
            trajectory
        )

    # ================================================================
    # EXECUTE
    # ================================================================

    def start_execute(
        self,
        trajectory
    ):

        if not (
            self.execute_client
            .wait_for_server(
                timeout_sec=5.0
            )
        ):

            self.get_logger().error(
                '/execute_trajectory unavailable'
            )

            self.motion_finished = True
            return

        goal = (
            ExecuteTrajectory.Goal()
        )

        goal.trajectory = trajectory

        self.get_logger().info(
            'Sending trajectory to '
            '/execute_trajectory...'
        )

        future = (
            self.execute_client
            .send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.execute_goal_callback
        )

    # ================================================================
    # EXECUTE accepted
    # ================================================================

    def execute_goal_callback(
        self,
        future
    ):

        self.execute_goal_handle = (
            future.result()
        )

        if not (
            self.execute_goal_handle.accepted
        ):

            self.get_logger().error(
                'Execution goal rejected'
            )

            self.execute_goal_handle = None
            self.motion_finished = True

            return

        self.get_logger().info(
            'Execution goal accepted'
        )

        result_future = (
            self.execute_goal_handle
            .get_result_async()
        )

        result_future.add_done_callback(
            self.execute_result_callback
        )

    # ================================================================
    # EXECUTE result
    # ================================================================

    def execute_result_callback(
        self,
        future
    ):

        wrapped_result = (
            future.result()
        )

        result = (
            wrapped_result.result
        )

        status = (
            wrapped_result.status
        )

        self.execute_goal_handle = None

        self.get_logger().info(
            f'Execution finished, '
            f'status={status}'
        )

        self.get_logger().info(
            f'MoveIt error code = '
            f'{result.error_code.val}'
        )

        # ============================================================
        # 如果是 force stop 造成 execution 中止
        # 就開始測試下一段 retreat motion
        # ============================================================

        if (
            self.stop_requested
            and not self.retreat_started
        ):

            self.retreat_started = True

            self.get_logger().warning(
                'Previous trajectory stopped.'
            )

            self.get_logger().warning(
                f'Waiting {self.retreat_delay_sec:.1f} s '
                'before retreat...'
            )

            self.retreat_timer = self.create_timer(
                self.retreat_delay_sec,
                self.start_retreat_after_delay
            )

            return
        # 第二段也結束了
        self.motion_finished = True

    # ================================================================
    # MoveIt trajectory STOP
    # ================================================================

    def stop_trajectory(self):

        if self.stop_requested:
            return

        self.stop_requested = True

        self.get_logger().warning(
            'Publishing MoveIt trajectory STOP event...'
        )

        msg = String()
        msg.data = 'stop'

        self.trajectory_event_pub.publish(
            msg
        )

        self.get_logger().warning(
            'STOP event published to '
            '/trajectory_execution_event'
        )

    def start_retreat_after_delay(self):

        # timer 只執行一次
        if self.retreat_timer is not None:

            self.retreat_timer.cancel()
            self.retreat_timer = None

        self.get_logger().warning(
            'Retreat delay finished.'
        )

        self.start_retreat()

    # ================================================================
    # Retreat test
    # 停止後重新取得目前 tool0，再往 base_link +Z
    # ================================================================

    def start_retreat(self):

        current_pose = (
            self.get_current_tool0_pose()
        )

        if current_pose is None:

            self.get_logger().error(
                'Cannot get current tool0 '
                'for retreat'
            )

            self.motion_finished = True

            return

        retreat_pose = PoseStamped()

        retreat_pose.header.frame_id = (
            'base_link'
        )

        retreat_pose.pose.position.x = (
            current_pose.pose.position.x
        )

        retreat_pose.pose.position.y = (
            current_pose.pose.position.y
        )

        retreat_pose.pose.position.z = (
            current_pose.pose.position.z
            + self.retreat_distance_m
        )

        retreat_pose.pose.orientation = (
            current_pose.pose.orientation
        )

        self.get_logger().info(
            'Current stopped pose: '
            f'X={current_pose.pose.position.x:.4f}, '
            f'Y={current_pose.pose.position.y:.4f}, '
            f'Z={current_pose.pose.position.z:.4f}'
        )

        self.get_logger().info(
            'Retreat target: '
            f'X={retreat_pose.pose.position.x:.4f}, '
            f'Y={retreat_pose.pose.position.y:.4f}, '
            f'Z={retreat_pose.pose.position.z:.4f}'
        )

        # 重要：
        # 第二段運動不要再讓 force callback 觸發 stop
        self.stop_requested = False

        self.send_plan_goal(
            retreat_pose
        )

def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        URForceInsertTest()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()