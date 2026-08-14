#!/usr/bin/env python3

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

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
from shape_msgs.msg import SolidPrimitive


class URMoveItTest(Node):
    """Test UR5e MoveIt 2 with joint and Cartesian pose goals."""

    def __init__(self):
        super().__init__('ur_moveit_test')

        # ============================================================
        # MoveIt Action Client
        # /move_action 只負責規劃，/execute_trajectory 負責執行規劃結果。
        # 另外記錄目前的 Goal Handle，讓取消或離開程式時可以正常送 cancel。
        # ============================================================
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.current_plan_goal_handle = None
        self.current_execute_goal_handle = None

        self.get_logger().info('Waiting for /move_action...')
        if not self.move_group_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('/move_action server not available')
        self.get_logger().info('Connected to /move_action.')

        self.get_logger().info('Waiting for /execute_trajectory...')
        if not self.execute_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('/execute_trajectory server not available')
        self.get_logger().info('Connected to /execute_trajectory.')
        self.get_logger().info('OMPL pipeline = ompl')
        self.get_logger().info('Pilz pipeline = pilz_industrial_motion_planner')

    # ============================================================
    # 共用 Planning Request 設定
    # Joint Goal 與 Pose Goal 都使用 ur_manipulator，並從目前 robot state
    # 開始規劃。plan_only=True，所以 /move_action 絕對不會直接執行手臂。
    # ============================================================
    @staticmethod
    def configure_request(goal, pipeline_id='ompl', planner_id=''):
        request = goal.request
        request.group_name = 'ur_manipulator'
        request.start_state.is_diff = True

        # 明確指定 MoveIt 要使用哪一條 planning pipeline。
        #
        # OMPL:
        #   pipeline_id = 'ompl'
        #   planner_id  = ''  -> 使用 OMPL 預設 planner
        #
        # Pilz:
        #   pipeline_id = 'pilz_industrial_motion_planner'
        #   planner_id  = 'PTP' / 'LIN' / 'CIRC'
        request.pipeline_id = pipeline_id
        request.planner_id = planner_id

        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        return request

    # ============================================================
    # 建立 Joint Planning Goal
    # joint_positions 必須有 6 個值，單位為 rad。
    # 每一軸會轉成 JointConstraint，MoveIt 再依目前姿態規劃 trajectory。
    # ============================================================
    def create_joint_goal(
        self,
        joint_positions,
        pipeline_id='ompl',
        planner_id='',
    ):
        if len(joint_positions) != 6:
            self.get_logger().error('UR5e requires exactly 6 joints.')
            return None

        goal = MoveGroup.Goal()
        request = self.configure_request(
            goal,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
        )
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        constraints = Constraints()
        for joint_name, joint_position in zip(joint_names, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_position)
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        request.goal_constraints.append(constraints)
        return goal

    # ============================================================
    # 建立 Cartesian Pose Planning Goal
    # XYZ 是 base_link 座標系下的位置，單位為 meter；
    # Quaternion 是 tool0 的目標姿態。
    #
    # PositionConstraint 用一個小球描述容許的位置範圍，預設半徑 5 mm。
    # OrientationConstraint 的容許誤差單位是 rad，預設約 2.9 度。
    # ============================================================
    def create_pose_goal(
        self,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        position_tolerance=0.005,
        orientation_tolerance=0.05,
        pipeline_id='ompl',
        planner_id='',
    ):
        quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if quaternion_norm < 1e-8:
            self.get_logger().error('Quaternion cannot be zero.')
            return None

        qx /= quaternion_norm
        qy /= quaternion_norm
        qz /= quaternion_norm
        qw /= quaternion_norm

        goal = MoveGroup.Goal()
        request = self.configure_request(
            goal,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
        )
        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'tool0'
        position_constraint.weight = 1.0

        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(position_tolerance)]

        sphere_pose = Pose()
        sphere_pose.position.x = float(x)
        sphere_pose.position.y = float(y)
        sphere_pose.position.z = float(z)
        sphere_pose.orientation.w = 1.0

        region.primitives.append(sphere)
        region.primitive_poses.append(sphere_pose)
        position_constraint.constraint_region = region
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'base_link'
        orientation_constraint.link_name = 'tool0'
        orientation_constraint.orientation.x = float(qx)
        orientation_constraint.orientation.y = float(qy)
        orientation_constraint.orientation.z = float(qz)
        orientation_constraint.orientation.w = float(qw)
        orientation_constraint.absolute_x_axis_tolerance = float(orientation_tolerance)
        orientation_constraint.absolute_y_axis_tolerance = float(orientation_tolerance)
        orientation_constraint.absolute_z_axis_tolerance = float(orientation_tolerance)
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        request.goal_constraints.append(constraints)
        return goal


    # ============================================================
    # 建立 Pilz CIRC Cartesian Goal
    #
    # CIRC 除了終點 Pose，還必須提供一個圓弧條件：
    #   point_type='interim' -> 圓弧必須經過 interim 點
    #   point_type='center'  -> 指定圓心
    #
    # arc_x/y/z 是 base_link 座標系下的位置，單位為 meter。
    # ============================================================
    def create_circ_goal(
        self,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        arc_x,
        arc_y,
        arc_z,
        point_type='interim',
        position_tolerance=0.005,
        orientation_tolerance=0.05,
    ):
        if point_type not in ('interim', 'center'):
            self.get_logger().error(
                "CIRC point_type must be 'interim' or 'center'."
            )
            return None

        goal = self.create_pose_goal(
            x,
            y,
            z,
            qx,
            qy,
            qz,
            qw,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            pipeline_id='pilz_industrial_motion_planner',
            planner_id='CIRC',
        )

        if goal is None:
            return None

        # Pilz CIRC 會從 path_constraints.name 判斷這個點是
        # interim 還是 center。
        path_constraints = Constraints()
        path_constraints.name = point_type

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'tool0'
        position_constraint.weight = 1.0

        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(position_tolerance)]

        arc_pose = Pose()
        arc_pose.position.x = float(arc_x)
        arc_pose.position.y = float(arc_y)
        arc_pose.position.z = float(arc_z)
        arc_pose.orientation.w = 1.0

        region.primitives.append(sphere)
        region.primitive_poses.append(arc_pose)
        position_constraint.constraint_region = region
        path_constraints.position_constraints.append(position_constraint)

        goal.request.path_constraints = path_constraints
        return goal

    # ============================================================
    # Step 1：只做 Planning
    # 這個函式同時供 Joint Goal 與 Pose Goal 使用。
    # 成功時回傳 planned_trajectory；失敗或 timeout 時回傳 None。
    # ============================================================
    def plan_goal(self, goal, goal_name):
        if goal is None:
            return None

        self.get_logger().info('==============================')
        self.get_logger().info(f'PLAN ONLY - {goal_name}')
        self.get_logger().info(
            f'Pipeline = {goal.request.pipeline_id or "(default)"}'
        )
        self.get_logger().info(
            f'Planner  = {goal.request.planner_id or "(default)"}'
        )
        self.get_logger().info('Sending PLAN request...')

        send_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done():
            self.get_logger().error('Planning goal response TIMEOUT.')
            return None

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error('Planning goal handle is None.')
            return None
        if not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected.')
            return None

        self.current_plan_goal_handle = goal_handle
        self.get_logger().info('Planning goal accepted.')
        self.get_logger().info('Planning...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)

        if not result_future.done():
            self.get_logger().error('Planning TIMEOUT.')
            self.get_logger().warn('Cancelling planning goal...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
            self.current_plan_goal_handle = None
            return None

        wrapped_result = result_future.result()
        result = wrapped_result.result
        self.current_plan_goal_handle = None

        self.get_logger().info(f'Planning Action status = {wrapped_result.status}')
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'Planning FAILED. error_code={result.error_code.val}')
            return None

        trajectory = result.planned_trajectory
        point_count = len(trajectory.joint_trajectory.points)
        self.get_logger().info('Planning SUCCESS!')
        self.get_logger().info(f'Trajectory points = {point_count}')
        self.get_logger().warn('ROBOT HAS NOT BEEN EXECUTED.')
        return trajectory

    # ============================================================
    # Step 2：Execute
    # 只有呼叫這個函式才會真的送 /execute_trajectory。
    # 明確指定目前已驗證可用的 scaled_joint_trajectory_controller。
    # ============================================================
    def execute_trajectory(self, trajectory):
        if trajectory is None:
            self.get_logger().error('No trajectory to execute.')
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        # goal.controller_names = ['scaled_joint_trajectory_controller']

        self.get_logger().warn('==============================')
        self.get_logger().warn('EXECUTE REQUEST IS BEING SENT')
        self.get_logger().warn('Controller: scaled_joint_trajectory_controller')
        self.get_logger().warn('==============================')

        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)

        if not send_future.done():
            self.get_logger().error('Execute goal response TIMEOUT.')
            return False

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error('Execute goal handle is None.')
            return False
        if not goal_handle.accepted:
            self.get_logger().error('Execute goal rejected.')
            return False

        self.current_execute_goal_handle = goal_handle
        self.get_logger().warn('Execute goal accepted.')
        self.get_logger().warn('Executing...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        if not result_future.done():
            self.get_logger().error('Execution TIMEOUT.')
            self.get_logger().warn('Cancelling execution...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
            self.current_execute_goal_handle = None
            return False

        wrapped_result = result_future.result()
        result = wrapped_result.result
        self.current_execute_goal_handle = None

        self.get_logger().info(f'Execution Action status = {wrapped_result.status}')
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Execution SUCCESS!')
            return True

        self.get_logger().error(f'Execution FAILED. error_code={result.error_code.val}')
        return False

    # ============================================================
    # 取消目前這支程式持有的 Planning / Execute Goal。
    # 這只會取消本程式記住的 Goal Handle，不會任意取消其他節點的 Goal。
    # ============================================================
    def cancel_current_goals(self):
        self.get_logger().warn('Trying to cancel current goals...')

        if self.current_plan_goal_handle is not None:
            self.get_logger().warn('Cancelling current PLAN goal...')
            try:
                cancel_future = self.current_plan_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
            except Exception as error:
                self.get_logger().error(f'Failed to cancel PLAN goal: {error}')
            self.current_plan_goal_handle = None

        if self.current_execute_goal_handle is not None:
            self.get_logger().warn('Cancelling current EXECUTE goal...')
            try:
                cancel_future = self.current_execute_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
            except Exception as error:
                self.get_logger().error(f'Failed to cancel EXECUTE goal: {error}')
            self.current_execute_goal_handle = None

        self.get_logger().info('Cancel request finished.')

    # ============================================================
    # Plan → 人工確認 → Execute
    # Joint 與 Pose 都走同一個人工確認流程。規劃成功後必須輸入大寫 E，
    # 才會送 ExecuteTrajectory；其他任何輸入都直接丟棄 trajectory。
    # ============================================================
    def plan_and_confirm(self, goal, goal_name):
        trajectory = self.plan_goal(goal, goal_name)
        if trajectory is None:
            self.get_logger().error('Planning failed.')
            return False

        print(
            '\n========================================\n'
            '        PLANNING SUCCESS\n'
            '========================================\n'
            'Robot has NOT been executed.\n'
            'Check the planned path first.\n\n'
            'Type E to allow execution.\n'
            'Anything else = cancel\n'
            '========================================'
        )

        command = input('Confirmation: ').strip()
        if command != 'E':
            self.get_logger().info('Execution cancelled by user.')
            self.get_logger().info('Trajectory discarded.')
            return False

        return self.execute_trajectory(trajectory)


# ================================================================
# 顯示 Joint / Cartesian Pose Target。
# ================================================================
def print_joint_goal(name, joint_goal):
    print(f'\n==============================\n{name}\n==============================')
    for index, angle in enumerate(joint_goal):
        print(f'J{index + 1} = {math.degrees(angle):7.2f} deg')
    print()


def print_cartesian_goal(name, pose):
    print(
        f'\n==============================\n{name}\n==============================\n'
        f"X  = {pose['x']:.4f} m\n"
        f"Y  = {pose['y']:.4f} m\n"
        f"Z  = {pose['z']:.4f} m\n"
        f"qx = {pose['qx']:.5f}\n"
        f"qy = {pose['qy']:.5f}\n"
        f"qz = {pose['qz']:.5f}\n"
        f"qw = {pose['qw']:.5f}\n"
    )


# ================================================================
# main
# 目前建議搭配 use_mock_hardware:=true 測試。
#
# Joint Pose 1~6 保留原本測試點。
# Cartesian Pose 1 是新增的末端目標點；XYZ 相對 base_link，姿態控制 tool0。
# 第一次測試時，建議把這組數值換成目前 tool0 Pose，再只小幅修改 XYZ。
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = URMoveItTest()

    #初始位
    pose_1 = [
        math.radians(90),
        math.radians(-90),
        math.radians(90),
        math.radians(-90),
        math.radians(-90),
        math.radians(0),
    ]
    # PHOTO_POSE = [
    #     1.612801432609558,       # shoulder_pan_joint
    #     -1.5463765303241175,     # shoulder_lift_joint
    #     1.464296817779541,       # elbow_joint
    #     -1.5707486311541956,     # wrist_1_joint
    #     -1.5208280722247522,     # wrist_2_joint
    #     -3.5587941304981996e-05, # wrist_3_joint
    # ]
    pose_2 = [
        1.612801432609558,       # shoulder_pan_joint
        -1.5463765303241175,     # shoulder_lift_joint
        1.464296817779541,       # elbow_joint
        -1.5707486311541956,     # wrist_1_joint
        -1.5208280722247522,     # wrist_2_joint
        -3.5587941304981996e-05, # wrist_3_joint
    ]

    pose_3 = [
        math.radians(0),
        math.radians(-90),
        math.radians(-90),
        math.radians(-90),
        math.radians(0),
        math.radians(0),
    ]

    pose_4 = [
        math.radians(0),
        math.radians(-90),
        math.radians(0),
        math.radians(-45),
        math.radians(0),
        math.radians(0),
    ]

    pose_5 = [
        math.radians(0),
        math.radians(-90),
        math.radians(0),
        math.radians(-90),
        math.radians(-90),
        math.radians(0),
    ]

    pose_6 = [
        math.radians(0),
        math.radians(-90),
        math.radians(0),
        math.radians(-90),
        math.radians(0),
        math.radians(-90),
    ]

    joint_poses = {
        '1': ('Joint Pose 1', pose_1),
        '2': ('Joint Pose 2', pose_2),
        '3': ('Joint Pose 3', pose_3),
        '4': ('Joint Pose 4', pose_4),
        '5': ('Joint Pose 5', pose_5),
        '6': ('Joint Pose 6', pose_6),
    }

    # Cartesian Pose 測試點。
    # 這組只是範例，第一次請先用目前 tool0 的 Pose 作基準，再小幅調整 XYZ。
    cartesian_pose_1 = {
        'x': 0.016,
        'y': 0.232,
        'z': 1.079,
        'qx': -0.706,
        'qy': 0.047,
        'qz': -0.047,
        'qw': 0.705,
    }
    cartesian_pose_2 = {
        'x': 0.016,
        'y': 0.232,
        'z': 1.079,
        'qx': -0.706,
        'qy': 0.047,
        'qz': -0.047,
        'qw': 0.705,
    }

    # ============================================================
    # Motion mode
    #
    # O = OMPL
    # P = Pilz PTP
    # L = Pilz LIN
    # C = Pilz CIRC
    #
    # 為了讓行為清楚：
    # - Joint Pose 1~6：允許 OMPL 或 PTP。
    # - Cartesian Pose 1：允許 OMPL、PTP、LIN。
    # - CIRC：使用 Cartesian Pose 1 當終點，再另外指定 interim 點。
    # ============================================================
    motion_mode = 'OMPL'

    # CIRC 範例 interim 點。
    # 注意：這只是程式格式範例；實際測試前請改成「目前 TCP 到目標 TCP」
    # 之間合理且可達的圓弧中繼點。
    circ_interim = {
        'x': 0.068,
        'y': 0.157,
        'z': 0.978,
    }

    try:
        while rclpy.ok():
            print(
                '\n==============================\n'
                'UR MoveIt Test\n'
                '==============================\n'
                f'Current motion mode = {motion_mode}\n\n'
                'm = Change motion mode\n\n'
                '1 = Joint Pose 1\n'
                '2 = Joint Pose 2\n'
                '3 = Joint Pose 3\n'
                '4 = Joint Pose 4\n'
                '5 = Joint Pose 5\n'
                '6 = Joint Pose 6\n'
                '7 = Cartesian Pose 1\n\n'
                'c = Cancel current goal\n'
                'q = Quit\n'
                '=============================='
            )

            command = input('Command: ').strip().lower()

            # ----------------------------------------------------
            # 切換規劃模式
            # ----------------------------------------------------
            if command == 'm':
                print(
                    '\n==============================\n'
                    'Choose Motion Mode\n'
                    '==============================\n'
                    'o = OMPL\n'
                    '    Automatically find a feasible path\n\n'
                    'p = Pilz PTP\n'
                    '    Point-to-point synchronized joint motion\n\n'
                    'l = Pilz LIN\n'
                    '    Move the TCP along a Cartesian straight line\n\n'
                    'r = Pilz CIRC\n'
                    '    Move the TCP along a Cartesian arc\n'
                    '=============================='
                )
                mode_command = input('Mode: ').strip().lower()

                if mode_command == 'o':
                    motion_mode = 'OMPL'
                elif mode_command == 'p':
                    motion_mode = 'PTP'
                elif mode_command == 'l':
                    motion_mode = 'LIN'
                elif mode_command == 'r':
                    motion_mode = 'CIRC'
                else:
                    print('Unknown motion mode.')

                continue

            # ----------------------------------------------------
            # Joint Goal
            # ----------------------------------------------------
            if command in joint_poses:
                pose_name, pose = joint_poses[command]
                print_joint_goal(pose_name, pose)

                if motion_mode == 'OMPL':
                    goal = node.create_joint_goal(
                        pose,
                        pipeline_id='ompl',
                        planner_id='',
                    )
                    node.plan_and_confirm(
                        goal,
                        f'OMPL - {pose_name}',
                    )

                elif motion_mode == 'PTP':
                    goal = node.create_joint_goal(
                        pose,
                        pipeline_id='pilz_industrial_motion_planner',
                        planner_id='PTP',
                    )
                    node.plan_and_confirm(
                        goal,
                        f'Pilz PTP - {pose_name}',
                    )

                else:
                    print(
                        f'\n{motion_mode} is a Cartesian path mode.\n'
                        'Select 7 = Cartesian Pose 1.\n'
                        'Use OMPL or PTP for Joint Pose 1~6.'
                    )

            # ----------------------------------------------------
            # Cartesian Goal
            # ----------------------------------------------------
            elif command == '7':
                print_cartesian_goal('Cartesian Pose 1', cartesian_pose_1)

                if motion_mode == 'OMPL':
                    goal = node.create_pose_goal(
                        cartesian_pose_1['x'],
                        cartesian_pose_1['y'],
                        cartesian_pose_1['z'],
                        cartesian_pose_1['qx'],
                        cartesian_pose_1['qy'],
                        cartesian_pose_1['qz'],
                        cartesian_pose_1['qw'],
                        pipeline_id='ompl',
                        planner_id='',
                    )
                    node.plan_and_confirm(
                        goal,
                        'OMPL - Cartesian Pose 1',
                    )

                elif motion_mode == 'PTP':
                    goal = node.create_pose_goal(
                        cartesian_pose_1['x'],
                        cartesian_pose_1['y'],
                        cartesian_pose_1['z'],
                        cartesian_pose_1['qx'],
                        cartesian_pose_1['qy'],
                        cartesian_pose_1['qz'],
                        cartesian_pose_1['qw'],
                        pipeline_id='pilz_industrial_motion_planner',
                        planner_id='PTP',
                    )
                    node.plan_and_confirm(
                        goal,
                        'Pilz PTP - Cartesian Pose 1',
                    )

                elif motion_mode == 'LIN':
                    goal = node.create_pose_goal(
                        cartesian_pose_1['x'],
                        cartesian_pose_1['y'],
                        cartesian_pose_1['z'],
                        cartesian_pose_1['qx'],
                        cartesian_pose_1['qy'],
                        cartesian_pose_1['qz'],
                        cartesian_pose_1['qw'],
                        pipeline_id='pilz_industrial_motion_planner',
                        planner_id='LIN',
                    )
                    node.plan_and_confirm(
                        goal,
                        'Pilz LIN - Cartesian Pose 1',
                    )

                elif motion_mode == 'CIRC':
                    print(
                        '\nCIRC interim point:\n'
                        f"X = {circ_interim['x']:.4f} m\n"
                        f"Y = {circ_interim['y']:.4f} m\n"
                        f"Z = {circ_interim['z']:.4f} m\n"
                    )

                    goal = node.create_circ_goal(
                        cartesian_pose_1['x'],
                        cartesian_pose_1['y'],
                        cartesian_pose_1['z'],
                        cartesian_pose_1['qx'],
                        cartesian_pose_1['qy'],
                        cartesian_pose_1['qz'],
                        cartesian_pose_1['qw'],
                        circ_interim['x'],
                        circ_interim['y'],
                        circ_interim['z'],
                        point_type='interim',
                    )
                    node.plan_and_confirm(
                        goal,
                        'Pilz CIRC - Cartesian Pose 1',
                    )

            elif command == 'c':
                node.cancel_current_goals()

            elif command == 'q':
                print('Cancelling current goals before exit...')
                node.cancel_current_goals()
                print('Exiting program.')
                break

            else:
                print('Unknown command.')

    except KeyboardInterrupt:
        print('\nKeyboard interrupt.')
        try:
            node.cancel_current_goals()
        except Exception:
            pass

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()