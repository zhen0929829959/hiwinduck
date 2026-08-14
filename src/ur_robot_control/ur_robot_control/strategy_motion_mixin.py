#!/usr/bin/env python3

from geometry_msgs.msg import Twist


class StrategyMotionMixin:

    def create_pose(
        self,
        x,
        y,
        z,
        rx,
        ry,
        rz
    ):
        pose = Twist()

        pose.linear.x = float(x)
        pose.linear.y = float(y)
        pose.linear.z = float(z)

        pose.angular.x = float(rx)
        pose.angular.y = float(ry)
        pose.angular.z = float(rz)

        return pose

    def pose_list_to_twist(
        self,
        pose_values
    ):
        if (
            pose_values is None
            or len(pose_values) < 6
        ):
            self.get_logger().error(
                'Pose requires six values'
            )
            return None

        return self.create_pose(
            pose_values[0],
            pose_values[1],
            pose_values[2],
            pose_values[3],
            pose_values[4],
            pose_values[5]
        )

    # ========================================================
    # Joint
    # ========================================================

    def move_joints(
        self,
        joints,
        velocity,
        acceleration,
        holding=True
    ):
        # 舊名稱保留，預設 Pilz PTP
        return self.ur_move_joints_ptp(
            joints,
            velocity,
            acceleration,
            holding
        )

    def move_joints_ptp(
        self,
        joints,
        velocity,
        acceleration,
        holding=True
    ):
        return self.ur_move_joints_ptp(
            joints,
            velocity,
            acceleration,
            holding
        )

    def move_joints_ompl(
        self,
        joints,
        velocity,
        acceleration,
        holding=True
    ):
        return self.ur_move_joints_ompl(
            joints,
            velocity,
            acceleration,
            holding
        )

    # ========================================================
    # Cartesian Pose
    # ========================================================

    def move_pose(
        self,
        pose,
        velocity,
        acceleration,
        holding=True
    ):
        # 舊名稱保留，預設 Pilz PTP
        return self.ur_move_pose_ptp(
            pose,
            velocity,
            acceleration,
            holding
        )

    def move_pose_ptp(
        self,
        pose,
        velocity,
        acceleration,
        holding=True
    ):
        return self.ur_move_pose_ptp(
            pose,
            velocity,
            acceleration,
            holding
        )

    def move_pose_lin(
        self,
        pose,
        velocity,
        acceleration,
        holding=True
    ):
        return self.ur_move_pose_lin(
            pose,
            velocity,
            acceleration,
            holding
        )

    def move_pose_ompl(
        self,
        pose,
        velocity,
        acceleration,
        holding=True
    ):
        return self.ur_move_pose_ompl(
            pose,
            velocity,
            acceleration,
            holding
        )