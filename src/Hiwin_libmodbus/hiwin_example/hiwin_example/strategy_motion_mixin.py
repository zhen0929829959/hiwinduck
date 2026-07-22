#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from hiwin_interfaces.srv import RobotCommand


class StrategyMotionMixin:

    def create_pose(self, x, y, z, rx, ry, rz):
        pose = Twist()
        pose.linear.x = float(x)
        pose.linear.y = float(y)
        pose.linear.z = float(z)
        pose.angular.x = float(rx)
        pose.angular.y = float(ry)
        pose.angular.z = float(rz)
        return pose

    def pose_list_to_twist(self, pose_values):
        if pose_values is None or len(pose_values) < 6:
            self.get_logger().error('Pose requires six values')
            return None

        return self.create_pose(
            pose_values[0],
            pose_values[1],
            pose_values[2],
            pose_values[3],
            pose_values[4],
            pose_values[5]
        )

    def move_pose(self, pose, velocity, acceleration, holding=True):
        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=velocity,
            acceleration=acceleration,
            tool=self.ACTIVE_TOOL,
            base=self.ACTIVE_BASE,
            pose=pose,
            holding=holding
        )

        response = self.call_hiwin(request)

        if response is None:
            return False

        return True

    def move_joints(self, joints, velocity, acceleration, holding=True):
        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.JOINTS_CMD,
            velocity=velocity,
            acceleration=acceleration,
            joints=joints,
            tool=self.ACTIVE_TOOL,
            base=self.ACTIVE_BASE,
            holding=holding
        )

        response = self.call_hiwin(request)

        if response is None:
            return False

        return True