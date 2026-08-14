import time

import rclpy
from geometry_msgs.msg import Twist
from hiwin_interfaces.srv import RobotCommand
from rclpy.task import Future


DEFAULT_VELOCITY = 20
DEFAULT_ACCELERATION = 15
ACTIVE_TOOL = 7
ACTIVE_BASE = 0


class HiwinRobotMixin:

    # ========================================================
    # 取得目前手臂位姿
    # ========================================================

    def get_current_robot_pose(self):
        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.CHECK_POSE,
            tool=ACTIVE_TOOL,
            base=ACTIVE_BASE,
            holding=True
        )

        response = self.call_hiwin(request)

        if response is None:
            self.get_logger().error(
                'Cannot get current robot pose'
            )
            return None

        if not response.current_position:
            self.get_logger().error(
                'Current robot pose is empty'
            )
            return None

        if len(response.current_position) < 6:
            self.get_logger().error(
                'Current robot pose requires 6 values'
            )
            return None

        return [
            float(value)
            for value in response.current_position[:6]
        ]

    # ========================================================
    # 等待 service future
    # ========================================================

    def _wait_for_future_done(
        self,
        future: Future,
        timeout=-1
    ):
        start_time = time.time()

        while (
            rclpy.ok()
            and not future.done()
        ):
            time.sleep(0.01)

            if (
                timeout > 0
                and time.time() - start_time > timeout
            ):
                self.get_logger().error(
                    'Wait for service timeout'
                )
                return False

        return future.done()

    # ========================================================
    # 呼叫 HIWIN service
    # ========================================================

    def call_hiwin(
        self,
        request,
        timeout=-1
    ):
        while (
            rclpy.ok()
            and not self.hiwin_client.wait_for_service(
                timeout_sec=2.0
            )
        ):
            self.get_logger().info(
                'Service not available, waiting...'
            )

        if not rclpy.ok():
            return None

        future = self.hiwin_client.call_async(
            request
        )

        if not self._wait_for_future_done(
            future,
            timeout=timeout
        ):
            return None

        try:
            return future.result()

        except Exception as exc:
            self.get_logger().error(
                f'HIWIN service call failed: {exc}'
            )
            return None

    # ========================================================
    # 建立 HIWIN request
    # ========================================================

    def generate_robot_request(
        self,
        holding=True,
        cmd_mode=RobotCommand.Request.PTP,
        cmd_type=RobotCommand.Request.POSE_CMD,
        velocity=DEFAULT_VELOCITY,
        acceleration=DEFAULT_ACCELERATION,
        tool=ACTIVE_TOOL,
        base=ACTIVE_BASE,
        digital_input_pin=2,
        digital_output_pin=6,
        digital_output_cmd=(
            RobotCommand.Request.DIGITAL_OFF
        ),
        pose=None,
        joints=None,
        circ_s=None,
        circ_end=None,
        jog_joint=6,
        jog_dir=0,
        move_dir='z',
        move_dis=0.01
    ):
        if pose is None:
            pose = Twist()

        if joints is None:
            joints = [
                float('inf')
            ] * 6

        if circ_s is None:
            circ_s = []

        if circ_end is None:
            circ_end = []

        request = RobotCommand.Request()

        request.digital_input_pin = (
            digital_input_pin
        )

        request.digital_output_pin = (
            digital_output_pin
        )

        request.digital_output_cmd = (
            digital_output_cmd
        )

        request.acceleration = acceleration
        request.velocity = velocity

        request.jog_joint = jog_joint
        request.jog_dir = jog_dir

        request.tool = tool
        request.base = base

        request.cmd_mode = cmd_mode
        request.cmd_type = cmd_type

        request.holding = holding
        request.joints = joints

        request.circ_s = circ_s
        request.circ_end = circ_end

        request.pose = pose

        request.move_dir = move_dir
        request.move_dis = move_dis

        return request