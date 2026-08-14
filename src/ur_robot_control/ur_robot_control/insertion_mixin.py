#!/usr/bin/env python3

import json
import time

import rclpy
from hiwin_interfaces.srv import RobotCommand
from std_msgs.msg import String


class InsertionMixin:

    def publish_insertion_command(self, command):
        msg = String()
        msg.data = str(command)
        self.insertion_command_pub.publish(msg)

        self.get_logger().info(
            f'Published insertion command: {command}'
        )

    def insertion_status_callback(self, msg):
        try:
            data = json.loads(msg.data)

        except json.JSONDecodeError:
            data = {
                'state': msg.data.strip().upper(),
                'reason': ''
            }

        state = str(
            data.get('state', '')
        ).strip().upper()

        if state in (
            'SEARCHING',
            'COLLISION',
            'EMERGENCY',
            'SENSOR_ERROR'
        ):
            self.latest_insertion_monitor_data = (
                dict(data)
            )

        if state == 'MONITORING':
            if not self.insertion_monitor_ready_event.is_set():
                self.insertion_monitor_ready_event.set()

                self.get_logger().info(
                    'Insertion force monitor is ready'
                )
            return

        if state not in (
            'COLLISION',
            'EMERGENCY',
            'SENSOR_ERROR'
        ):
            return

        if self.insertion_stop_event.is_set():
            return

        self.insertion_stop_reason = state

        self.get_logger().warn(
            f'Insertion stop requested: '
            f'{state}, '
            f'force={data.get("force_level", 0.0)}'
        )

        self.insertion_stop_event.set()

    def stop_insertion_motion(self):
        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.MOTION_STOP,
            tool=self.ACTIVE_TOOL,
            base=self.ACTIVE_BASE,
            holding=False
        )

        start_time = time.time()
        response = self.call_hiwin(request)
        elapsed = time.time() - start_time

        self.get_logger().warn(
            f'Insertion MOTION_STOP returned after '
            f'{elapsed:.3f} seconds'
        )

        if response is None:
            self.get_logger().error(
                'Insertion MOTION_STOP failed'
            )
            return False

        self.get_logger().warn(
            f'Insertion MOTION_STOP sent, '
            f'arm_state={response.arm_state}'
        )
        return True

    def prepare_insertion(self):
        if self.insertion_start_pose is None:
            self.get_logger().error(
                'Insertion start pose is missing'
            )
            return False

        self.insertion_monitor_ready_event.clear()
        self.insertion_stop_event.clear()

        self.insertion_stop_reason = None
        self.latest_insertion_monitor_data = None
        self.insertion_target_pose = None
        self.insertion_stopped_pose = None
        self.insertion_depth_mm = 0.0
        self.insertion_result = None

        self.publish_insertion_command(
            'START'
        )

        self.get_logger().info(
            'Waiting for insertion force monitor calibration'
        )

        ready = self.insertion_monitor_ready_event.wait(
            timeout=self.MONITOR_READY_TIMEOUT_SEC
        )

        if not ready:
            self.insertion_result = (
                'MONITOR_NOT_READY'
            )

            self.get_logger().error(
                'Insertion force monitor did not become ready'
            )
            return False

        return True

    def run_insertion(self):
        self.insertion_target_pose = list(
            self.insertion_start_pose
        )

        self.insertion_target_pose[2] = (
            float(self.insertion_start_pose[2])
            - self.INSERT_DISTANCE_MM
        )

        pose = self.pose_list_to_twist(
            self.insertion_target_pose
        )

        if pose is None:
            self.insertion_result = (
                'INVALID_TARGET_POSE'
            )
            return False

        self.get_logger().warn(
            f'Insertion movement: '
            f'z={self.insertion_start_pose[2]:.3f} '
            f'-> {self.insertion_target_pose[2]:.3f} mm'
        )

        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=self.INSERT_VELOCITY,
            acceleration=self.INSERT_ACCELERATION,
            tool=self.ACTIVE_TOOL,
            base=self.ACTIVE_BASE,
            pose=pose,
            holding=False
        )

        start_time = time.time()
        response = self.call_hiwin(request)
        elapsed = time.time() - start_time

        self.get_logger().info(
            f'Insertion PTP holding=False returned after '
            f'{elapsed:.3f} seconds'
        )

        if response is None:
            self.insertion_result = (
                'MOVE_COMMAND_FAILED'
            )

            self.get_logger().error(
                'Insertion movement command failed'
            )
            return False

        self.get_logger().info(
            f'Insertion response arm_state='
            f'{response.arm_state}'
        )

        wait_start_time = time.time()

        while (
            rclpy.ok()
            and not self.insertion_stop_event.is_set()
        ):
            elapsed = (
                time.time()
                - wait_start_time
            )

            if elapsed >= self.MAX_INSERTION_TIME_SEC:
                self.insertion_stop_reason = (
                    'TIMEOUT'
                )
                self.insertion_stop_event.set()

                self.get_logger().error(
                    f'Insertion timeout after '
                    f'{elapsed:.3f} seconds'
                )
                break

            time.sleep(0.005)

        self.get_logger().warn(
            f'Insertion stop condition received: '
            f'{self.insertion_stop_reason}'
        )

        if not self.stop_insertion_motion():
            self.insertion_result = (
                'MOTION_STOP_FAILED'
            )
            return False

        self.publish_insertion_command(
            'STOP'
        )

        time.sleep(
            self.STOP_SETTLE_SEC
        )

        return True

    def check_insertion_result(self):
        self.insertion_stopped_pose = (
            self.get_current_robot_pose()
        )

        if self.insertion_stopped_pose is None:
            self.insertion_result = (
                'POSE_READ_FAILED'
            )

            self.get_logger().error(
                'Cannot read insertion stopped pose'
            )
            return False

        self.insertion_depth_mm = max(
            0.0,
            float(self.insertion_start_pose[2])
            - float(self.insertion_stopped_pose[2])
        )

        self.get_logger().info(
            f'Insertion stopped z='
            f'{self.insertion_stopped_pose[2]:.3f} mm'
        )

        self.get_logger().info(
            f'Insertion depth='
            f'{self.insertion_depth_mm:.3f} mm'
        )

        self.get_logger().info(
            f'Success depth threshold='
            f'{self.SUCCESS_DEPTH_THRESHOLD_MM:.3f} mm'
        )

        self.log_insertion_force()
        self.determine_insertion_result()

        if self.insertion_result == 'SUCCESS':
            self.get_logger().info(
                'Insertion succeeded; '
                'robot remains at inserted position'
            )
            return True

        self.get_logger().warn(
            f'Insertion failed with result='
            f'{self.insertion_result}; '
            f'returning to aligned start pose'
        )

        return self.return_to_insertion_start_pose()

    def determine_insertion_result(self):
        if self.insertion_stop_reason == 'EMERGENCY':
            self.insertion_result = 'EMERGENCY'

        elif self.insertion_stop_reason == 'SENSOR_ERROR':
            self.insertion_result = (
                'SENSOR_ERROR'
            )

        elif self.insertion_stop_reason == 'TIMEOUT':
            if self.insertion_depth_mm < 0.5:
                self.insertion_result = (
                    'ROBOT_NOT_MOVING'
                )
            else:
                self.insertion_result = (
                    'TIMEOUT'
                )

        elif self.insertion_stop_reason == 'COLLISION':
            success_depth_reached = (
                self.insertion_depth_mm
                + self.DEPTH_TOLERANCE_MM
                >= self.SUCCESS_DEPTH_THRESHOLD_MM
            )

            if success_depth_reached:
                self.insertion_result = (
                    'SUCCESS'
                )
            else:
                self.insertion_result = (
                    'HOLE_COLLISION'
                )

        else:
            self.insertion_result = (
                self.insertion_stop_reason
                or 'UNKNOWN'
            )

        self.get_logger().info(
            f'Insertion result='
            f'{self.insertion_result}'
        )

    def log_insertion_force(self):
        if self.latest_insertion_monitor_data is None:
            self.get_logger().warn(
                'No insertion force data was saved'
            )
            return

        self.get_logger().info(
            'Insertion collision force: '
            f'f1='
            f'{self.latest_insertion_monitor_data.get("force_1", 0.0)}, '
            f'f2='
            f'{self.latest_insertion_monitor_data.get("force_2", 0.0)}, '
            f'level='
            f'{self.latest_insertion_monitor_data.get("force_level", 0.0)}'
        )

    def return_to_insertion_start_pose(self):
        if self.insertion_start_pose is None:
            self.get_logger().error(
                'Insertion start pose is missing'
            )
            return False

        pose = self.pose_list_to_twist(
            self.insertion_start_pose
        )

        if pose is None:
            return False

        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=self.RETURN_VELOCITY,
            acceleration=self.RETURN_ACCELERATION,
            tool=self.ACTIVE_TOOL,
            base=self.ACTIVE_BASE,
            pose=pose,
            holding=True
        )

        self.get_logger().warn(
            f'Returning to insertion start pose: '
            f'z={self.insertion_start_pose[2]:.3f} mm'
        )

        response = self.call_hiwin(request)

        if response is None:
            self.get_logger().error(
                'Return to insertion start pose failed'
            )
            return False

        time.sleep(
            self.RETURN_SETTLE_SEC
        )

        return_pose = self.get_current_robot_pose()

        if return_pose is None:
            self.get_logger().error(
                'Cannot read pose after insertion return'
            )
            return False

        return_error_mm = abs(
            float(return_pose[2])
            - float(self.insertion_start_pose[2])
        )

        self.get_logger().info(
            f'Insertion return z={return_pose[2]:.3f} mm, '
            f'error={return_error_mm:.3f} mm'
        )

        return True