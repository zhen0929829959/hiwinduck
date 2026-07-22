#!/usr/bin/env python3

import json
import time
from threading import Event, Thread

import rclpy
from geometry_msgs.msg import Twist
from hiwin_interfaces.srv import RobotCommand
from rclpy.node import Node
from std_msgs.msg import String

from hiwin_example.hiwin_robot_mixin import HiwinRobotMixin


ACTIVE_TOOL = 7
ACTIVE_BASE = 0


# ============================================================
# 插入設定
# ============================================================

# 最多允許向下移動多少
INSERT_DISTANCE_MM = 50.0

# 一般碰撞發生時，插入深度大於等於此值視為成功
SUCCESS_DEPTH_THRESHOLD_MM = 30.0

# 深度判斷容許誤差
DEPTH_TOLERANCE_MM = 0.5

# 插入速度
INSERT_VELOCITY = 1
INSERT_ACCELERATION = 1

# 失敗後回到插入起點的速度
RETURN_VELOCITY = 3
RETURN_ACCELERATION = 3

# 等待力感測器完成零點校正
MONITOR_READY_TIMEOUT_SEC = 5.0

# 最長插入時間
MAX_INSERTION_TIME_SEC = 100.0

# MOTION_STOP 後等待機械手臂穩定
STOP_SETTLE_SEC = 0.5

# 回原位完成後等待穩定
RETURN_SETTLE_SEC = 0.5

# 回原位容許誤差
RETURN_TOLERANCE_MM = 1.0

# ------------------------------------------------------------
# 安全測試模式
#
# True：
# 手臂往上移動，用於先測 COLLISION → STOP 流程
#
# False：
# 手臂真正往下插入
# ------------------------------------------------------------
SAFE_UPWARD_TEST_MODE = False


class ContinuousInsertionTestNode(
    HiwinRobotMixin,
    Node
):

    def __init__(self):
        super().__init__(
            'continuous_insertion_test_node'
        )

        self.hiwin_client = self.create_client(
            RobotCommand,
            'hiwinmodbus_service'
        )

        self.monitor_command_pub = (
            self.create_publisher(
                String,
                '/insertion/command',
                10
            )
        )

        self.monitor_status_sub = (
            self.create_subscription(
                String,
                '/insertion/status',
                self.monitor_status_callback,
                10
            )
        )

        self.running = True
        self.test_thread = None

        # Monitor 完成零點校正
        self.monitor_ready_event = Event()

        # 收到碰撞或其他停止條件
        self.stop_requested_event = Event()

        self.stop_reason = None
        self.latest_monitor_data = None

        self.start_pose = None
        self.target_pose = None
        self.stopped_pose = None
        self.return_pose = None

        self.insertion_result = None

        self.get_logger().info(
            'Continuous insertion test node started'
        )

    # ========================================================
    # List 轉 Twist
    # ========================================================

    def pose_list_to_twist(self, pose_values):
        if (
            pose_values is None
            or len(pose_values) < 6
        ):
            self.get_logger().error(
                'Pose requires six values'
            )
            return None

        pose = Twist()

        pose.linear.x = float(pose_values[0])
        pose.linear.y = float(pose_values[1])
        pose.linear.z = float(pose_values[2])

        pose.angular.x = float(pose_values[3])
        pose.angular.y = float(pose_values[4])
        pose.angular.z = float(pose_values[5])

        return pose

    # ========================================================
    # 發送 Monitor command
    # ========================================================

    def publish_monitor_command(self, command):
        if not rclpy.ok():
            return False

        try:
            msg = String()
            msg.data = str(command)

            self.monitor_command_pub.publish(msg)

            self.get_logger().info(
                f'Published monitor command: {command}'
            )
            return True

        except Exception as exc:
            self.get_logger().error(
                f'Failed to publish monitor command '
                f'{command}: {exc}'
            )
            return False

    # ========================================================
    # 接收 Monitor 狀態
    # ========================================================

    def monitor_status_callback(self, msg):
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

        # 保存真正有力量資料的狀態。
        # 不讓 STOPPED 的零值蓋掉碰撞資料。
        if state in (
            'SEARCHING',
            'COLLISION',
            'EMERGENCY',
            'SENSOR_ERROR'
        ):
            self.latest_monitor_data = dict(data)

        # Monitor 完成歸零，可以開始插入
        if state == 'MONITORING':
            if not self.monitor_ready_event.is_set():
                self.monitor_ready_event.set()

                self.get_logger().info(
                    'Force monitor is ready'
                )
            return

        # 這些狀態不用停止手臂
        if state in (
            'CALIBRATING',
            'SEARCHING',
            'STOPPED',
            'RESET'
        ):
            return

        # 這些狀態需要停止手臂
        if state in (
            'COLLISION',
            'EMERGENCY',
            'SENSOR_ERROR'
        ):
            if not self.stop_requested_event.is_set():
                self.stop_reason = state

                self.get_logger().warn(
                    f'Stop requested by force monitor: '
                    f'{state}, '
                    f'force='
                    f'{data.get("force_level", 0.0)}, '
                    f'reason='
                    f'{data.get("reason", "")}'
                )

                # callback 只設定 Event，
                # 不在 callback 內呼叫 HIWIN service。
                self.stop_requested_event.set()

    # ========================================================
    # holding=False 開始插入
    # ========================================================

    def start_insertion_motion(self):
        pose = self.pose_list_to_twist(
            self.target_pose
        )

        if pose is None:
            return False

        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=INSERT_VELOCITY,
            acceleration=INSERT_ACCELERATION,
            tool=ACTIVE_TOOL,
            base=ACTIVE_BASE,
            pose=pose,
            holding=False
        )

        start_time = time.time()
        response = self.call_hiwin(request)
        elapsed = time.time() - start_time

        self.get_logger().info(
            f'PTP holding=False returned after '
            f'{elapsed:.3f} seconds'
        )

        if response is None:
            self.get_logger().error(
                'Insertion movement returned no response'
            )
            return False

        self.get_logger().info(
            f'Insertion response arm_state='
            f'{response.arm_state}'
        )

        return True

    # ========================================================
    # MOTION_STOP
    # ========================================================

    def stop_robot_motion(self):
        if not rclpy.ok():
            return False

        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.MOTION_STOP,
            tool=ACTIVE_TOOL,
            base=ACTIVE_BASE,
            holding=False
        )

        start_time = time.time()
        response = self.call_hiwin(request)
        elapsed = time.time() - start_time

        self.get_logger().warn(
            f'MOTION_STOP returned after '
            f'{elapsed:.3f} seconds'
        )

        if response is None:
            self.get_logger().error(
                'MOTION_STOP returned no response'
            )
            return False

        self.get_logger().warn(
            f'MOTION_STOP sent, '
            f'arm_state={response.arm_state}'
        )

        return True

    # ========================================================
    # 失敗後回到插入起點
    # ========================================================

    def return_to_start_pose(self):
        if self.start_pose is None:
            self.get_logger().error(
                'Start pose is unavailable'
            )
            return False

        pose = self.pose_list_to_twist(
            self.start_pose
        )

        if pose is None:
            return False

        request = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=RETURN_VELOCITY,
            acceleration=RETURN_ACCELERATION,
            tool=ACTIVE_TOOL,
            base=ACTIVE_BASE,
            pose=pose,
            holding=True
        )

        self.get_logger().warn(
            f'Returning to start pose: '
            f'z={self.start_pose[2]:.3f} mm'
        )

        start_time = time.time()
        response = self.call_hiwin(request)
        elapsed = time.time() - start_time

        if response is None:
            self.get_logger().error(
                'Return movement returned no response'
            )
            return False

        self.get_logger().info(
            f'Return movement finished after '
            f'{elapsed:.3f} seconds, '
            f'arm_state={response.arm_state}'
        )

        return True

    # ========================================================
    # 等待碰撞或 Timeout
    # ========================================================

    def wait_for_stop_condition(self):
        start_time = time.time()

        while (
            rclpy.ok()
            and self.running
            and not self.stop_requested_event.is_set()
        ):
            elapsed = (
                time.time()
                - start_time
            )

            if elapsed >= MAX_INSERTION_TIME_SEC:
                self.stop_reason = 'TIMEOUT'

                self.get_logger().error(
                    f'Insertion timeout after '
                    f'{elapsed:.3f} seconds'
                )

                self.stop_requested_event.set()
                break

            time.sleep(0.005)

    # ========================================================
    # 計算插入深度
    # ========================================================

    def calculate_insertion_depth(self):
        if (
            self.start_pose is None
            or self.stopped_pose is None
        ):
            return None

        if SAFE_UPWARD_TEST_MODE:
            # 安全測試是往上
            depth_mm = (
                float(self.stopped_pose[2])
                - float(self.start_pose[2])
            )

        else:
            # 真實插入是往下
            depth_mm = (
                float(self.start_pose[2])
                - float(self.stopped_pose[2])
            )

        # 避免量測雜訊造成很小的負數
        return max(
            0.0,
            depth_mm
        )

    # ========================================================
    # 根據停止原因與碰撞位置判斷結果
    # ========================================================

    def determine_insertion_result(
        self,
        insertion_depth_mm
    ):
        # 緊急狀態優先判斷。
        # 即使深度已超過成功門檻，也不能判定成功。
        if self.stop_reason == 'EMERGENCY':
            self.insertion_result = 'EMERGENCY'

            self.get_logger().error(
                'INSERTION EMERGENCY: force exceeded '
                'the emergency threshold'
            )
            return

        if self.stop_reason == 'SENSOR_ERROR':
            self.insertion_result = 'SENSOR_ERROR'

            self.get_logger().error(
                'INSERTION FAILED: force sensor error'
            )
            return

        if self.stop_reason == 'MOTION_STOP_FAILED':
            self.insertion_result = 'MOTION_STOP_FAILED'

            self.get_logger().error(
                'INSERTION FAILED: MOTION_STOP failed'
            )
            return

        if self.stop_reason == 'TIMEOUT':
            if insertion_depth_mm < 0.5:
                self.insertion_result = (
                    'ROBOT_NOT_MOVING'
                )

                self.get_logger().error(
                    'ROBOT NOT MOVING: movement command '
                    'was sent, but position did not change'
                )

            else:
                self.insertion_result = 'TIMEOUT'

                self.get_logger().error(
                    'INSERTION TIMEOUT: no collision '
                    'detected before the time limit'
                )

            return

        # 只有一般碰撞才依插入深度判斷成功。
        if self.stop_reason == 'COLLISION':
            success_depth_reached = (
                insertion_depth_mm
                + DEPTH_TOLERANCE_MM
                >= SUCCESS_DEPTH_THRESHOLD_MM
            )

            if success_depth_reached:
                self.insertion_result = 'SUCCESS'

                self.get_logger().info(
                    'INSERTION SUCCESS: collision occurred '
                    'after sufficient insertion depth'
                )

                self.get_logger().info(
                    f'Depth comparison: '
                    f'{insertion_depth_mm:.3f} mm '
                    f'>= '
                    f'{SUCCESS_DEPTH_THRESHOLD_MM:.3f} mm'
                )

            else:
                self.insertion_result = (
                    'HOLE_COLLISION'
                )

                self.get_logger().error(
                    'HOLE COLLISION: collision occurred '
                    'before sufficient insertion depth'
                )

                self.get_logger().error(
                    f'Depth comparison: '
                    f'{insertion_depth_mm:.3f} mm '
                    f'< '
                    f'{SUCCESS_DEPTH_THRESHOLD_MM:.3f} mm'
                )

            return

        self.insertion_result = (
            self.stop_reason
            or 'UNKNOWN'
        )

        self.get_logger().error(
            f'Unknown insertion result: '
            f'{self.insertion_result}'
        )

    # ========================================================
    # 判斷是否需要退回起點
    # ========================================================

    def should_return_to_start(self):
        # 成功時保留在插入位置，不回起點。
        if self.insertion_result == 'SUCCESS':
            return False

        # 碰撞失敗、緊急、逾時等狀態皆退回。
        return self.insertion_result in (
            'HOLE_COLLISION',
            'EMERGENCY',
            'SENSOR_ERROR',
            'TIMEOUT',
            'ROBOT_NOT_MOVING',
            'MOTION_STOP_FAILED',
            'MOVE_COMMAND_FAILED',
            'MONITOR_NOT_READY',
            'UNKNOWN'
        )

    # ========================================================
    # 執行失敗後回程
    # ========================================================

    def handle_return_motion(self):
        if not self.should_return_to_start():
            self.get_logger().info(
                'Insertion succeeded; robot remains '
                'at the inserted position'
            )
            return True

        # ROBOT_NOT_MOVING 時本來就沒有下降，
        # 不需要再送一次完全相同的位置。
        if self.insertion_result == 'ROBOT_NOT_MOVING':
            self.get_logger().warn(
                'Robot did not move; return movement '
                'is not required'
            )
            return True

        self.get_logger().warn(
            f'Return required because result='
            f'{self.insertion_result}'
        )

        time.sleep(1.0)

        if not self.return_to_start_pose():
            self.get_logger().error(
                'Return command failed'
            )
            return False

        time.sleep(
            RETURN_SETTLE_SEC
        )

        self.return_pose = (
            self.get_current_robot_pose()
        )

        if self.return_pose is None:
            self.get_logger().error(
                'Cannot read return pose'
            )
            return False

        return_error_mm = abs(
            float(self.return_pose[2])
            - float(self.start_pose[2])
        )

        self.get_logger().info(
            f'Return z='
            f'{self.return_pose[2]:.3f} mm'
        )

        self.get_logger().info(
            f'Return error='
            f'{return_error_mm:.3f} mm'
        )

        if (
            return_error_mm
            <= RETURN_TOLERANCE_MM
        ):
            self.get_logger().info(
                'RETURN PASSED: robot returned '
                'to insertion start pose'
            )
            return True

        self.get_logger().error(
            'RETURN PARTIAL: robot moved upward, '
            'but return error is too large'
        )
        return False

    # ========================================================
    # 印出碰撞力量
    # ========================================================

    def log_collision_force(self):
        if self.latest_monitor_data is None:
            self.get_logger().warn(
                'No force data was saved'
            )
            return

        self.get_logger().info(
            'Collision force: '
            f'f1='
            f'{self.latest_monitor_data.get("force_1", 0.0)}, '
            f'f2='
            f'{self.latest_monitor_data.get("force_2", 0.0)}, '
            f'level='
            f'{self.latest_monitor_data.get("force_level", 0.0)}'
        )

    # ========================================================
    # 印出最終結果
    # ========================================================

    def log_final_result(self):
        if self.insertion_result == 'SUCCESS':
            self.get_logger().info(
                'FINAL RESULT: INSERTION SUCCESS'
            )

        elif self.insertion_result == 'HOLE_COLLISION':
            self.get_logger().error(
                'FINAL RESULT: COLLISION AT HOLE'
            )

        elif self.insertion_result == 'EMERGENCY':
            self.get_logger().error(
                'FINAL RESULT: EMERGENCY'
            )

        elif self.insertion_result == 'TIMEOUT':
            self.get_logger().error(
                'FINAL RESULT: TIMEOUT'
            )

        elif self.insertion_result == 'SENSOR_ERROR':
            self.get_logger().error(
                'FINAL RESULT: SENSOR ERROR'
            )

        elif self.insertion_result == 'ROBOT_NOT_MOVING':
            self.get_logger().error(
                'FINAL RESULT: ROBOT NOT MOVING'
            )

        elif self.insertion_result == 'MOTION_STOP_FAILED':
            self.get_logger().error(
                'FINAL RESULT: MOTION STOP FAILED'
            )

        else:
            self.get_logger().error(
                f'FINAL RESULT: '
                f'{self.insertion_result}'
            )

    # ========================================================
    # 主測試流程
    # ========================================================

    def run_test(self):
        motion_started = False

        try:
            # self.get_logger().warn(
            #     'Confirm EXT mode, Servo ON and safe '
            #     'workspace before running'
            # )

            if SAFE_UPWARD_TEST_MODE:
                self.get_logger().warn(
                    'SAFE TEST MODE: robot moves upward'
                )
            else:
                self.get_logger().warn(
                    'REAL INSERTION MODE: '
                    'robot moves downward'
                )

            # ------------------------------------------------
            # 1. 清除舊狀態
            # ------------------------------------------------

            self.monitor_ready_event.clear()
            self.stop_requested_event.clear()

            self.stop_reason = None
            self.insertion_result = None
            self.latest_monitor_data = None

            self.start_pose = None
            self.target_pose = None
            self.stopped_pose = None
            self.return_pose = None

            # ------------------------------------------------
            # 2. 讀取插入起點
            # ------------------------------------------------

            self.start_pose = (
                self.get_current_robot_pose()
            )

            if self.start_pose is None:
                self.get_logger().error(
                    'Cannot get insertion start pose'
                )
                return

            self.get_logger().info(
                f'Start pose: '
                f'x={self.start_pose[0]:.3f}, '
                f'y={self.start_pose[1]:.3f}, '
                f'z={self.start_pose[2]:.3f}, '
                f'rx={self.start_pose[3]:.3f}, '
                f'ry={self.start_pose[4]:.3f}, '
                f'rz={self.start_pose[5]:.3f}'
            )

            # ------------------------------------------------
            # 3. 通知力量 Node 開始零點校正
            # ------------------------------------------------

            self.publish_monitor_command('START')

            self.get_logger().info(
                'Waiting for force monitor calibration'
            )

            ready = self.monitor_ready_event.wait(
                timeout=MONITOR_READY_TIMEOUT_SEC
            )

            if not ready:
                self.stop_reason = (
                    'MONITOR_NOT_READY'
                )
                self.insertion_result = (
                    'MONITOR_NOT_READY'
                )

                self.get_logger().error(
                    'Force monitor did not become ready'
                )
                return

            # ------------------------------------------------
            # 4. 建立目標位置
            # ------------------------------------------------

            self.target_pose = list(
                self.start_pose
            )

            if SAFE_UPWARD_TEST_MODE:
                self.target_pose[2] = (
                    float(self.start_pose[2])
                    + INSERT_DISTANCE_MM
                )
            else:
                self.target_pose[2] = (
                    float(self.start_pose[2])
                    - INSERT_DISTANCE_MM
                )

            self.get_logger().warn(
                f'Movement target: '
                f'z={self.start_pose[2]:.3f} '
                f'-> {self.target_pose[2]:.3f} mm'
            )

            # ------------------------------------------------
            # 5. holding=False 開始移動
            # ------------------------------------------------

            if not self.start_insertion_motion():
                self.stop_reason = (
                    'MOVE_COMMAND_FAILED'
                )
                self.insertion_result = (
                    'MOVE_COMMAND_FAILED'
                )
                return

            motion_started = True

            # ------------------------------------------------
            # 6. 等待力量碰撞或 Timeout
            # ------------------------------------------------

            self.wait_for_stop_condition()

            self.get_logger().warn(
                f'Stop condition received: '
                f'{self.stop_reason}'
            )

            # ------------------------------------------------
            # 7. 發送 MOTION_STOP
            # ------------------------------------------------

            if not self.stop_robot_motion():
                self.stop_reason = (
                    'MOTION_STOP_FAILED'
                )
                self.insertion_result = (
                    'MOTION_STOP_FAILED'
                )

                self.get_logger().error(
                    'Failed to stop robot'
                )
                return

            motion_started = False

            self.publish_monitor_command('STOP')

            time.sleep(
                STOP_SETTLE_SEC
            )

            # ------------------------------------------------
            # 8. 停止後讀取位置
            # ------------------------------------------------

            self.stopped_pose = (
                self.get_current_robot_pose()
            )

            if self.stopped_pose is None:
                self.get_logger().error(
                    'Cannot read stopped pose'
                )
                return

            insertion_depth_mm = (
                self.calculate_insertion_depth()
            )

            if insertion_depth_mm is None:
                self.get_logger().error(
                    'Cannot calculate insertion depth'
                )
                return

            self.get_logger().info(
                f'Stopped z='
                f'{self.stopped_pose[2]:.3f} mm'
            )

            self.get_logger().info(
                f'Insertion depth='
                f'{insertion_depth_mm:.3f} mm'
            )

            self.get_logger().info(
                f'Success depth threshold='
                f'{SUCCESS_DEPTH_THRESHOLD_MM:.3f} mm'
            )

            # ------------------------------------------------
            # 9. 判斷插入結果
            # ------------------------------------------------

            self.determine_insertion_result(
                insertion_depth_mm
            )

            # ------------------------------------------------
            # 10. 印出碰撞力量
            # ------------------------------------------------

            self.log_collision_force()

            # ------------------------------------------------
            # 11. 失敗就退回；成功就留在插入位置
            # ------------------------------------------------

            self.handle_return_motion()

            # ------------------------------------------------
            # 12. 印出最終結果
            # ------------------------------------------------

            self.log_final_result()

        except Exception as exc:
            self.get_logger().error(
                f'Continuous insertion error: {exc}'
            )

            if motion_started and rclpy.ok():
                try:
                    self.stop_robot_motion()

                except Exception as stop_exc:
                    self.get_logger().error(
                        f'Emergency stop exception: '
                        f'{stop_exc}'
                    )

        finally:
            if rclpy.ok():
                self.publish_monitor_command('STOP')

            self.running = False

    # ========================================================
    # 啟動測試 Thread
    # ========================================================

    def start_test_thread(self):
        self.test_thread = Thread(
            target=self.run_test,
            daemon=True
        )

        self.test_thread.start()


def main(args=None):
    rclpy.init(args=args)

    node = ContinuousInsertionTestNode()

    try:
        if not node.hiwin_client.wait_for_service(
            timeout_sec=5.0
        ):
            node.get_logger().error(
                'hiwinmodbus_service is unavailable'
            )
            return

        node.start_test_thread()

        # 主執行緒負責：
        # 1. service future
        # 2. /insertion/status callback
        while (
            rclpy.ok()
            and node.running
            and node.test_thread is not None
            and node.test_thread.is_alive()
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

    except KeyboardInterrupt:
        print(
            'Continuous insertion interrupted'
        )

        node.running = False
        node.stop_reason = 'USER_INTERRUPT'
        node.stop_requested_event.set()

        # 僅在 ROS context 仍有效時嘗試停止。
        if rclpy.ok():
            try:
                node.stop_robot_motion()

            except Exception:
                pass

    finally:
        node.running = False
        node.stop_requested_event.set()

        if (
            node.test_thread is not None
            and node.test_thread.is_alive()
        ):
            node.test_thread.join(
                timeout=2.0
            )

        if rclpy.ok():
            try:
                node.publish_monitor_command('STOP')
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
