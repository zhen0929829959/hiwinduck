#!/usr/bin/env python3

import json
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


# ============================================================
# 力感測參數
# 需要依照你的 HX711 實際數值調整
# ============================================================

# START 後收集幾筆資料作為力量零點
BASELINE_SAMPLE_COUNT = 10

# 移動平均濾波筆數
FILTER_WINDOW_SIZE = 3

# 超過這個力量，視為碰撞
COLLISION_FORCE_THRESHOLD = 600.0

# 超過這個力量，視為緊急碰撞
EMERGENCY_FORCE_THRESHOLD = 2000.0

# 碰撞必須連續出現幾筆才成立，避免瞬間雜訊
REQUIRED_COLLISION_COUNT = 3


class InsertionMonitorNode(Node):

    def __init__(self):
        super().__init__('insertion_monitor_node')

        # Arduino 發布的力量
        self.force_sub = self.create_subscription(
            Float32MultiArray,
            '/force_values',
            self.force_callback,
            10
        )

        # 接收 START、STOP、RESET
        self.command_sub = self.create_subscription(
            String,
            '/insertion/command',
            self.command_callback,
            10
        )

        # 發布監測結果
        self.status_pub = self.create_publisher(
            String,
            '/insertion/status',
            10
        )

        self.monitoring = False
        self.calibrating = False

        self.baseline_samples_1 = []
        self.baseline_samples_2 = []

        self.baseline_force_1 = 0.0
        self.baseline_force_2 = 0.0

        self.force_window_1 = deque(
            maxlen=FILTER_WINDOW_SIZE
        )

        self.force_window_2 = deque(
            maxlen=FILTER_WINDOW_SIZE
        )

        self.collision_count = 0
        self.last_status = None

        self.get_logger().info(
            'Insertion monitor node started'
        )

        self.get_logger().info(
            'Waiting for /insertion/command START'
        )

    # ========================================================
    # 工具函式
    # ========================================================

    @staticmethod
    def average(values):
        if not values:
            return 0.0

        return sum(values) / len(values)

    # ========================================================
    # 發布狀態
    # ========================================================

    def publish_status(
        self,
        state,
        force_1=0.0,
        force_2=0.0,
        force_level=0.0,
        collision_count=0,
        reason='',
        always_publish=False
    ):
        # SEARCHING 狀態不要一直重複發布
        if (
            not always_publish
            and state == self.last_status
        ):
            return

        self.last_status = state

        data = {
            'state': str(state),
            'force_1': float(force_1),
            'force_2': float(force_2),
            'force_level': float(force_level),
            'collision_count': int(collision_count),
            'reason': str(reason)
        }

        msg = String()
        msg.data = json.dumps(
            data,
            ensure_ascii=False
        )

        self.status_pub.publish(msg)

        if state in (
            'COLLISION',
            'EMERGENCY',
            'SENSOR_ERROR'
        ):
            self.get_logger().warn(
                f'Status={state}, '
                f'f1={force_1:.3f}, '
                f'f2={force_2:.3f}, '
                f'force_level={force_level:.3f}, '
                f'reason={reason}'
            )
        else:
            self.get_logger().info(
                f'Status={state}: {reason}'
            )

    # ========================================================
    # 接收控制命令
    # ========================================================

    def command_callback(self, msg):
        command = msg.data.strip().upper()

        self.get_logger().info(
            f'Received command: {command}'
        )

        if command == 'START':
            self.start_monitoring()

        elif command == 'STOP':
            self.stop_monitoring()

        elif command == 'RESET':
            self.reset_monitor()

        else:
            self.get_logger().warn(
                f'Unknown insertion command: {command}'
            )

    def start_monitoring(self):
        # START 後重新計算零點
        self.monitoring = False
        self.calibrating = True

        self.baseline_samples_1.clear()
        self.baseline_samples_2.clear()

        self.force_window_1.clear()
        self.force_window_2.clear()

        self.collision_count = 0
        self.last_status = None

        self.publish_status(
            state='CALIBRATING',
            reason=(
                f'Collecting {BASELINE_SAMPLE_COUNT} '
                f'baseline samples'
            ),
            always_publish=True
        )

    def stop_monitoring(self):
        self.monitoring = False
        self.calibrating = False
        self.collision_count = 0

        self.publish_status(
            state='STOPPED',
            reason='Force monitoring stopped',
            always_publish=True
        )

    def reset_monitor(self):
        self.monitoring = False
        self.calibrating = False

        self.baseline_force_1 = 0.0
        self.baseline_force_2 = 0.0

        self.baseline_samples_1.clear()
        self.baseline_samples_2.clear()

        self.force_window_1.clear()
        self.force_window_2.clear()

        self.collision_count = 0
        self.last_status = None

        self.publish_status(
            state='RESET',
            reason='Force monitor reset',
            always_publish=True
        )

    # ========================================================
    # 接收力量
    # ========================================================

    def force_callback(self, msg):
        # self.get_logger().info(
        #     f'Force received: '
        #     f'{list(msg.data)}, '
        #     f'calibrating={self.calibrating}, '
        #     f'baseline_count={len(self.baseline_samples_1)}'
        # )
        if len(msg.data) < 2:
            self.monitoring = False
            self.calibrating = False

            self.publish_status(
                state='SENSOR_ERROR',
                reason=(
                    '/force_values requires at least '
                    'two values'
                ),
                always_publish=True
            )
            return

        raw_force_1 = float(msg.data[0])
        raw_force_2 = float(msg.data[1])

        # ----------------------------------------------------
        # 零點校正
        # ----------------------------------------------------

        if self.calibrating:
            self.baseline_samples_1.append(
                raw_force_1
            )

            self.baseline_samples_2.append(
                raw_force_2
            )

            if (
                len(self.baseline_samples_1)
                >= BASELINE_SAMPLE_COUNT
            ):
                self.baseline_force_1 = self.average(
                    self.baseline_samples_1
                )

                self.baseline_force_2 = self.average(
                    self.baseline_samples_2
                )

                self.calibrating = False
                self.monitoring = True

                self.force_window_1.clear()
                self.force_window_2.clear()

                self.publish_status(
                    state='MONITORING',
                    reason=(
                        'Baseline completed: '
                        f'b1={self.baseline_force_1:.3f}, '
                        f'b2={self.baseline_force_2:.3f}'
                    ),
                    always_publish=True
                )

            return

        if not self.monitoring:
            return

        # ----------------------------------------------------
        # 扣除力量零點
        # ----------------------------------------------------

        corrected_force_1 = (
            raw_force_1
            - self.baseline_force_1
        )

        corrected_force_2 = (
            raw_force_2
            - self.baseline_force_2
        )

        # ----------------------------------------------------
        # 移動平均濾波
        # ----------------------------------------------------

        self.force_window_1.append(
            corrected_force_1
        )

        self.force_window_2.append(
            corrected_force_2
        )

        filtered_force_1 = self.average(
            self.force_window_1
        )

        filtered_force_2 = self.average(
            self.force_window_2
        )

        # 取兩顆感測器中的最大力量
        force_level = max(
            abs(filtered_force_1),
            abs(filtered_force_2)
        )

        # ----------------------------------------------------
        # 緊急碰撞
        # ----------------------------------------------------

        if (
            force_level
            >= EMERGENCY_FORCE_THRESHOLD
        ):
            self.monitoring = False

            self.publish_status(
                state='EMERGENCY',
                force_1=filtered_force_1,
                force_2=filtered_force_2,
                force_level=force_level,
                # collision_count=self.collision_count,
                # reason=(
                #     'Force exceeded emergency threshold'
                # ),
                always_publish=True
            )
            return

        # ----------------------------------------------------
        # 一般碰撞判斷
        # ----------------------------------------------------

        if force_level >= COLLISION_FORCE_THRESHOLD:
            self.collision_count += 1

        else:
            self.collision_count = 0

        # 必須連續多筆超過門檻
        if (
            self.collision_count
            >= REQUIRED_COLLISION_COUNT
        ):
            self.monitoring = False

            self.publish_status(
                state='COLLISION',
                force_1=filtered_force_1,
                force_2=filtered_force_2,
                force_level=force_level,
                # collision_count=self.collision_count,
                # reason=(
                #     'Collision force continuously '
                #     'exceeded threshold'
                # ),
                always_publish=True
            )
            return

        # 正常搜尋中
        self.publish_status(
            state='SEARCHING',
            force_1=filtered_force_1,
            force_2=filtered_force_2,
            force_level=force_level,
            # collision_count=self.collision_count,
            # reason=(
            #     f'Waiting for collision, '
            #     f'count={self.collision_count}/'
            #     f'{REQUIRED_COLLISION_COUNT}'
            # )
        )


def main(args=None):
    rclpy.init(args=args)

    node = InsertionMonitorNode()

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