#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

import serial


class ArduinoSerialPublisher(Node):
    def __init__(self):
        super().__init__('arduino_serial_publisher')

        # ===== 參數設定 =====
        self.port = '/dev/ttyUSB1'
        self.baudrate = 115200

        # ===== Publisher =====
        self.force_array_pub = self.create_publisher(
            Float32MultiArray,
            'force_values',
            10
        )

        self.force_1_pub = self.create_publisher(
            Float32,
            'force_1',
            10
        )

        self.force_2_pub = self.create_publisher(
            Float32,
            'force_2',
            10
        )

        # ===== Serial =====
        self.ser = None
        self.connect_serial()

        # 20Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1
            )

            # Arduino 重開機時給一點時間
            self.get_logger().info(
                f'Connected to Arduino: {self.port}, baudrate={self.baudrate}'
            )

        except serial.SerialException as e:
            self.ser = None
            self.get_logger().error(f'Failed to connect serial: {e}')
            self.get_logger().error('Check port name or permission.')

    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            return

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            if line == '':
                return

            # Arduino 輸出格式：
            # -0.66,-0.02
            parts = line.split(',')

            if len(parts) == 1:
                # 只接一個感測器時
                force1 = float(parts[0])

                msg1 = Float32()
                msg1.data = force1
                self.force_1_pub.publish(msg1)

                array_msg = Float32MultiArray()
                array_msg.data = [force1]
                self.force_array_pub.publish(array_msg)

                self.get_logger().info(f'force_1: {force1:.2f}')

            elif len(parts) == 2:
                # 兩個感測器時
                force1 = float(parts[0])
                force2 = float(parts[1])

                msg1 = Float32()
                msg1.data = force1
                self.force_1_pub.publish(msg1)

                msg2 = Float32()
                msg2.data = force2
                self.force_2_pub.publish(msg2)

                array_msg = Float32MultiArray()
                array_msg.data = [force1, force2]
                self.force_array_pub.publish(array_msg)

                self.get_logger().info(
                    f'force_1: {force1:.2f}, force_2: {force2:.2f}'
                )

            else:
                self.get_logger().warn(f'Invalid format: {line}')

        except ValueError:
            self.get_logger().warn(f'Invalid number data: {line}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial disconnected: {e}')
            if self.ser is not None:
                self.ser.close()
            self.ser = None

        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = ArduinoSerialPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()