#!/usr/bin/env python3

import csv
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import tf2_ros


class TCPCalibrator(Node):

    def __init__(self):
        super().__init__('tcp_calibrator')

        # ============================================================
        # TF frame
        #
        # base:
        #   UR robot base frame
        #
        # tool0:
        #   UR 零 TCP 的工具座標系
        #
        # 我們現在要求的是：
        #
        #       tool0 -> 真正工具尖端 TCP
        #
        # 所以最後算出來的 XYZ 可以直接當作 UR TCP XYZ 使用
        # ============================================================

        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('reference_frame', 'flange')

        self.base_frame = (
            self.get_parameter('base_frame')
            .get_parameter_value()
            .string_value
        )

        self.reference_frame = (
            self.get_parameter('reference_frame')
            .get_parameter_value()
            .string_value
        )

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # 每筆資料：
        #
        # {
        #   'position': [x, y, z],
        #   'rotation': 3x3 rotation matrix,
        #   'quaternion': [qx, qy, qz, qw]
        # }
        #
        self.samples = []

        self.get_logger().info(
            'TCP calibration node started'
        )

        self.get_logger().info(
            f'TF used for calibration: '
            f'{self.base_frame} -> {self.reference_frame}'
        )

    # ================================================================
    # Quaternion xyzw -> Rotation Matrix
    # ================================================================
    def quaternion_to_rotation_matrix(
        self,
        qx,
        qy,
        qz,
        qw
    ):

        norm = np.sqrt(
            qx*qx +
            qy*qy +
            qz*qz +
            qw*qw
        )

        if norm < 1e-12:
            raise ValueError('Invalid quaternion')

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        rotation = np.array(
            [
                [
                    1 - 2*(qy*qy + qz*qz),
                    2*(qx*qy - qz*qw),
                    2*(qx*qz + qy*qw)
                ],
                [
                    2*(qx*qy + qz*qw),
                    1 - 2*(qx*qx + qz*qz),
                    2*(qy*qz - qx*qw)
                ],
                [
                    2*(qx*qz - qy*qw),
                    2*(qy*qz + qx*qw),
                    1 - 2*(qx*qx + qy*qy)
                ]
            ],
            dtype=np.float64
        )

        return rotation

    # ================================================================
    # 讀取目前 base -> tool0
    # ================================================================
    def get_current_reference_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.reference_frame,
                rclpy.time.Time()
            )

        except Exception as e:

            self.get_logger().error(
                f'Cannot get TF '
                f'{self.base_frame} -> {self.reference_frame}: {e}'
            )

            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # reference frame 原點在 base 座標的位置
        p = np.array(
            [
                translation.x,
                translation.y,
                translation.z
            ],
            dtype=np.float64
        )

        # base -> reference frame quaternion
        q = np.array(
            [
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w
            ],
            dtype=np.float64
        )

        # base -> reference frame rotation matrix
        R = self.quaternion_to_rotation_matrix(
            q[0],
            q[1],
            q[2],
            q[3]
        )

        return p, R, q

    # ================================================================
    # 記錄目前姿態
    # ================================================================
    def capture_sample(self):

        result = self.get_current_reference_pose()

        if result is None:
            return False

        p, R, q = result

        self.samples.append(
            {
                'position': p,
                'rotation': R,
                'quaternion': q
            }
        )

        number = len(self.samples)

        print()
        print(
            f'========== Sample {number} =========='
        )

        print(
            f'{self.reference_frame} position in '
            f'{self.base_frame} [m]:'
        )

        print(
            f'X = {p[0]:.6f}, '
            f'Y = {p[1]:.6f}, '
            f'Z = {p[2]:.6f}'
        )

        print(
            'Quaternion [x y z w]: '
            f'[{q[0]:.6f}, '
            f'{q[1]:.6f}, '
            f'{q[2]:.6f}, '
            f'{q[3]:.6f}]'
        )

        return True

    # ================================================================
    # TCP Least Squares Calibration
    # ================================================================
    def calibrate(self):

        number = len(self.samples)

        if number < 4:

            print()
            print(
                '至少需要 4 組姿態，目前只有 '
                f'{number} 組'
            )

            return

        # ------------------------------------------------------------
        # 幾何模型：
        #
        # P = R * t + p
        #
        # P:
        #   固定接觸點在 base 座標的位置
        #
        # R:
        #   base -> tool0 的旋轉
        #
        # p:
        #   tool0 原點在 base 的位置
        #
        # t:
        #   tool0 -> 真正 TCP
        #
        # 整理：
        #
        # R*t - P = -p
        #
        # 未知數：
        #
        # [tx ty tz Px Py Pz]
        #
        # ------------------------------------------------------------

        A_list = []
        b_list = []

        identity = np.eye(
            3,
            dtype=np.float64
        )

        for sample in self.samples:

            R = sample['rotation']
            p = sample['position']

            A_i = np.hstack(
                (
                    R,
                    -identity
                )
            )

            b_i = -p

            A_list.append(A_i)
            b_list.append(b_i)

        A = np.vstack(A_list)
        b = np.hstack(b_list)

        solution, residuals, rank, singular_values = (
            np.linalg.lstsq(
                A,
                b,
                rcond=None
            )
        )

        # ------------------------------------------------------------
        # tool0 -> TCP
        # ------------------------------------------------------------

        tcp = solution[0:3]

        # ------------------------------------------------------------
        # 固定 pivot point 在 base 座標的位置
        # ------------------------------------------------------------

        pivot = solution[3:6]

        # ============================================================
        # 計算每筆校正誤差
        # ============================================================

        errors = []

        calculated_points = []

        for sample in self.samples:

            R = sample['rotation']
            p = sample['position']

            # 根據求出的 TCP，
            # 計算這個 sample 的工具尖端在 base 中的位置
            point = R @ tcp + p

            calculated_points.append(point)

            # 理論上所有 point 都應該等於同一個 pivot
            error = np.linalg.norm(
                point - pivot
            )

            errors.append(error)

        errors = np.array(
            errors,
            dtype=np.float64
        )

        rms_error = np.sqrt(
            np.mean(
                errors ** 2
            )
        )

        mean_error = np.mean(errors)

        max_error = np.max(errors)

        # ------------------------------------------------------------
        # Condition number
        # ------------------------------------------------------------

        if singular_values[-1] > 1e-12:

            condition_number = (
                singular_values[0] /
                singular_values[-1]
            )

        else:

            condition_number = float('inf')

        # ============================================================
        # 顯示結果
        # ============================================================

        print()
        print('========================================')
        print('          TCP CALIBRATION RESULT')
        print('========================================')

        print(
            f'Samples = {number}'
        )

        print()

        print(
            f'{self.reference_frame} -> TCP:'
        )

        print(
            f'X = {tcp[0]: .6f} m '
            f'= {tcp[0]*1000: .3f} mm'
        )

        print(
            f'Y = {tcp[1]: .6f} m '
            f'= {tcp[1]*1000: .3f} mm'
        )

        print(
            f'Z = {tcp[2]: .6f} m '
            f'= {tcp[2]*1000: .3f} mm'
        )

        print()
        print(
            f'Pivot point in {self.base_frame}:'
        )

        print(
            f'X = {pivot[0]: .6f} m'
        )

        print(
            f'Y = {pivot[1]: .6f} m'
        )

        print(
            f'Z = {pivot[2]: .6f} m'
        )

        print()
        print('Calibration error:')

        print(
            f'Mean = '
            f'{mean_error*1000:.3f} mm'
        )

        print(
            f'RMS  = '
            f'{rms_error*1000:.3f} mm'
        )

        print(
            f'Max  = '
            f'{max_error*1000:.3f} mm'
        )

        print()
        print(
            f'Matrix rank = {rank}/6'
        )

        print(
            f'Condition number = '
            f'{condition_number:.3f}'
        )

        # ============================================================
        # 直接輸出 UR TCP XYZ
        # ============================================================

        print()
        print('UR TCP position XYZ:')

        print(
            'p['
            f'{tcp[0]:.6f}, '
            f'{tcp[1]:.6f}, '
            f'{tcp[2]:.6f}, '
            '0, 0, 0]'
        )

        print()
        print(
            '注意：這個 Pivot Calibration '
            '只校正 TCP 位置 XYZ。'
        )

        print(
            'Rx, Ry, Rz 並沒有被這個方法校正。'
        )

        print('========================================')

        # ============================================================
        # 每筆 sample 誤差
        # ============================================================

        print()
        print('Individual sample errors:')

        for index, error in enumerate(errors):

            print(
                f'  Sample {index+1:02d}: '
                f'{error*1000:.3f} mm'
            )

        # ============================================================
        # 找出最大誤差 sample
        # ============================================================

        worst_index = int(
            np.argmax(errors)
        )

        print()
        print(
            'Worst sample: '
            f'{worst_index + 1}'
        )

        print(
            f'Worst error: '
            f'{errors[worst_index]*1000:.3f} mm'
        )

        # ============================================================
        # 判斷資料品質
        # ============================================================

        print()

        if rank < 6:

            print(
                'WARNING: Calibration matrix rank deficient.'
            )

            print(
                '姿態變化不足，TCP 結果不可信。'
            )

        elif condition_number > 100:

            print(
                'WARNING: Condition number is high.'
            )

            print(
                '建議增加更多不同 Roll / Pitch / Yaw 的姿態。'
            )

        elif rms_error > 0.005:

            print(
                'WARNING: RMS error > 5 mm.'
            )

            print(
                '校正誤差很大。'
            )

            print(
                '請檢查是否每次真的碰到完全相同的固定點。'
            )

        elif rms_error > 0.002:

            print(
                'WARNING: RMS error > 2 mm.'
            )

            print(
                '結果可以算出，但精度偏差較大。'
            )

            print(
                '建議重新取點或檢查高誤差 sample。'
            )

        elif rms_error > 0.001:

            print(
                'Calibration usable, '
                'but RMS error is above 1 mm.'
            )

        else:

            print(
                'Calibration geometry looks good.'
            )

        return tcp, pivot, errors

    # ================================================================
    # 刪除最後一筆
    # ================================================================
    def delete_last_sample(self):

        if len(self.samples) == 0:

            print(
                '沒有 sample 可以刪除'
            )

            return

        self.samples.pop()

        print(
            f'已刪除最後一筆，剩下 '
            f'{len(self.samples)} 組'
        )

    # ================================================================
    # 清除全部 sample
    # ================================================================
    def clear_samples(self):

        self.samples.clear()

        print(
            '全部 sample 已清除'
        )

    # ================================================================
    # 儲存 raw data
    # ================================================================
    def save_csv(
        self,
        filename='tcp_calibration_samples.csv'
    ):

        if len(self.samples) == 0:

            print(
                '沒有資料可以儲存'
            )

            return

        with open(
            filename,
            'w',
            newline=''
        ) as csvfile:

            writer = csv.writer(csvfile)

            writer.writerow(
                [
                    'reference_frame',
                    'x',
                    'y',
                    'z',
                    'qx',
                    'qy',
                    'qz',
                    'qw'
                ]
            )

            for sample in self.samples:

                p = sample['position']
                q = sample['quaternion']

                writer.writerow(
                    [
                        self.reference_frame,
                        p[0],
                        p[1],
                        p[2],
                        q[0],
                        q[1],
                        q[2],
                        q[3]
                    ]
                )

        print(
            f'Saved: {filename}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = TCPCalibrator()

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    # TF listener 必須持續 spin
    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )

    spin_thread.start()

    # 等待 TF tree 收到資料
    time.sleep(2.0)

    print()
    print('========================================')
    print('        UR TCP CALIBRATION')
    print('========================================')

    print(
        f'TF: '
        f'{node.base_frame} -> '
        f'{node.reference_frame}'
    )

    print()

    print(
        '工具尖端每次都必須碰在同一個固定點。'
    )

    print(
        '建議記錄 8~15 個不同姿態。'
    )

    print(
        'Roll / Pitch / Yaw 都要有明顯變化。'
    )

    print()
    print('Commands:')

    print(
        '  Enter : 記錄目前姿態'
    )

    print(
        '  c     : 計算 TCP'
    )

    print(
        '  s     : 儲存 CSV'
    )

    print(
        '  d     : 刪除最後一筆'
    )

    print(
        '  r     : 清除全部 sample'
    )

    print(
        '  q     : 離開'
    )

    print()

    try:

        while rclpy.ok():

            command = input(
                f'[{len(node.samples)} samples] > '
            ).strip().lower()

            # --------------------------------------------------------
            # Enter
            # --------------------------------------------------------

            if command == '':

                node.capture_sample()

            # --------------------------------------------------------
            # Calculate
            # --------------------------------------------------------

            elif command == 'c':

                node.calibrate()

            # --------------------------------------------------------
            # Save CSV
            # --------------------------------------------------------

            elif command == 's':

                node.save_csv()

            # --------------------------------------------------------
            # Delete last
            # --------------------------------------------------------

            elif command == 'd':

                node.delete_last_sample()

            # --------------------------------------------------------
            # Reset all
            # --------------------------------------------------------

            elif command == 'r':

                answer = input(
                    '確定要清除全部 sample？ [y/N]: '
                ).strip().lower()

                if answer == 'y':

                    node.clear_samples()

                else:

                    print(
                        '取消'
                    )

            # --------------------------------------------------------
            # Quit
            # --------------------------------------------------------

            elif command == 'q':

                break

            else:

                print(
                    '可用指令：'
                    'Enter / c / s / d / r / q'
                )

    except KeyboardInterrupt:

        pass

    finally:

        executor.shutdown()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()