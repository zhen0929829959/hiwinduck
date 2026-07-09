import json
import time

import numpy as np
import rclpy


DISCARD_SAMPLE_COUNT = 10
MEDIAN_SAMPLE_COUNT = 20
TAG_TIMEOUT_SEC = 10.0
APRILTAG_TOPIC = '/apriltag/pose_base'
DEFAULT_CENTER_THRESHOLD_PX = 8.0


class AprilTagSamplingMixin:

    # ========================================================
    # 重設 AprilTag 取樣
    # ========================================================

    def reset_tag_sampling(self):
        """
        清除這一輪 AprilTag 取樣資料。

        注意：
        不在這裡清除 center_align_count，
        否則每次重新辨識都會回到第 0 次，
        最大置中次數限制會失效。
        """

        self.waiting_for_tag = False
        self.has_tag = False

        self.received_sample_count = 0

        self.tag_samples = []
        self.rj45_samples = []
        self.board_center_samples = []
        self.quat_samples = []

        self.center_error_samples = []
        self.center_correction_samples = []
        self.center_threshold_samples = []

        self.latest_tag_position_m = None
        self.latest_tag_rj45_position_m = None
        self.latest_board_center_position_m = None
        self.latest_tag_quat = None

        self.latest_centered = False
        self.latest_center_error_px = None
        self.latest_center_correction_base_m = None
        self.latest_center_threshold_px = (
            DEFAULT_CENTER_THRESHOLD_PX
        )

    # ========================================================
    # AprilTag topic callback
    # ========================================================

    def tag_pose_callback(self, msg):
        if not self.waiting_for_tag:
            return

        try:
            data = json.loads(msg.data)

            # ------------------------------------------------
            # 讀取 AprilTag Base 座標
            # ------------------------------------------------

            tag_position = [
                float(value)
                for value in data['position_m'][:3]
            ]

            if len(tag_position) != 3:
                raise ValueError(
                    'position_m must contain 3 values'
                )

            # ------------------------------------------------
            # 讀取 RJ45 Base 座標
            # ------------------------------------------------

            rj45_position = [
                float(value)
                for value in data['rj45_position_m'][:3]
            ]

            if len(rj45_position) != 3:
                raise ValueError(
                    'rj45_position_m must contain 3 values'
                )

            # ------------------------------------------------
            # 讀取板中心 Base 座標
            # ------------------------------------------------

            raw_board_center_position = data.get(
                'board_center_base_position_m'
            )

            if raw_board_center_position is None:
                raise ValueError(
                    'board_center_base_position_m is missing'
                )

            board_center_position = [
                float(value)
                for value in raw_board_center_position[:3]
            ]

            if len(board_center_position) != 3:
                raise ValueError(
                    'board_center_base_position_m '
                    'must contain 3 values'
                )

            # ------------------------------------------------
            # 讀取 AprilTag 姿態
            # ------------------------------------------------

            quat = [
                float(value)
                for value in data[
                    'orientation_quat_xyzw'
                ][:4]
            ]

            if len(quat) != 4:
                raise ValueError(
                    'orientation_quat_xyzw '
                    'must contain 4 values'
                )

            quat_norm = float(
                np.linalg.norm(quat)
            )

            if quat_norm < 1e-12:
                raise ValueError(
                    'Received quaternion norm is zero'
                )

            quat = (
                np.array(quat, dtype=float)
                / quat_norm
            ).tolist()

            # ------------------------------------------------
            # 讀取畫面中心誤差
            # ------------------------------------------------

            raw_center_error = data.get(
                'center_error_px',
                [9999.0, 9999.0]
            )

            if len(raw_center_error) < 2:
                center_error_px = [
                    9999.0,
                    9999.0
                ]
            else:
                center_error_px = [
                    float(raw_center_error[0]),
                    float(raw_center_error[1])
                ]

            # ------------------------------------------------
            # 讀取 Base 座標修正量
            # ------------------------------------------------

            raw_center_correction = data.get(
                'center_correction_base_m',
                [0.0, 0.0, 0.0]
            )

            if len(raw_center_correction) < 3:
                raise ValueError(
                    'center_correction_base_m '
                    'must contain 3 values'
                )

            center_correction_base_m = [
                float(raw_center_correction[0]),
                float(raw_center_correction[1]),
                float(raw_center_correction[2])
            ]

            # ------------------------------------------------
            # 讀取中心閥值
            # ------------------------------------------------

            center_threshold_px = float(
                data.get(
                    'center_threshold_px',
                    DEFAULT_CENTER_THRESHOLD_PX
                )
            )

            # ------------------------------------------------
            # 計算收到第幾筆
            # ------------------------------------------------

            self.received_sample_count += 1

            # 一開始先丟掉不穩定資料
            if (
                self.received_sample_count
                <= DISCARD_SAMPLE_COUNT
            ):
                self.get_logger().info(
                    f'Discard sample '
                    f'{self.received_sample_count}/'
                    f'{DISCARD_SAMPLE_COUNT}'
                )
                return

            # ------------------------------------------------
            # 正式收集資料
            # ------------------------------------------------

            self.tag_samples.append(
                tag_position
            )

            self.rj45_samples.append(
                rj45_position
            )

            self.board_center_samples.append(
                board_center_position
            )

            self.quat_samples.append(
                quat
            )

            self.center_error_samples.append(
                center_error_px
            )

            self.center_correction_samples.append(
                center_correction_base_m
            )

            self.center_threshold_samples.append(
                center_threshold_px
            )

            sample_count = len(
                self.tag_samples
            )

            self.get_logger().info(
                f'Collect sample '
                f'{sample_count}/'
                f'{MEDIAN_SAMPLE_COUNT}'
            )

            if sample_count < MEDIAN_SAMPLE_COUNT:
                return

            # ------------------------------------------------
            # 取中位數
            # ------------------------------------------------

            tag_array = np.array(
                self.tag_samples,
                dtype=float
            )

            rj45_array = np.array(
                self.rj45_samples,
                dtype=float
            )

            board_center_array = np.array(
                self.board_center_samples,
                dtype=float
            )

            center_error_array = np.array(
                self.center_error_samples,
                dtype=float
            )

            center_correction_array = np.array(
                self.center_correction_samples,
                dtype=float
            )

            self.latest_tag_position_m = (
                np.median(
                    tag_array,
                    axis=0
                ).tolist()
            )

            self.latest_tag_rj45_position_m = (
                np.median(
                    rj45_array,
                    axis=0
                ).tolist()
            )

            self.latest_board_center_position_m = (
                np.median(
                    board_center_array,
                    axis=0
                ).tolist()
            )

            self.latest_center_error_px = (
                np.median(
                    center_error_array,
                    axis=0
                ).tolist()
            )

            self.latest_center_correction_base_m = (
                np.median(
                    center_correction_array,
                    axis=0
                ).tolist()
            )

            self.latest_center_threshold_px = float(
                np.median(
                    np.array(
                        self.center_threshold_samples,
                        dtype=float
                    )
                )
            )

            # ------------------------------------------------
            # 四元數平均
            # ------------------------------------------------

            self.latest_tag_quat = (
                self.average_quaternions(
                    self.quat_samples
                )
            )

            # ------------------------------------------------
            # 使用中位數 pixel 誤差重新判斷是否置中
            # ------------------------------------------------

            error_u = float(
                self.latest_center_error_px[0]
            )

            error_v = float(
                self.latest_center_error_px[1]
            )

            threshold = float(
                self.latest_center_threshold_px
            )

            self.latest_centered = (
                abs(error_u) <= threshold
                and abs(error_v) <= threshold
            )

            self.get_logger().info(
                f'Tag median: '
                f'{self.latest_tag_position_m}'
            )

            self.get_logger().info(
                f'RJ45 median: '
                f'{self.latest_tag_rj45_position_m}'
            )

            self.get_logger().info(
                f'Board center median: '
                f'{self.latest_board_center_position_m}'
            )

            self.get_logger().info(
                f'Center median: '
                f'u={error_u:.2f}px, '
                f'v={error_v:.2f}px, '
                f'threshold={threshold:.2f}px, '
                f'centered={self.latest_centered}'
            )

            dx, dy, dz = (
                self.latest_center_correction_base_m
            )

            self.get_logger().info(
                f'Base center correction median: '
                f'dx={dx * 1000.0:.3f}mm, '
                f'dy={dy * 1000.0:.3f}mm, '
                f'dz={dz * 1000.0:.3f}mm'
            )

            self.has_tag = True
            self.waiting_for_tag = False

        except (
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError
        ) as exc:
            self.get_logger().error(
                f'Failed to parse AprilTag data: {exc}'
            )

        except Exception as exc:
            self.get_logger().error(
                f'Unexpected AprilTag callback error: {exc}'
            )

    # ========================================================
    # 四元數平均
    # ========================================================

    def average_quaternions(self, quaternions):
        """
        對四元數進行簡單平均。

        四元數 q 和 -q 表示相同旋轉，因此先統一符號，
        避免平均時互相抵消。
        """

        quat_array = np.array(
            quaternions,
            dtype=float
        )

        if quat_array.ndim != 2:
            raise ValueError(
                'Quaternion samples must be a 2D array'
            )

        if quat_array.shape[1] != 4:
            raise ValueError(
                'Each quaternion must contain 4 values'
            )

        reference = quat_array[0].copy()

        for index in range(
            quat_array.shape[0]
        ):
            if np.dot(
                quat_array[index],
                reference
            ) < 0.0:
                quat_array[index] *= -1.0

        mean_quat = np.mean(
            quat_array,
            axis=0
        )

        norm = np.linalg.norm(
            mean_quat
        )

        if norm < 1e-12:
            raise ValueError(
                'Average quaternion norm is zero'
            )

        return (
            mean_quat / norm
        ).tolist()

    # ========================================================
    # 等待 AprilTag 結果
    # ========================================================

    def wait_for_tag_result(self, state_name):
        self.get_logger().info(
            f'{state_name}: waiting for '
            f'{APRILTAG_TOPIC}'
        )

        start_time = time.time()

        while (
            rclpy.ok()
            and not self.has_tag
        ):
            time.sleep(0.05)

            elapsed_time = (
                time.time() - start_time
            )

            if elapsed_time > TAG_TIMEOUT_SEC:
                self.get_logger().error(
                    f'{state_name} timeout after '
                    f'{TAG_TIMEOUT_SEC:.1f} seconds'
                )

                self.waiting_for_tag = False
                return False

        return self.has_tag