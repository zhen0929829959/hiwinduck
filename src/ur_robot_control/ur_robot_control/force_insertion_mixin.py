#!/usr/bin/env python3

import time
from threading import Event

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String


class ForceInsertionMixin:
    """
    Simulation / URSim-only insertion flow.

    Responsibilities:
      1. Collect a force baseline before insertion
      2. Monitor delta Fz while insertion is running
      3. Request a MoveIt trajectory stop when the threshold is exceeded
      4. Calculate insertion depth after the stop
      5. Report SUCCESS if depth >= threshold
      6. Otherwise report COLLISION and return to the saved start pose

    IMPORTANT:
      /trajectory_execution_event "stop" is a software trajectory stop.
      It is NOT an emergency stop or protective stop.
    """

    def init_force_insertion(self):
        # ------------------------------------------------------------
        # Simulation guard
        # ------------------------------------------------------------
        self.declare_parameter(
            'force_insertion_simulation_only',
            True
        )

        self.force_insertion_simulation_only = (
            self.get_parameter(
                'force_insertion_simulation_only'
            ).get_parameter_value().bool_value
        )

        if not self.force_insertion_simulation_only:
            raise RuntimeError(
                'ForceInsertionMixin is configured for simulation/URSim '
                'validation only. Real-hardware insertion is intentionally '
                'disabled in this integration.'
            )

        # ------------------------------------------------------------
        # Parameters copied from the standalone force test
        # ------------------------------------------------------------
        self.force_limit_n = 30.0
        self.force_baseline_sample_count = 50
        self.force_trigger_count_required = 3

        self.insert_distance_mm = 150.0
        self.success_depth_threshold_mm = 90.0

        self.insert_velocity = 6
        self.insert_acceleration = 6

        self.return_velocity = 6
        self.return_acceleration = 6

        self.force_baseline_timeout_sec = 5.0
        self.max_insertion_time_sec = 100.0
        self.stop_settle_sec = 2.0

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------
        self.force_samples = []
        self.force_baseline = None
        self.force_trigger_count = 0

        self.force_monitoring = False
        self.force_stop_requested = False

        self.latest_force = None

        self.insertion_start_pose = None
        self.insertion_stopped_pose = None

        self.insertion_depth_mm = 0.0
        self.insertion_result = None
        self.insertion_stop_reason = None

        if not hasattr(
            self,
            'insertion_stop_event'
        ):
            self.insertion_stop_event = Event()

        # ------------------------------------------------------------
        # ROS I/O
        # ------------------------------------------------------------
        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.force_insertion_callback,
            10
        )

        self.trajectory_event_pub = self.create_publisher(
            String,
            '/trajectory_execution_event',
            10
        )

        self.get_logger().info(
            'ForceInsertionMixin initialized '
            '(simulation / URSim only)'
        )

    # ================================================================
    # Force callback
    # ================================================================

    def force_insertion_callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        self.latest_force = [
            fx,
            fy,
            fz
        ]

        # Only collect / monitor when the insertion stage enables it.
        if not self.force_monitoring:
            return

        # ------------------------------------------------------------
        # Baseline
        # ------------------------------------------------------------
        if self.force_baseline is None:
            self.force_samples.append(
                [
                    fx,
                    fy,
                    fz
                ]
            )

            if (
                len(self.force_samples)
                >= self.force_baseline_sample_count
            ):
                n = len(
                    self.force_samples
                )

                baseline_fx = (
                    sum(
                        sample[0]
                        for sample in self.force_samples
                    )
                    / n
                )

                baseline_fy = (
                    sum(
                        sample[1]
                        for sample in self.force_samples
                    )
                    / n
                )

                baseline_fz = (
                    sum(
                        sample[2]
                        for sample in self.force_samples
                    )
                    / n
                )

                self.force_baseline = [
                    baseline_fx,
                    baseline_fy,
                    baseline_fz
                ]

                self.get_logger().info(
                    'Force baseline ready'
                )

                self.get_logger().info(
                    f'Baseline = '
                    f'[{baseline_fx:.2f}, '
                    f'{baseline_fy:.2f}, '
                    f'{baseline_fz:.2f}] N'
                )

            return

        # Already requested stop.
        if self.force_stop_requested:
            return

        # ------------------------------------------------------------
        # Delta force
        # ------------------------------------------------------------
        dfx = (
            fx
            - self.force_baseline[0]
        )

        dfy = (
            fy
            - self.force_baseline[1]
        )

        dfz = (
            fz
            - self.force_baseline[2]
        )

        self.get_logger().info(
            f'Insertion dF = '
            f'[{dfx:+.2f}, '
            f'{dfy:+.2f}, '
            f'{dfz:+.2f}] N'
        )

        # ------------------------------------------------------------
        # Threshold: only Fz
        # ------------------------------------------------------------
        if abs(dfz) > self.force_limit_n:
            self.force_trigger_count += 1
        else:
            self.force_trigger_count = 0

        if (
            self.force_trigger_count
            < self.force_trigger_count_required
        ):
            return

        self.force_stop_requested = True
        self.insertion_stop_reason = (
            'FORCE_LIMIT'
        )

        self.get_logger().warning(
            'Force limit exceeded during insertion'
        )

        self.get_logger().warning(
            f'dFz = {dfz:.2f} N'
        )

        self.insertion_stop_event.set()
        self.stop_force_insertion_trajectory()

    # ================================================================
    # Prepare
    # ================================================================

    def prepare_force_insertion(self):
        if not self.force_insertion_simulation_only:
            self.get_logger().error(
                'Force insertion is disabled outside simulation/URSim'
            )
            return False

        self.force_samples = []
        self.force_baseline = None
        self.force_trigger_count = 0

        self.force_stop_requested = False
        self.insertion_stop_reason = None
        self.insertion_stop_event.clear()

        self.insertion_depth_mm = 0.0
        self.insertion_result = None
        self.insertion_stopped_pose = None

        self.force_monitoring = True

        self.get_logger().info(
            'Waiting for force baseline...'
        )

        start_time = time.time()

        while (
            time.time() - start_time
            < self.force_baseline_timeout_sec
        ):
            if self.force_baseline is not None:
                self.get_logger().info(
                    'Force baseline complete'
                )
                return True

            time.sleep(0.05)

        self.force_monitoring = False

        self.get_logger().error(
            'Force baseline timeout'
        )

        return False

    # ================================================================
    # Start insertion
    # ================================================================

    def run_force_insertion(self):
        if not self.force_insertion_simulation_only:
            self.get_logger().error(
                'Force insertion is disabled outside simulation/URSim'
            )
            return False

        if self.insertion_start_pose is None:
            self.get_logger().error(
                'Insertion start pose is missing'
            )
            return False

        start_pose = list(
            self.insertion_start_pose
        )

        target_z = (
            start_pose[2]
            - self.insert_distance_mm
        )

        target_pose = self.create_pose(
            start_pose[0],
            start_pose[1],
            target_z,
            start_pose[3],
            start_pose[4],
            start_pose[5]
        )

        self.get_logger().info(
            f'Insertion start Z = '
            f'{start_pose[2]:.2f} mm'
        )

        self.get_logger().info(
            f'Insertion target Z = '
            f'{target_z:.2f} mm'
        )

        if not self.move_pose_lin(
            target_pose,
            self.insert_velocity,
            self.insert_acceleration,
            holding=False
        ):
            self.force_monitoring = False

            self.get_logger().error(
                'Insertion motion failed to start'
            )

            return False

        start_time = time.time()

        while True:
            if self.insertion_stop_event.is_set():
                break

            if (
                time.time() - start_time
                > self.max_insertion_time_sec
            ):
                self.force_monitoring = False

                self.get_logger().error(
                    'Insertion timeout'
                )

                self.stop_force_insertion_trajectory()
                return False

            time.sleep(0.01)

        self.force_monitoring = False

        # Software settle time after stop request.
        time.sleep(
            self.stop_settle_sec
        )

        return True

    # ================================================================
    # Check result
    # ================================================================

    def check_force_insertion_result(self):
        if self.insertion_start_pose is None:
            self.get_logger().error(
                'Insertion start pose is missing'
            )
            return False

        stopped_pose = (
            self.get_current_robot_pose()
        )

        if stopped_pose is None:
            self.get_logger().error(
                'Cannot read stopped pose'
            )
            return False

        self.insertion_stopped_pose = list(
            stopped_pose
        )

        start_z = (
            self.insertion_start_pose[2]
        )

        stopped_z = (
            self.insertion_stopped_pose[2]
        )

        self.insertion_depth_mm = (
            start_z
            - stopped_z
        )

        self.get_logger().warning(
            f'Insertion start Z = '
            f'{start_z:.2f} mm'
        )

        self.get_logger().warning(
            f'Stopped Z = '
            f'{stopped_z:.2f} mm'
        )

        self.get_logger().warning(
            f'Insertion depth = '
            f'{self.insertion_depth_mm:.2f} mm'
        )

        # ------------------------------------------------------------
        # Success
        # ------------------------------------------------------------
        if (
            self.insertion_depth_mm
            >= self.success_depth_threshold_mm
        ):
            self.insertion_result = (
                'SUCCESS'
            )

            self.get_logger().warning(
                '=========================='
            )
            self.get_logger().warning(
                'INSERTION SUCCESS'
            )
            self.get_logger().warning(
                f'Depth >= '
                f'{self.success_depth_threshold_mm:.1f} mm'
            )
            self.get_logger().warning(
                '=========================='
            )

            return True

        # ------------------------------------------------------------
        # Early collision
        # ------------------------------------------------------------
        self.insertion_result = (
            'COLLISION'
        )

        self.get_logger().warning(
            '=========================='
        )
        self.get_logger().warning(
            'INSERTION COLLISION'
        )
        self.get_logger().warning(
            f'Depth < '
            f'{self.success_depth_threshold_mm:.1f} mm'
        )
        self.get_logger().warning(
            'Returning to insertion start pose'
        )
        self.get_logger().warning(
            '=========================='
        )

        return self.return_force_insertion_to_start()

    # ================================================================
    # Stop current MoveIt trajectory
    # ================================================================

    def stop_force_insertion_trajectory(self):
        msg = String()
        msg.data = 'stop'

        self.trajectory_event_pub.publish(
            msg
        )

        self.get_logger().warning(
            'Insertion trajectory STOP requested'
        )

    # ================================================================
    # Return to saved start pose
    # ================================================================

    def return_force_insertion_to_start(self):

        if self.insertion_start_pose is None:

            self.get_logger().error(
                'Insertion start pose is missing'
            )

            return False

        start_pose = (
            self.insertion_start_pose
        )

        return_pose = self.create_pose(
            start_pose[0],
            start_pose[1],
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        )

        self.get_logger().info(
            'Returning to insertion start pose'
        )

        return self.move_pose_lin(
            return_pose,
            self.return_velocity,
            self.return_acceleration,
            holding=True
        )
