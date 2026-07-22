#!/usr/bin/env python3

from enum import Enum


class InsertionState(str, Enum):
    IDLE = 'IDLE'
    INSERTING = 'INSERTING'
    SUCCESS = 'SUCCESS'
    JAM = 'JAM'
    EMERGENCY = 'EMERGENCY'


class InsertionDetector:
    def __init__(
        self,
        target_depth_mm=12.0,
        safe_force_threshold=5.0,
        jam_force_threshold=8.0,
        emergency_force_threshold=12.0,
        force_balance_threshold=3.0,
        required_success_count=5
    ):
        self.target_depth_mm = float(target_depth_mm)

        self.safe_force_threshold = float(
            safe_force_threshold
        )

        self.jam_force_threshold = float(
            jam_force_threshold
        )

        self.emergency_force_threshold = float(
            emergency_force_threshold
        )

        self.force_balance_threshold = float(
            force_balance_threshold
        )

        self.required_success_count = int(
            required_success_count
        )

        self.active = False
        self.start_z_mm = None
        self.success_count = 0
        self.state = InsertionState.IDLE

    def start(self, start_z_mm):
        self.start_z_mm = float(start_z_mm)
        self.success_count = 0
        self.active = True
        self.state = InsertionState.INSERTING

    def reset(self):
        self.active = False
        self.start_z_mm = None
        self.success_count = 0
        self.state = InsertionState.IDLE

    def stop(self):
        self.active = False

    def calculate_depth(self, current_z_mm):
        if self.start_z_mm is None:
            return 0.0

        # 假設手臂往下移動時，Base Z 會變小
        depth_mm = (
            float(self.start_z_mm)
            - float(current_z_mm)
        )

        return max(0.0, depth_mm)

    def update(
        self,
        current_z_mm,
        force_1,
        force_2
    ):
        current_z_mm = float(current_z_mm)
        force_1 = float(force_1)
        force_2 = float(force_2)

        depth_mm = self.calculate_depth(
            current_z_mm
        )

        # 使用絕對值，避免正負力量互相抵消
        abs_force_1 = abs(force_1)
        abs_force_2 = abs(force_2)

        total_force = (
            abs_force_1
            + abs_force_2
        )

        max_force = max(
            abs_force_1,
            abs_force_2
        )

        force_difference = abs(
            abs_force_1 - abs_force_2
        )

        state = InsertionState.INSERTING
        reason = 'Insertion in progress'

        # 最高優先：緊急力量
        if max_force >= self.emergency_force_threshold:
            state = InsertionState.EMERGENCY
            reason = 'Emergency force threshold exceeded'
            self.active = False
            self.success_count = 0

        # 力量過大，視為卡住
        elif max_force >= self.jam_force_threshold:
            state = InsertionState.JAM
            reason = 'Jam force threshold exceeded'
            self.active = False
            self.success_count = 0

        # 左右受力差距太大
        elif (
            max_force >= self.safe_force_threshold
            and force_difference
            >= self.force_balance_threshold
        ):
            state = InsertionState.JAM
            reason = 'Force imbalance detected'
            self.active = False
            self.success_count = 0

        # 到達插入深度，而且力量沒有過大
        elif depth_mm >= self.target_depth_mm:
            if max_force <= self.safe_force_threshold:
                self.success_count += 1

                if (
                    self.success_count
                    >= self.required_success_count
                ):
                    state = InsertionState.SUCCESS
                    reason = (
                        'Target depth reached '
                        'with acceptable force'
                    )
                    self.active = False

                else:
                    state = InsertionState.INSERTING
                    reason = (
                        'Target depth reached, '
                        'confirming stability'
                    )

            else:
                self.success_count = 0
                state = InsertionState.JAM
                reason = (
                    'Target depth reached '
                    'with excessive force'
                )
                self.active = False

        else:
            self.success_count = 0

        self.state = state

        return {
            'state': state.value,
            'reason': reason,
            'active': self.active,
            'start_z_mm': self.start_z_mm,
            'current_z_mm': current_z_mm,
            'depth_mm': depth_mm,
            'target_depth_mm': self.target_depth_mm,
            'force_1': force_1,
            'force_2': force_2,
            'abs_force_1': abs_force_1,
            'abs_force_2': abs_force_2,
            'total_force': total_force,
            'max_force': max_force,
            'force_difference': force_difference,
            'success_count': self.success_count,
            'required_success_count': (
                self.required_success_count
            )
        }