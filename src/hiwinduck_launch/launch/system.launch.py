#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. 啟動 RealSense 官方 launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        )
    )

    # 2. 啟動 AprilTag
    apriltag_node = Node(
        package='yolo',
        executable='apriltag',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 3. 啟動 YOLO
    yolo_node = Node(
        package='yolo',
        executable='yolo_sub',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 4. 啟動 HIWIN 手臂通訊 server
    hiwin_server_node = Node(
        package='hiwin_libmodbus',
        executable='hiwinlibmodbus_server',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 5. 啟動相機與手臂座標轉換程式
    camera_flange_matrix_node = Node(
        package='hiwin_example',
        executable='camera_flange_matrix2',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    force_pub_node = Node(
        package='arduino_bridge',
        executable='force_pub',
        output='screen',
        # arguments=['--ros-args', '--log-level', 'error']
    )

    insertion_monitor_node = Node(
        package='insertion_monitor',
        executable='insertion_monitor_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 6. 啟動手臂策略程式
    strategy_node = Node(
        package='hiwin_example',
        executable='strategy_example',
        output='screen'
    )

    return LaunchDescription([
        # 啟動相機
        realsense_launch,

        # 啟動 AprilTag
        TimerAction(
            period=2.0,
            actions=[apriltag_node]
        ),

        # # 啟動 Arduino 力感測
        # TimerAction(
        #     period=2.0,
        #     actions=[force_pub_node]
        # ),

        # 啟動座標轉換
        TimerAction(
            period=5.0,
            actions=[camera_flange_matrix_node]
        ),

        # 啟動 YOLO
        TimerAction(
            period=5.0,
            actions=[yolo_node]
        ),

        # # 啟動 HIWIN server
        # TimerAction(
        #     period=5.0,
        #     actions=[hiwin_server_node]
        # ),

        # # 啟動插入監測
        # TimerAction(
        #     period=5.0,
        #     actions=[insertion_monitor_node]
        # ),

        # 啟動手臂策略
        # TimerAction(
        #     period=10.0,
        #     actions=[strategy_node]
        # )
    ])