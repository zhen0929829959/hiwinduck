#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os


LEFT_SERIAL = '912112073118'


def generate_launch_description():

    yolo_share = get_package_share_directory('yolo')
    realsense_share = get_package_share_directory(
        'realsense2_camera'
    )

    realsense_launch = os.path.join(
        realsense_share,
        'launch',
        'rs_launch.py'
    )

    left_intrinsics = os.path.join(
        yolo_share,
        'config',
        'left_camera.yaml'
    )

    # ========================================================
    # 1. RealSense
    # ========================================================

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(
                    'realsense2_camera'
                ),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'serial_no': f"'{LEFT_SERIAL}'",

            # =================================================
            # Color stream
            # =================================================
            'enable_color': 'true',

            'rgb_camera.color_profile':
                '1920,1080,30',

            # =================================================
            # QoS
            # =================================================
            'color_qos':
                'SENSOR_DATA',

            'color_info_qos':
                'SENSOR_DATA',

            # =================================================
            # Disable unused streams
            # =================================================
            'enable_depth':
                'false',

            'enable_infra1':
                'false',

            'enable_infra2':
                'false',

            'enable_gyro':
                'false',

            'enable_accel':
                'false',

            'initial_reset':
                'false',

        }.items()
    )

    # ========================================================
    # 2. AprilTag
    # ========================================================

    apriltag_node = Node(
        package='yolo',
        executable='apriltag',
        name='apriltag',
        output='screen',
        parameters=[
            left_intrinsics,
            {
                'camera_id': 'camera',

                'image_topic':
                    '/camera/camera/color/image_raw',

                'pose_topic':
                    '/apriltag/pose_camera',

                'center_error_topic':
                    '/apriltag/center_error',

                'camera_frame':
                    'camera_color_optical_frame',

                'window_name':
                    'AprilTag'
            }
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'error'
        ]
    )

    # ========================================================
    # 3. YOLO
    # ========================================================

    yolo_node = Node(
        package='yolo',
        executable='yolo_sub',
        name='yolo',
        output='screen',
        parameters=[
            left_intrinsics,
            {
                'camera_id': 'camera',

                'image_topic':
                    '/camera/camera/color/image_raw',

                'apriltag_pose_topic':
                    '/apriltag/pose_camera',

                'freeze_pnp_z_topic':
                    '/apriltag/freeze_pnp_z',

                'detections_topic':
                    '/yolo/detections',

                'window_name':
                    'YOLO',

                'model_path':
                    'src/yolo/best.pt'
            }
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'error'
        ]
    )

    # ========================================================
    # 4. 相機 / UR 座標轉換
    # ========================================================

    camera_flange_matrix_node = Node(
        package='ur_robot_control',
        executable='camera_flange_matrix2',
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level',
            'error'
        ]
    )

    # ========================================================
    # 5. Arduino 力感測
    # ========================================================

    force_pub_node = Node(
        package='arduino_bridge',
        executable='force_pub',
        output='screen'
    )

    # ========================================================
    # 6. 插入監測
    # ========================================================

    insertion_monitor_node = Node(
        package='insertion_monitor',
        executable='insertion_monitor_node',
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level',
            'error'
        ]
    )

    # ========================================================
    # 7. UR5e Strategy
    # ========================================================

    strategy_node = Node(
        package='ur_robot_control',
        executable='strategy_example',
        output='screen'
    )

    # ========================================================
    # 啟動順序
    # ========================================================

    return LaunchDescription([
        # camera,
        realsense_launch,


        TimerAction(
            period=2.0,
            actions=[apriltag_node]
        ),

        # TimerAction(
        #     period=2.0,
        #     actions=[force_pub_node]
        # ),

        TimerAction(
            period=5.0,
            actions=[camera_flange_matrix_node]
        ),

        TimerAction(
            period=5.0,
            actions=[yolo_node]
        ),

        # TimerAction(
        #     period=5.0,
        #     actions=[insertion_monitor_node]
        # ),

        # TimerAction(
        #     period=10.0,
        #     actions=[strategy_node]
        # )
    ])