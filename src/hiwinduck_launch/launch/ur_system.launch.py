#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


LEFT_SERIAL = '912112073118'


def generate_launch_description():
    yolo_share = get_package_share_directory('yolo')
    realsense_share = get_package_share_directory('realsense2_camera')

    realsense_launch_path = os.path.join(
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
    # 1. 左 RealSense
    #
    # Image：
    # /camera_left/camera_left/color/image_raw
    #
    # CameraInfo：
    # /camera_left/camera_left/color/camera_info
    # ========================================================

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            realsense_launch_path
        ),
        launch_arguments={
            'serial_no': f"'{LEFT_SERIAL}'",

            # 相機 Namespace 與名稱
            'camera_namespace': 'camera_left',
            'camera_name': 'camera_left',

            # Color stream
            'enable_color': 'true',
            'rgb_camera.color_profile': '1920x1080x30',

            # QoS
            'color_qos': 'SENSOR_DATA',
            'color_info_qos': 'SENSOR_DATA',

            # Disable unused streams
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',

            'initial_reset': 'false',

            # 避免 TF 名稱衝突
            'tf_prefix': 'camera_left'
        }.items()
    )

    # ========================================================
    # 2. 左 AprilTag
    # ========================================================

    left_apriltag_node = Node(
        package='yolo',
        executable='apriltag',
        name='left_apriltag',
        output='screen',
        parameters=[
            left_intrinsics,
            {
                'camera_id': 'left',
                'image_topic': '/camera_left/camera_left/color/image_raw',
                'pose_topic': '/camera_left/apriltag/pose_camera',
                'center_error_topic': '/camera_left/apriltag/center_error',
                'camera_frame': 'camera_color_optical_frame',
                'window_name': 'Left AprilTag'
            }
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'error'
        ]
    )

    # ========================================================
    # 3. 左 YOLO
    # ========================================================

    left_yolo_node = Node(
        package='yolo',
        executable='yolo_sub',
        name='left_yolo',
        output='screen',
        parameters=[
            left_intrinsics,
            {
                'camera_id': 'left',
                'image_topic': '/camera_left/camera_left/color/image_raw',
                'apriltag_pose_topic': '/camera_left/apriltag/pose_camera',
                'freeze_pnp_z_topic': '/camera_left/apriltag/freeze_pnp_z',
                'detections_topic': '/camera_left/yolo/detections',
                'window_name': 'Left YOLO',
                'model_path': 'src/yolo/best.pt'
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
        left_camera,

        TimerAction(
            period=2.0,
            actions=[left_apriltag_node]
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
            actions=[left_yolo_node]
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