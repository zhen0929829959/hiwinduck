#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


LEFT_SERIAL = '912112073118'
RIGHT_SERIAL = '908212070822'


def generate_launch_description():
    yolo_share = get_package_share_directory('yolo')
    realsense_share = get_package_share_directory('realsense2_camera')
    realsense_launch = os.path.join(realsense_share, 'launch', 'rs_launch.py')
    left_intrinsics = os.path.join(yolo_share, 'config', 'left_camera.yaml')
    right_intrinsics = os.path.join(yolo_share, 'config', 'right_camera.yaml')
    stereo_calibration = os.path.join(yolo_share, 'config', 'stereo_camera.yaml')

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'serial_no': f"'{LEFT_SERIAL}'",
            'camera_namespace': 'camera_left',
            'camera_name': 'camera_left',
            'enable_color': 'true',
            'rgb_camera.color_profile': '1920x1080x30',
            'color_qos': 'SENSOR_DATA',
            'color_info_qos': 'SENSOR_DATA',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'initial_reset': 'false',
            'tf_prefix': 'camera_left'
        }.items()
    )

    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'serial_no': f"'{RIGHT_SERIAL}'",
            'camera_namespace': 'camera_right',
            'camera_name': 'camera_right',
            'enable_color': 'true',
            'rgb_camera.color_profile': '1920x1080x30',
            'color_qos': 'SENSOR_DATA',
            'color_info_qos': 'SENSOR_DATA',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'initial_reset': 'false',
            'tf_prefix': 'camera_right'
        }.items()
    )

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
                'window_name': 'Left AprilTag',
                'debug_image_topic': '/camera_left/apriltag/debug_image',
                'show_window': False,
            }
        ]
    )

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
                'model_path': 'src/yolo/best.pt',
                'debug_image_topic': '/camera_left/yolo/debug_image',
                'show_window': False
            }
        ]
    )

    right_yolo_node = Node(
        package='yolo',
        executable='yolo_sub',
        name='right_yolo',
        output='screen',
        parameters=[
            right_intrinsics,
            {
                'camera_id': 'right',
                'image_topic': '/camera_right/camera_right/color/image_raw',
                'apriltag_pose_topic': '/camera_right/apriltag/pose_camera',
                'freeze_pnp_z_topic': '/camera_right/apriltag/freeze_pnp_z',
                'detections_topic': '/camera_right/yolo/detections',
                'window_name': 'Right YOLO',
                'model_path': 'src/yolo/best.pt',
                'debug_image_topic': '/camera_right/yolo/debug_image',
                'show_window': False
            }
        ]
    )
    combined_view_node = Node(
        package='yolo',
        executable='combined_vision_view',
        name='combined_vision_view',
        output='screen',
        parameters=[{
            'left_apriltag_image_topic': '/camera_left/apriltag/debug_image',
            'left_yolo_detections_topic': '/camera_left/yolo/detections',
            'right_yolo_image_topic': '/camera_right/yolo/debug_image',
            'window_name': 'Stereo Vision',
            'display_width': 1600,
        }]
    )

    stereo_depth_node = Node(
        package='yolo',
        executable='stereo_depth',
        name='stereo_depth',
        output='screen',
        parameters=[stereo_calibration]
    )

    camera_flange_matrix_node = Node(
        package='ur_robot_control',
        executable='camera_flange_matrix2',
        name='camera_flange_matrix_node',
        output='screen',
        # remappings=[
        #     ('/camera_left/yolo/detections_base', '/yolo/detections_base')
        # ]
    )

    insertion_monitor_node = Node(
        package='insertion_monitor',
        executable='insertion_monitor_node',
        name='insertion_monitor_node',
        output='screen'
    )

    strategy_node = Node(
        package='ur_robot_control',
        executable='strategy_example',
        name='strategy_example',
        output='screen'
    )

    start_detection_nodes = TimerAction(
        period=3.0,
        actions=[left_apriltag_node, left_yolo_node, right_yolo_node]
    )
    start_processing_nodes = TimerAction(
        period=5.0,
        actions=[stereo_depth_node, camera_flange_matrix_node]
    )
    start_insertion_monitor = TimerAction(
        period=7.0,
        actions=[insertion_monitor_node]
    )
    start_strategy = TimerAction(
        period=10.0,
        actions=[strategy_node]
    )

    return LaunchDescription([
        left_camera,
        right_camera,
        start_detection_nodes,
        start_processing_nodes,
        start_insertion_monitor,
        # start_strategy,
        combined_view_node,
    ])
