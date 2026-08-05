from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


# ============================================================
# 兩台 RealSense 的序號
# 執行以下指令查詢：
# rs-enumerate-devices | grep -E "Name|Serial Number"
# ============================================================

LEFT_SERIAL = '912112073118'
RIGHT_SERIAL = '908212070937'


def generate_launch_description():

    yolo_share = get_package_share_directory('yolo')
    realsense_share = get_package_share_directory('realsense2_camera')

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

    right_intrinsics = os.path.join(
        yolo_share,
        'config',
        'right_camera.yaml'
    )

    stereo_calibration = os.path.join(
        yolo_share,
        'config',
        'stereo_camera.yaml'
    )

    # ========================================================
    # 左 RealSense
    #
    # topic：
    # /camera_left/camera_left/color/image_raw
    # /camera_left/camera_left/color/camera_info
    # ========================================================

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            realsense_launch
        ),
        launch_arguments={
            'serial_no': f"'{LEFT_SERIAL}'",

            'camera_namespace': 'camera_left',
            'camera_name': 'camera_left',

            'enable_color': 'true',
            'rgb_camera.color_profile': '1920x1080x30',

            # 目前只使用彩色影像，先關閉其他串流
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',

            # 多相機時先不要同時 reset，避免互相影響
            'initial_reset': 'false',

            # 避免左右相機 TF 名稱衝突
            'tf_prefix': 'camera_left'
        }.items()
    )

    # ========================================================
    # 右 RealSense
    #
    # topic：
    # /camera_right/camera_right/color/image_raw
    # /camera_right/camera_right/color/camera_info
    # ========================================================

    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            realsense_launch
        ),
        launch_arguments={
            'serial_no': f"'{RIGHT_SERIAL}'",

            'camera_namespace': 'camera_right',
            'camera_name': 'camera_right',

            'enable_color': 'true',
            'rgb_camera.color_profile': '1920x1080x30',

            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',

            'initial_reset': 'false',

            'tf_prefix': 'camera_right'
        }.items()
    )

    # ========================================================
    # 左 AprilTag
    # ========================================================

    left_apriltag = Node(
        package='yolo',
        executable='apriltag',
        name='left_apriltag',
        output='screen',
        parameters=[
            left_intrinsics,
            {
                'camera_id': 'left',

                'image_topic':
                    '/camera_left/camera_left/color/image_raw',

                'pose_topic':
                    '/camera_left/apriltag/pose_camera',

                'center_error_topic':
                    '/camera_left/apriltag/center_error',

                'camera_frame':
                    'camera_left_color_optical_frame',

                'window_name':
                    'Left AprilTag'
            }
        ]
    )

    # ========================================================
    # 右 AprilTag
    # 沒有啟動
    # ========================================================

    right_apriltag = Node(
        package='yolo',
        executable='apriltag',
        name='right_apriltag',
        output='screen',
        parameters=[
            right_intrinsics,
            {
                'camera_id': 'right',

                'image_topic':
                    '/camera_right/camera_right/color/image_raw',

                'pose_topic':
                    '/camera_right/apriltag/pose_camera',

                'center_error_topic':
                    '/camera_right/apriltag/center_error',

                'camera_frame':
                    'camera_right_color_optical_frame',

                'window_name':
                    'Right AprilTag'
            }
        ]
    )

    # ========================================================
    # 左 YOLO
    # ========================================================

    left_yolo = Node(
        package='yolo',
        executable='yolo_sub',
        name='left_yolo',
        output='screen',
        parameters=[
            left_intrinsics,
            {
                'camera_id': 'left',

                'image_topic':
                    '/camera_left/camera_left/color/image_raw',

                'apriltag_pose_topic':
                    '/camera_left/apriltag/pose_camera',

                'freeze_pnp_z_topic':
                    '/camera_left/apriltag/freeze_pnp_z',

                'detections_topic':
                    '/camera_left/yolo/detections',

                'window_name':
                    'Left YOLO',

                'model_path':
                    'src/yolo/best.pt'
            }
        ]
    )

    # ========================================================
    # 右 YOLO
    #
    # 注意：
    # 右 YOLO 目前訂閱右 AprilTag pose。
    # 如果沒有啟動 right_apriltag，右 YOLO 就無法取得
    # frozen_tag_z_m，因此 camera_xyz 會是 None。
    # 但 YOLO 偵測與 pixel_center 還是能正常輸出。
    # ========================================================

    right_yolo = Node(
        package='yolo',
        executable='yolo_sub',
        name='right_yolo',
        output='screen',
        parameters=[
            right_intrinsics,
            {
                'camera_id': 'right',

                'image_topic':
                    '/camera_right/camera_right/color/image_raw',

                'apriltag_pose_topic':
                    '/camera_right/apriltag/pose_camera',

                'freeze_pnp_z_topic':
                    '/camera_right/apriltag/freeze_pnp_z',

                'detections_topic':
                    '/camera_right/yolo/detections',

                'window_name':
                    'Right YOLO',

                'model_path':
                    'src/yolo/best.pt'
            }
        ]
    )


    # # ========================================================
    # # 左 YOLO 定位
    # #
    # # 注意：
    # # 右 YOLO 目前訂閱右 AprilTag pose。
    # # 如果沒有啟動 right_apriltag，右 YOLO 就無法取得
    # # frozen_tag_z_m，因此 camera_xyz 會是 None。
    # # 但 YOLO 偵測與 pixel_center 還是能正常輸出。
    # # ========================================================
    # left_yolo = Node(
    #     package='yolo',
    #     executable='yolo_left',
    #     name='left_yolo',
    #     output='screen',
    #     parameters=[
    #         left_intrinsics,
    #         {
    #             'camera_id': 'left',
    #             'image_topic':
    #                 '/camera_left/camera_left/color/image_raw',
    #             'apriltag_pose_topic':
    #                 '/camera_left/apriltag/pose_camera',
    #             'freeze_pnp_z_topic':
    #                 '/camera_left/apriltag/freeze_pnp_z',
    #             'detections_topic':
    #                 '/camera_left/yolo/detections',
    #             'window_name':
    #                 'Left YOLO',

    #             # 主相機負責編號
    #             'identity_mode': 'master',

    #             # 同類別由左到右編號
    #             'sort_axis': "x",
    #             'sort_reverse': False,

    #             'model_path':
    #                 'src/yolo/best.pt'
    #         }
    #     ]
    # )

    # right_yolo = Node(
    #     package='yolo',
    #     executable='yolo_left',
    #     name='right_yolo',
    #     output='screen',
    #     parameters=[
    #         right_intrinsics,
    #         {
    #             'camera_id': 'right',
    #             'image_topic':
    #                 '/camera_right/camera_right/color/image_raw',
    #             'apriltag_pose_topic':
    #                 '/camera_right/apriltag/pose_camera',
    #             'freeze_pnp_z_topic':
    #                 '/camera_right/apriltag/freeze_pnp_z',
    #             'detections_topic':
    #                 '/camera_right/yolo/detections',
    #             'window_name':
    #                 'Right YOLO',

    #             # 側相機不決定 RJ45_0 / RJ45_1
    #             'identity_mode': 'side',

    #             # side 模式不使用這兩個參數做實體編號
    #             'sort_axis': "vertical",
    #             'sort_reverse': False,

    #             'model_path':
    #                 'src/yolo/best.pt'
    #         }
    #     ]
    # )

    stereo_depth = Node(
        package='yolo',
        executable='stereo_depth',
        name='stereo_depth',
        output='screen',
        parameters=[
            stereo_calibration
        ]
    )



    # ========================================================
    # 延遲啟動視覺節點
    #
    # 先讓 RealSense 初始化完成，避免 YOLO / AprilTag 一啟動時
    # 找不到 image topic。
    # ========================================================

    start_vision_nodes = TimerAction(
        period=3.0,
        actions=[
            left_apriltag,
            left_yolo,
            right_yolo,
            stereo_depth

            # 需要右 AprilTag 時，改成：
            # left_apriltag,
            # right_apriltag,
            # left_yolo,
            # right_yolo
        ]
    )

    return LaunchDescription([
        left_camera,
        right_camera,
        start_vision_nodes
    ])