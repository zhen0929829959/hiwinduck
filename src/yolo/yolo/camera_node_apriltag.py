
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import math
import numpy as np
from pupil_apriltags import Detector
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import json
from std_msgs.msg import String


class AprilTagSubNode(Node):
    def __init__(self):
        super().__init__('apriltag_sub')

        self.bridge = CvBridge()
        # self.depth_image = None

        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # # RealSense color camera intrinsics
        # self.fx = 1362.7784437304256
        # self.fy = 1361.8357978878923
        # self.cx = 962.1412542551096
        # self.cy = 538.5980610605569

        self.fx = 923.5084228515625
        self.fy = 923.7492065429688
        self.cx = 641.0979614257812
        self.cy = 364.9132385253906

        # AprilTag black border size, unit: meter
        self.tag_size = 0.0475

        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # self.depth_sub = self.create_subscription(
        #     Image,
        #     '/camera/camera/aligned_depth_to_color/image_raw',
        #     self.depth_callback,
        #     10
        # )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/apriltag/pose_camera',
            10
        )
        self.center_error_pub = self.create_publisher(
            String,
            '/apriltag/center_error',
            10
        )

        self.center_threshold_px = 8.0

        self.get_logger().info('AprilTag + Depth node started, unit = meter')

    # def depth_callback(self, msg):
    #     try:
    #         self.depth_image = self.bridge.imgmsg_to_cv2(
    #             msg,
    #             desired_encoding='passthrough'
    #         )
    #     except Exception as e:
    #         self.get_logger().error(f'depth cv_bridge error: {e}')

    def rotation_matrix_to_euler(self, R_mat):
        sy = math.sqrt(R_mat[0, 0] ** 2 + R_mat[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R_mat[2, 1], R_mat[2, 2])
            y = math.atan2(-R_mat[2, 0], sy)
            z = math.atan2(R_mat[1, 0], R_mat[0, 0])
        else:
            x = math.atan2(-R_mat[1, 2], R_mat[1, 1])
            y = math.atan2(-R_mat[2, 0], sy)
            z = 0

        return math.degrees(x), math.degrees(y), math.degrees(z)

    def draw_axis(self, frame, tag):
        axis_length = 0.03  # meter

        points_3d = np.float32([
            [0, 0, 0],
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, axis_length]
        ])

        rvec, _ = cv2.Rodrigues(tag.pose_R)
        tvec = tag.pose_t

        camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

        dist_coeffs = np.array([
            [0.17917354946525302],
            [-0.5132894964666538],
            [0.0007240371137749375],
            [-0.001236800492529745],
            [0.40194811602762903]
        ], dtype=np.float64)

        imgpts, _ = cv2.projectPoints(
            points_3d,
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs
        )

        imgpts = np.int32(imgpts).reshape(-1, 2)
        origin = tuple(imgpts[0])

        cv2.line(frame, origin, tuple(imgpts[1]), (0, 0, 255), 3)
        cv2.line(frame, origin, tuple(imgpts[2]), (0, 255, 0), 3)
        cv2.line(frame, origin, tuple(imgpts[3]), (255, 0, 0), 3)

    # def get_depth_xyz_m(self, u, v, tag):
    #     # AprilTag PnP pose_t is already in meters
    #     tx_m = tag.pose_t[0][0]
    #     ty_m = tag.pose_t[1][0]
    #     tz_m = tag.pose_t[2][0]
    #     used_depth = False

    #     if self.depth_image is None:
    #         return tx_m, ty_m, tz_m, used_depth

    #     h, w = self.depth_image.shape[:2]

    #     if not (2 <= u < w - 2 and 2 <= v < h - 2):
    #         return tx_m, ty_m, tz_m, used_depth

    #     roi = self.depth_image[v - 2:v + 3, u - 2:u + 3]
    #     valid = roi[roi > 0]

    #     if valid.size == 0:
    #         return tx_m, ty_m, tz_m, used_depth

    #     depth_raw = np.median(valid)

    #     # RealSense aligned_depth_to_color:
    #     # uint16 is usually millimeters, so convert to meters.
    #     # 32FC1 is usually already meters.
    #     if self.depth_image.dtype == np.uint16:
    #         z_depth_m = float(depth_raw) / 1000.0
    #     else:
    #         z_depth_m = float(depth_raw)

    #     tx_m = (u - self.cx) * z_depth_m / self.fx
    #     ty_m = (v - self.cy) * z_depth_m / self.fy
    #     tz_m = z_depth_m
    #     used_depth = True

    #     return tx_m, ty_m, tz_m, used_depth

    def image_callback(self, msg):
        try:
            # frame = self.bridge.imgmsg_to_cv2(
            #     msg,
            #     desired_encoding='passthrough'
            # )
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

            if frame is None or frame.size == 0:
                self.get_logger().warn('Received empty image')
                return      

            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                pass
            else:
                self.get_logger().warn(f'Unexpected encoding: {msg.encoding}')
                return
        except Exception as e:
            self.get_logger().error(f'color cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=self.tag_size
        )

        for tag in tags:
            tag_id = tag.tag_id
            center_x, center_y = tag.center
            u = int(center_x)
            v = int(center_y)
            error_u = float(center_x - self.cx)
            error_v = float(center_y - self.cy)

            centered = (
                abs(error_u) <= self.center_threshold_px
                and abs(error_v) <= self.center_threshold_px
            )

            corners = tag.corners.astype(int)

            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)

            # X/Y/Z unit: meter
            # tx, ty, tz, used_depth = self.get_depth_xyz_m(u, v, tag)
            # AprilTag PnP 計算出的相機座標，單位為公尺
            tx = float(tag.pose_t[0][0])
            ty = float(tag.pose_t[1][0])
            tz = float(tag.pose_t[2][0])
            center_data = {
                'tag_id': int(tag_id),
                'tag_center_px': [float(center_x), float(center_y)],
                'image_center_px': [float(self.cx), float(self.cy)],
                'error_px': [error_u, error_v],
                'center_threshold_px': self.center_threshold_px,
                'centered': bool(centered),

                # AprilTag 相對相機的位置
                # tx > 0：Tag 在畫面右邊
                # ty > 0：Tag 在畫面下方
                'correction_camera_m': [
                    float(tx),
                    float(ty),
                    0.0
                ]
            }

            center_msg = String()
            center_msg.data = json.dumps(center_data)
            self.center_error_pub.publish(center_msg)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'camera_color_optical_frame'

            pose_msg.pose.position.x = tx
            pose_msg.pose.position.y = ty
            pose_msg.pose.position.z = tz

            quat = R.from_matrix(tag.pose_R).as_quat()

            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            self.pose_pub.publish(pose_msg)

            rx, ry, rz = self.rotation_matrix_to_euler(tag.pose_R)
            # source = 'Depth' if used_depth else 'AprilTag'
            source = 'AprilTag PnP'

            self.get_logger().info(
                f'ID:{tag_id} | '
                f'X:{tx:.4f} m, Y:{ty:.4f} m, Z:{tz:.4f} m ({source}) | '
                f'Rx:{rx:.1f}, Ry:{ry:.1f}, Rz:{rz:.1f}'
            )

            cv2.putText(
                frame,
                f'ID:{tag_id}',
                (u, v - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f'Z:{tz:.3f}m',
                (u, v + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2
            )

            self.draw_axis(frame, tag)
            image_center = (
                int(round(self.cx)),
                int(round(self.cy))
            )

            threshold = int(round(self.center_threshold_px))

            # 畫面中心
            cv2.drawMarker(
                frame,
                image_center,
                (255, 0, 255),
                cv2.MARKER_CROSS,
                25,
                2
            )

            # 可接受的置中範圍
            cv2.rectangle(
                frame,
                (
                    image_center[0] - threshold,
                    image_center[1] - threshold
                ),
                (
                    image_center[0] + threshold,
                    image_center[1] + threshold
                ),
                (255, 255, 0),
                2
            )

        # cv2.imshow('AprilTag RealSense Detection', frame)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagSubNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

