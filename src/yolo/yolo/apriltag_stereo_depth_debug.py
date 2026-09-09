#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)


class AprilTagStereoDepthDebug(Node):
    def __init__(self):
        super().__init__('apriltag_stereo_depth_debug')
        self.declare_parameter('left_image_topic', '/camera_left/camera_left/color/image_raw')
        self.declare_parameter('right_image_topic', '/camera_right/camera_right/color/image_raw')
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('tag_size_m', 0.05)
        self.declare_parameter('max_time_difference_sec', 0.08)
        self.declare_parameter('extrinsic_mode', 'original')
        self.declare_parameter('max_valid_reprojection_error_px', 10.0)
        self.declare_parameter('show_images', True)
        self.declare_parameter('left_k', [1370.481966698142, 0.0, 957.8839678123309, 0.0, 1368.2282506378035, 541.0615417682238, 0.0, 0.0, 1.0])
        self.declare_parameter('left_d', [0.18970955702068168, -0.5620843081215285, 0.0014242865497269136, -0.002233374919288368, 0.4613305846866371])
        self.declare_parameter('right_k', [1376.2993702005285, 0.0, 966.4965648414827, 0.0, 1374.3873733700716, 554.7627465081724, 0.0, 0.0, 1.0])
        self.declare_parameter('right_d', [0.17302852476860411, -0.558236486919957, 0.0018853562229434549, 0.001071749355689855, 0.5066598285552415])
        # 由兩份 hand-eye 依 H_right_left = inv(H_flange_right) @ H_flange_left 推導。
        # 定義：X_right = R * X_left + T；left=camera6，right=camera2。
        self.declare_parameter('stereo_rotation', [
            0.9996585911399021,
            0.02484022276150315,
            -0.008103363088553656,
            -0.006910731209812965,
            0.5504526183653868,
            0.8348378026472181,
            0.025198074417140493,
            -0.8344967814604491,
            0.5504363530671095
        ])

        self.declare_parameter('stereo_translation', [
            -0.001407604632185056,
            -0.12859698183565704,
            0.061440498218346226
        ])
        self.left_k = np.asarray(self.get_parameter('left_k').value, dtype=np.float64).reshape(3, 3)
        self.right_k = np.asarray(self.get_parameter('right_k').value, dtype=np.float64).reshape(3, 3)
        self.left_d = np.asarray(self.get_parameter('left_d').value, dtype=np.float64).reshape(-1, 1)
        self.right_d = np.asarray(self.get_parameter('right_d').value, dtype=np.float64).reshape(-1, 1)
        self.rotation = np.asarray(self.get_parameter('stereo_rotation').value, dtype=np.float64).reshape(3, 3)
        self.translation = np.asarray(self.get_parameter('stereo_translation').value, dtype=np.float64).reshape(3, 1)
        self.tag_id = int(self.get_parameter('tag_id').value)
        self.tag_size = float(self.get_parameter('tag_size_m').value)
        self.max_dt = float(self.get_parameter('max_time_difference_sec').value)
        self.extrinsic_mode = str(self.get_parameter('extrinsic_mode').value).lower()
        self.max_valid_reprojection_error = float(self.get_parameter('max_valid_reprojection_error_px').value)
        if self.extrinsic_mode not in ('auto', 'original', 'inverse'):
            raise ValueError("extrinsic_mode must be 'auto', 'original', or 'inverse'")
        self.show_images = bool(self.get_parameter('show_images').value)
        self.bridge = CvBridge()
        self.detector = Detector(families='tag36h11', nthreads=4, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)
        self.latest = {'left': None, 'right': None}
        self.display_frames = {'left': None, 'right': None}
        self.frame_count = {'left': 0, 'right': 0}
        self.detect_count = {'left': 0, 'right': 0}
        self.callback_enter_count = {'left': 0, 'right': 0}
        self.last_pair_key = None
        self.result_pub = self.create_publisher(String, '/apriltag/stereo_depth_debug', 10)
        self.create_subscription(Image, self.get_parameter('left_image_topic').value, lambda msg: self.image_callback(msg, 'left'), IMAGE_QOS)
        self.create_subscription(Image, self.get_parameter('right_image_topic').value, lambda msg: self.image_callback(msg, 'right'), IMAGE_QOS)
        self.create_timer(1.0, self.watchdog_callback)
        det_r = np.linalg.det(self.rotation)
        ortho_error = np.linalg.norm(self.rotation @ self.rotation.T - np.eye(3))
        self.get_logger().info(f'Started | tag_id={self.tag_id}, tag_size={self.tag_size:.4f} m | baseline={np.linalg.norm(self.translation):.4f} m | det(R)={det_r:.6f}, R_orthogonal_error={ortho_error:.2e}')

    def watchdog_callback(self):
        self.get_logger().info(f'WATCHDOG | callback entered L/R={self.callback_enter_count["left"]}/{self.callback_enter_count["right"]} | processed L/R={self.frame_count["left"]}/{self.frame_count["right"]} | Tag found L/R={self.detect_count["left"]}/{self.detect_count["right"]}')

    @staticmethod
    def stamp_sec(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def to_bgr(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'bgr8':
            return frame
        if msg.encoding == 'mono8':
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        raise ValueError(f'unsupported image encoding: {msg.encoding}')

    def detect_tag(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for tag in self.detector.detect(gray, estimate_tag_pose=False):
            if int(tag.tag_id) == self.tag_id:
                return tag.corners.astype(np.float64), tag.center.astype(np.float64)
        return None, None

    def image_callback(self, msg, side):
        self.callback_enter_count[side] += 1
        self.frame_count[side] += 1
        try:
            frame = self.to_bgr(msg)
            corners, center = self.detect_tag(frame)
        except Exception as exc:
            self.get_logger().error(f'{side} image error: {exc}', throttle_duration_sec=2.0)
            return
        if corners is None:
            self.get_logger().warn(f'{side}: image received but Tag ID {self.tag_id} not found | frames={self.frame_count[side]}', throttle_duration_sec=2.0)
            if self.show_images:
                self.draw_debug_status(frame, side, msg, None, None, 'TAG NOT FOUND', (0, 0, 255))
                self.display_frames[side] = frame
                self.show_combined_images()
            return
        self.detect_count[side] += 1
        k = self.left_k if side == 'left' else self.right_k
        d = self.left_d if side == 'left' else self.right_d
        pnp = self.solve_pnp(corners, k, d)
        self.latest[side] = {'stamp': self.stamp_sec(msg), 'corners': corners, 'center': center, 'pnp': pnp, 'frame': frame}
        other_side = 'right' if side == 'left' else 'left'
        other = self.latest[other_side]
        if other is None:
            status = f'WAITING FOR {other_side.upper()} TAG'
            color = (0, 165, 255)
        else:
            dt = abs(self.stamp_sec(msg) - other['stamp'])
            status = f'dt={dt * 1000:.1f}ms / limit={self.max_dt * 1000:.1f}ms'
            color = (0, 255, 0) if dt <= self.max_dt else (0, 0, 255)
        if self.show_images:
            self.draw_debug_status(frame, side, msg, corners, center, status, color, pnp)
            self.display_frames[side] = frame
            self.show_combined_images()
        self.try_compare()

    def show_combined_images(self):
        left = self.display_frames['left']
        right = self.display_frames['right']
        if left is None or right is None:
            return
        combined = np.hstack((left, right))
        scale = min(0.5, 1500.0 / combined.shape[1])
        combined = cv2.resize(combined, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        cv2.imshow('AprilTag stereo - left | right', combined)
        cv2.waitKey(1)

    def draw_debug_status(self, frame, side, msg, corners, center, status, color, pnp=None):
        if corners is not None:
            cv2.polylines(frame, [np.rint(corners).astype(np.int32)], True, (0, 255, 0), 3)
            cv2.circle(frame, tuple(np.rint(center).astype(int)), 7, (0, 0, 255), -1)
        lines = [f'{side.upper()} image OK | frame={self.frame_count[side]}', f'stamp={self.stamp_sec(msg):.6f}', f'Tag ID {self.tag_id}: {"FOUND" if corners is not None else "NOT FOUND"}']
        if center is not None:
            lines.extend([f'center=({center[0]:.1f}, {center[1]:.1f})', f'PnP Z={float(pnp["tvec"][2, 0]):.4f} m | RMSE={pnp["reprojection_rmse_px"]:.2f}px'])
        lines.append(status)
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (900, 30 + len(lines) * 36), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        for i, line in enumerate(lines):
            line_color = color if i == len(lines) - 1 else (255, 255, 255)
            cv2.putText(frame, line, (25, 45 + i * 36), cv2.FONT_HERSHEY_SIMPLEX, 0.85, line_color, 2)

    def solve_pnp(self, corners, k, d):
        half = self.tag_size / 2.0
        object_points = np.array([[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(object_points, corners, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            raise RuntimeError('solvePnP failed')
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, k, d)
        error = np.linalg.norm(projected.reshape(-1, 2) - corners, axis=1)
        return {'rvec': rvec, 'tvec': tvec, 'reprojection_rmse_px': float(np.sqrt(np.mean(error ** 2)))}

    def triangulate(self, left_px, right_px, rotation, translation):
        left_n = cv2.undistortPoints(np.asarray(left_px).reshape(1, 1, 2), self.left_k, self.left_d).reshape(2, 1)
        right_n = cv2.undistortPoints(np.asarray(right_px).reshape(1, 1, 2), self.right_k, self.right_d).reshape(2, 1)
        p_left = np.hstack((np.eye(3), np.zeros((3, 1))))
        p_right = np.hstack((rotation, translation))
        homogeneous = cv2.triangulatePoints(p_left, p_right, left_n, right_n).reshape(4)
        return (homogeneous[:3] / homogeneous[3]).reshape(3, 1)

    @staticmethod
    def project_point(point, k, d, rotation=None, translation=None):
        if rotation is None:
            rotation, translation = np.eye(3), np.zeros((3, 1))
        rvec, _ = cv2.Rodrigues(rotation)
        pixel, _ = cv2.projectPoints(point.reshape(1, 3), rvec, translation, k, d)
        return pixel.reshape(2)

    def evaluate_extrinsic(self, name, rotation, translation, left_center, right_center):
        xyz = self.triangulate(left_center, right_center, rotation, translation)
        xyz_right = rotation @ xyz + translation
        left_reprojected = self.project_point(xyz, self.left_k, self.left_d)
        right_reprojected = self.project_point(xyz, self.right_k, self.right_d, rotation, translation)
        left_error = float(np.linalg.norm(left_reprojected - left_center))
        right_error = float(np.linalg.norm(right_reprojected - right_center))
        return {'name': name, 'rotation': rotation, 'translation': translation, 'xyz': xyz, 'xyz_right': xyz_right, 'left_error': left_error, 'right_error': right_error, 'total_error': left_error + right_error}

    @staticmethod
    def pnp_relative_extrinsic(left_pnp, right_pnp):
        left_tag_rotation, _ = cv2.Rodrigues(left_pnp['rvec'])
        right_tag_rotation, _ = cv2.Rodrigues(right_pnp['rvec'])
        rotation = right_tag_rotation @ left_tag_rotation.T
        translation = right_pnp['tvec'] - rotation @ left_pnp['tvec']
        return rotation, translation

    def try_compare(self):
        left, right = self.latest['left'], self.latest['right']
        if left is None or right is None:
            return
        dt = abs(left['stamp'] - right['stamp'])
        if dt > self.max_dt:
            self.get_logger().warn(f'Both tags found but time difference too large: {dt * 1000:.1f} ms > {self.max_dt * 1000:.1f} ms', throttle_duration_sec=1.0)
            return
        pair_key = (left['stamp'], right['stamp'])
        if pair_key == self.last_pair_key:
            return
        self.last_pair_key = pair_key
        inverse_rotation = self.rotation.T
        inverse_translation = -inverse_rotation @ self.translation
        original = self.evaluate_extrinsic('original', self.rotation, self.translation, left['center'], right['center'])
        inverse = self.evaluate_extrinsic('inverse', inverse_rotation, inverse_translation, left['center'], right['center'])
        candidates = {'original': original, 'inverse': inverse}
        selected = min(candidates.values(), key=lambda item: item['total_error']) if self.extrinsic_mode == 'auto' else candidates[self.extrinsic_mode]
        xyz = selected['xyz']
        xyz_right = selected['xyz_right']
        active_rotation = selected['rotation']
        active_translation = selected['translation']
        left_error = selected['left_error']
        right_error = selected['right_error']
        left_pnp = left['pnp']['tvec']
        right_pnp = right['pnp']['tvec']
        right_pnp_in_left = active_rotation.T @ (right_pnp - active_translation)
        pnp_rotation, pnp_translation = self.pnp_relative_extrinsic(left['pnp'], right['pnp'])
        stereo_z = float(xyz[2, 0])
        left_pnp_z = float(left_pnp[2, 0])
        right_pnp_left_z = float(right_pnp_in_left[2, 0])
        result = {
            'tag_id': self.tag_id, 'time_difference_sec': dt, 'selected_extrinsic': selected['name'],
            'left_center_px': left['center'].tolist(), 'right_center_px': right['center'].tolist(),
            'stereo_xyz_left_m': xyz.reshape(3).tolist(), 'stereo_xyz_right_m': xyz_right.reshape(3).tolist(),
            'pnp_xyz_left_m': left_pnp.reshape(3).tolist(), 'right_pnp_xyz_in_left_m': right_pnp_in_left.reshape(3).tolist(),
            'depth_m': {'stereo_left_z': stereo_z, 'left_pnp_z': left_pnp_z, 'right_pnp_converted_left_z': right_pnp_left_z},
            'depth_difference_m': {'stereo_minus_left_pnp': stereo_z - left_pnp_z, 'stereo_minus_right_pnp': stereo_z - right_pnp_left_z, 'left_pnp_minus_right_pnp': left_pnp_z - right_pnp_left_z},
            'triangulation_reprojection_error_px': {'left': left_error, 'right': right_error},
            'extrinsic_candidates': {
                'original': {'left_error_px': original['left_error'], 'right_error_px': original['right_error'], 'stereo_z_m': float(original['xyz'][2, 0])},
                'inverse': {'left_error_px': inverse['left_error'], 'right_error_px': inverse['right_error'], 'stereo_z_m': float(inverse['xyz'][2, 0])}
            },
            'pnp_derived_left_to_right_extrinsic': {'rotation': pnp_rotation.reshape(9).tolist(), 'translation_m': pnp_translation.reshape(3).tolist()},
            'pnp_corner_reprojection_rmse_px': {'left': left['pnp']['reprojection_rmse_px'], 'right': right['pnp']['reprojection_rmse_px']}
        }
        msg = String(); msg.data = json.dumps(result, ensure_ascii=False); self.result_pub.publish(msg)
        self.get_logger().info(f'ID:{self.tag_id} dt={dt*1000:.1f}ms | selected={selected["name"]} | original reproj={original["left_error"]:.1f}/{original["right_error"]:.1f}px, inverse={inverse["left_error"]:.1f}/{inverse["right_error"]:.1f}px | stereo Z={stereo_z:.4f}m | PnP L={left_pnp_z:.4f}m, R->L={right_pnp_left_z:.4f}m | dZ stereo-L={(stereo_z-left_pnp_z)*1000:+.1f}mm | selected reproj={left_error:.2f}/{right_error:.2f}px')
        if max(left_error, right_error) > self.max_valid_reprojection_error:
            self.get_logger().error(f'EXTRINSIC INVALID: even selected={selected["name"]} has reprojection error L/R={left_error:.1f}/{right_error:.1f}px. Do not trust stereo depth.', throttle_duration_sec=1.0)
        if self.show_images:
            self.draw_result(left['frame'], 'LEFT', left['corners'], left['center'], stereo_z, left_pnp_z, dt)
            self.draw_result(right['frame'], 'RIGHT', right['corners'], right['center'], float(xyz_right[2, 0]), float(right_pnp[2, 0]), dt)
            self.display_frames['left'] = left['frame']
            self.display_frames['right'] = right['frame']
            self.show_combined_images()

    def draw_result(self, frame, side, corners, center, stereo_z, pnp_z, dt):
        pts = np.rint(corners).astype(np.int32)
        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        cv2.circle(frame, tuple(np.rint(center).astype(int)), 6, (0, 0, 255), -1)
        lines = [f'{side} Tag {self.tag_id}', f'center=({center[0]:.1f},{center[1]:.1f})', f'stereo Z={stereo_z:.4f} m', f'PnP Z={pnp_z:.4f} m', f'dt={dt*1000:.1f} ms']
        for i, line in enumerate(lines):
            text_width = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0][0]
            cv2.putText(frame, line, (frame.shape[1] - text_width - 25, 40 + i * 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagStereoDepthDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
