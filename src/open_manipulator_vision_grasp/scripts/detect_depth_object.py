#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class DepthObjectDetector(Node):
    def __init__(self):
        super().__init__('detect_depth_object')
        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None

        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('output_frame', 'link1')
        self.declare_parameter('target_pose_topic', '/vision/target_pose')
        self.declare_parameter('marker_topic', '/vision/target_marker')
        self.declare_parameter('debug_image_topic', '/vision/debug_image')
        self.declare_parameter('min_depth_m', 0.12)
        self.declare_parameter('max_depth_m', 0.60)
        self.declare_parameter('closest_band_m', 0.04)
        self.declare_parameter('min_area_px', 400.0)
        self.declare_parameter('shape_filter_enabled', True)
        self.declare_parameter('min_aspect_ratio', 0.8)
        self.declare_parameter('max_aspect_ratio', 2.4)
        self.declare_parameter('min_extent', 0.45)
        self.declare_parameter('max_extent', 0.90)
        self.declare_parameter('min_solidity', 0.78)
        self.declare_parameter('max_solidity', 0.99)
        self.declare_parameter('min_bottom_to_top_width_ratio', 0.75)
        self.declare_parameter('max_bottom_to_top_width_ratio', 1.80)
        self.declare_parameter('min_width_profile_samples', 12)
        self.declare_parameter('sample_radius_px', 5)
        self.declare_parameter('roi_x', 0)
        self.declare_parameter('roi_y', 0)
        self.declare_parameter('roi_width', -1)
        self.declare_parameter('roi_height', -1)
        self.declare_parameter('target_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])

        self.output_frame = self.get_parameter('output_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter('target_pose_topic').value, 10)
        self.marker_pub = self.create_publisher(Marker, self.get_parameter('marker_topic').value, 10)
        self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_image_topic').value, 10)

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self._on_depth,
            qos_profile_sensor_data,
        )

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_depth(self, msg: Image):
        if self.camera_info is None:
            return

        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_m = depth_raw.astype(np.float32)
        if depth_raw.dtype == np.uint16:
            depth_m *= 0.001

        roi, offset = self._crop_roi(depth_m)
        valid = np.isfinite(roi)
        valid &= roi >= float(self.get_parameter('min_depth_m').value)
        valid &= roi <= float(self.get_parameter('max_depth_m').value)
        if not np.any(valid):
            self._publish_debug(depth_m)
            return

        closest = float(np.percentile(roi[valid], 2.0))
        band = float(self.get_parameter('closest_band_m').value)
        mask = (valid & (roi <= closest + band)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour, metrics = self._select_target_contour(contours)
        if contour is None:
            self._publish_debug(depth_m)
            return
        if metrics:
            self.get_logger().debug(
                f"cup candidate: area={metrics['area']:.0f} "
                f"aspect={metrics['aspect_ratio']:.2f} "
                f"extent={metrics['extent']:.2f} "
                f"solidity={metrics['solidity']:.2f} "
                f"bottom/top={metrics['bottom_to_top_width_ratio']:.2f}"
            )
        moments = cv2.moments(contour)
        if abs(moments['m00']) < 1e-6:
            return

        u_roi = int(moments['m10'] / moments['m00'])
        v_roi = int(moments['m01'] / moments['m00'])
        u = u_roi + offset[0]
        v = v_roi + offset[1]
        z = self._sample_depth_m(depth_m, u, v)
        if z is None:
            self._publish_debug(depth_m, contour, offset, u, v)
            return

        point = self._project_pixel_to_point(u, v, z, self.camera_info)
        if point is None:
            return
        point.header = msg.header if msg.header.frame_id else self.camera_info.header

        pose = PoseStamped()
        pose.header = point.header
        pose.pose.position.x = point.point.x
        pose.pose.position.y = point.point.y
        pose.pose.position.z = point.point.z
        q = self.get_parameter('target_orientation_xyzw').value
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])

        if self.output_frame:
            try:
                point.header.stamp.sec = 0
                point.header.stamp.nanosec = 0
                point = self.tf_buffer.transform(point, self.output_frame, timeout=rclpy.duration.Duration(seconds=0.05))
                pose.header = point.header
                pose.pose.position.x = point.point.x
                pose.pose.position.y = point.point.y
                pose.pose.position.z = point.point.z
            except TransformException as exc:
                self.get_logger().warn(f'TF transform failed: {exc}', throttle_duration_sec=2.0)
                return

        self.pose_pub.publish(pose)
        self._publish_marker(pose)
        self._publish_debug(depth_m, contour, offset, u, v, z)

    def _select_target_contour(self, contours):
        min_area = float(self.get_parameter('min_area_px').value)
        candidates = []
        for contour in contours:
            metrics = self._contour_metrics(contour)
            if metrics is None or metrics['area'] < min_area:
                continue
            if self.get_parameter('shape_filter_enabled').value and not self._passes_cup_shape(metrics):
                continue
            score = metrics['area'] * metrics['solidity'] * max(metrics['extent'], 0.01)
            candidates.append((score, contour, metrics))
        if not candidates:
            return None, None
        _, contour, metrics = max(candidates, key=lambda item: item[0])
        return contour, metrics

    def _contour_metrics(self, contour):
        area = float(cv2.contourArea(contour))
        x, y, width, height = cv2.boundingRect(contour)
        if width <= 0 or height <= 0:
            return None
        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        if hull_area <= 0.0:
            return None
        aspect_ratio = float(height) / float(width)
        extent = area / float(width * height)
        solidity = area / hull_area
        top_width = self._profile_width(contour, x, y, width, height, 0.25)
        mid_width = self._profile_width(contour, x, y, width, height, 0.50)
        bottom_width = self._profile_width(contour, x, y, width, height, 0.75)
        bottom_to_top = bottom_width / max(top_width, 1.0)
        return {
            'area': area,
            'x': x,
            'y': y,
            'width': float(width),
            'height': float(height),
            'aspect_ratio': aspect_ratio,
            'extent': extent,
            'solidity': solidity,
            'top_width': top_width,
            'mid_width': mid_width,
            'bottom_width': bottom_width,
            'bottom_to_top_width_ratio': bottom_to_top,
        }

    def _profile_width(self, contour, x, y, width, height, y_ratio):
        samples = int(self.get_parameter('min_width_profile_samples').value)
        band = max(3, height // max(samples, 1))
        row = int(y + height * y_ratio)
        mask = np.zeros((height, width), dtype=np.uint8)
        shifted = contour - np.array([[[x, y]]], dtype=np.int32)
        cv2.drawContours(mask, [shifted], -1, 255, thickness=cv2.FILLED)
        local_y0 = max(0, row - y - band)
        local_y1 = min(height, row - y + band + 1)
        cols = np.where(mask[local_y0:local_y1, :] > 0)[1]
        if cols.size == 0:
            return 0.0
        return float(cols.max() - cols.min() + 1)

    def _passes_cup_shape(self, metrics) -> bool:
        return (
            float(self.get_parameter('min_aspect_ratio').value) <= metrics['aspect_ratio'] <=
            float(self.get_parameter('max_aspect_ratio').value) and
            float(self.get_parameter('min_extent').value) <= metrics['extent'] <=
            float(self.get_parameter('max_extent').value) and
            float(self.get_parameter('min_solidity').value) <= metrics['solidity'] <=
            float(self.get_parameter('max_solidity').value) and
            float(self.get_parameter('min_bottom_to_top_width_ratio').value) <=
            metrics['bottom_to_top_width_ratio'] <=
            float(self.get_parameter('max_bottom_to_top_width_ratio').value)
        )

    def _crop_roi(self, depth_m: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        x = int(self.get_parameter('roi_x').value)
        y = int(self.get_parameter('roi_y').value)
        width = int(self.get_parameter('roi_width').value)
        height = int(self.get_parameter('roi_height').value)
        if width <= 0:
            width = depth_m.shape[1] - x
        if height <= 0:
            height = depth_m.shape[0] - y
        x = max(0, min(x, depth_m.shape[1] - 1))
        y = max(0, min(y, depth_m.shape[0] - 1))
        x2 = max(x + 1, min(x + width, depth_m.shape[1]))
        y2 = max(y + 1, min(y + height, depth_m.shape[0]))
        return depth_m[y:y2, x:x2], (x, y)

    def _sample_depth_m(self, depth_m: np.ndarray, u: int, v: int) -> Optional[float]:
        radius = int(self.get_parameter('sample_radius_px').value)
        y0 = max(0, v - radius)
        y1 = min(depth_m.shape[0], v + radius + 1)
        x0 = max(0, u - radius)
        x1 = min(depth_m.shape[1], u + radius + 1)
        sample = depth_m[y0:y1, x0:x1]
        valid = sample[np.isfinite(sample)]
        min_depth = float(self.get_parameter('min_depth_m').value)
        max_depth = float(self.get_parameter('max_depth_m').value)
        valid = valid[(valid >= min_depth) & (valid <= max_depth)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _project_pixel_to_point(self, u: int, v: int, z: float, info: CameraInfo) -> Optional[PointStamped]:
        fx = info.k[0]
        fy = info.k[4]
        cx = info.k[2]
        cy = info.k[5]
        if fx == 0.0 or fy == 0.0 or not math.isfinite(z):
            return None
        point = PointStamped()
        point.header = info.header
        point.point.x = (float(u) - cx) * z / fx
        point.point.y = (float(v) - cy) * z / fy
        point.point.z = z
        return point

    def _publish_marker(self, pose: PoseStamped):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'depth_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.1
        marker.color.a = 0.9
        self.marker_pub.publish(marker)

    def _publish_debug(self, depth_m, contour=None, offset=(0, 0), u=None, v=None, z=None):
        finite = np.isfinite(depth_m)
        display = np.zeros(depth_m.shape, dtype=np.uint8)
        if np.any(finite):
            clipped = np.clip(depth_m, 0.0, float(self.get_parameter('max_depth_m').value))
            display = cv2.convertScaleAbs(clipped, alpha=255.0 / max(float(self.get_parameter('max_depth_m').value), 0.001))
        debug = cv2.applyColorMap(display, cv2.COLORMAP_TURBO)
        if contour is not None:
            shifted = contour + np.array(offset, dtype=np.int32)
            cv2.drawContours(debug, [shifted], -1, (0, 255, 0), 2)
        if u is not None and v is not None:
            cv2.circle(debug, (u, v), 5, (255, 255, 255), -1)
            if z is not None:
                cv2.putText(debug, f'{z:.3f} m', (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))


def main():
    rclpy.init()
    node = DepthObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
