#!/usr/bin/env python3
"""
FastSAM colour detector — ROS 2 (rclpy) port.
Subscribes to an RGB image topic, runs FastSAM segmentation, matches masks to
colour specs via HSV ratios, and publishes pixel centroids + binary masks.
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

# Allow importing segment_generation from the installed segmentation folder
_share_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', 'share', 'patrick_vision', 'segmentation'
)
sys.path.insert(0, _share_dir)

from segment_generation import FastSAMMaskExtractor  # noqa: E402


class ColorSpec:
    def __init__(self, name: str, threshold: float, hsv_ranges: list):
        self.name = name
        self.threshold = threshold
        self.hsv_ranges = [
            (np.array(r['lower']), np.array(r['upper']))
            for r in hsv_ranges
        ]


class FastSAMColorDetector(Node):
    def __init__(self):
        super().__init__('fastsam_color_detector')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/patrick/camera/rgb/image_raw')
        self.declare_parameter('fastsam_imgsz', 1024)
        self.declare_parameter('fastsam_conf', 0.6)
        self.declare_parameter('fastsam_iou', 0.9)
        self.declare_parameter('min_mask_area', 100)
        # Colors are loaded from the structured parameter list in YAML
        self.declare_parameter('colors', rclpy.Parameter.Type.STRING_ARRAY)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            model_path = os.path.join(_share_dir, 'models', 'FastSAM-x.pt')

        self.min_mask_area = self.get_parameter('min_mask_area').get_parameter_value().integer_value

        # Load color definitions from YAML parameters
        self.colors = self._load_colors()

        if not self.colors:
            self.get_logger().warn('No colors defined, using default red.')
            self.colors.append(ColorSpec(
                name='red_cylinder', threshold=0.3,
                hsv_ranges=[
                    {'lower': [0, 80, 80], 'upper': [15, 255, 255]},
                    {'lower': [160, 80, 80], 'upper': [180, 255, 255]},
                ],
            ))

        self.get_logger().info(f'Detecting: {[c.name for c in self.colors]}')
        self.get_logger().info(f'Loading FastSAM model: {model_path}')
        self.extractor = FastSAMMaskExtractor(model_path=model_path)
        self.bridge = CvBridge()

        # ── Publishers ──────────────────────────────────────────────────
        self.pixel_pubs = {}
        self.mask_pubs = {}
        for c in self.colors:
            self.pixel_pubs[c.name] = self.create_publisher(
                PointStamped, f'/detected_{c.name}_pixel', 1)
            self.mask_pubs[c.name] = self.create_publisher(
                Image, f'/detected_{c.name}_mask', 1)

        # ── Subscriber ──────────────────────────────────────────────────
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.create_subscription(Image, image_topic, self.image_callback, 1)

        self._last_log = 0.0
        self.get_logger().info('FastSAM Color Detector ready.')

    # ── Load structured color config ────────────────────────────────────
    def _load_colors(self) -> list:
        colors = []
        idx = 0
        while True:
            prefix = f'colors.{idx}'
            name_param = f'{prefix}.name'
            thresh_param = f'{prefix}.threshold'
            try:
                self.declare_parameter(name_param, '')
                self.declare_parameter(thresh_param, 0.3)
            except Exception:
                pass
            name = self.get_parameter(name_param).get_parameter_value().string_value
            if not name:
                break
            thresh = self.get_parameter(thresh_param).get_parameter_value().double_value

            # Load HSV ranges
            hsv_ranges = []
            ridx = 0
            while True:
                rprefix = f'{prefix}.hsv_ranges.{ridx}'
                lp = f'{rprefix}.lower'
                up = f'{rprefix}.upper'
                try:
                    self.declare_parameter(lp, [0, 0, 0])
                    self.declare_parameter(up, [0, 0, 0])
                except Exception:
                    pass
                lower = self.get_parameter(lp).get_parameter_value().integer_array_value
                upper = self.get_parameter(up).get_parameter_value().integer_array_value
                if not lower or lower == [0, 0, 0]:
                    break
                hsv_ranges.append({'lower': list(lower), 'upper': list(upper)})
                ridx += 1

            if hsv_ranges:
                colors.append(ColorSpec(name=name, threshold=thresh, hsv_ranges=hsv_ranges))
            idx += 1
        return colors

    # ── Colour ratio ────────────────────────────────────────────────────
    def color_ratio_in_mask(self, hsv, mask, spec: ColorSpec) -> float:
        total = cv2.countNonZero(mask)
        if total < self.min_mask_area:
            return 0.0
        combined = np.zeros(mask.shape, dtype=np.uint8)
        for lower, upper in spec.hsv_ranges:
            combined = cv2.bitwise_or(combined, cv2.inRange(hsv, lower, upper))
        matched = cv2.bitwise_and(combined, combined, mask=mask)
        return cv2.countNonZero(matched) / float(total)

    # ── Image callback ──────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        masks = self.extractor.extract_masks(cv_image)
        if masks.shape[0] == 0:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        for color in self.colors:
            best_ratio, best_idx = 0.0, -1
            for i in range(masks.shape[0]):
                ratio = self.color_ratio_in_mask(hsv, masks[i], color)
                if ratio > best_ratio:
                    best_ratio = ratio
                    best_idx = i

            if best_idx < 0 or best_ratio < color.threshold:
                continue

            mask = masks[best_idx]
            M = cv2.moments(mask)
            if M['m00'] == 0:
                continue

            u = M['m10'] / M['m00']
            v = M['m01'] / M['m00']

            # Publish pixel centroid
            pt = PointStamped()
            pt.header = msg.header
            pt.point.x = float(u)
            pt.point.y = float(v)
            self.pixel_pubs[color.name].publish(pt)

            # Publish binary mask
            mask_img = (mask * 255).astype(np.uint8)
            mask_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pubs[color.name].publish(mask_msg)

            now = time.monotonic()
            if now - self._last_log > 10.0:
                self._last_log = now
                self.get_logger().info(
                    f'[{color.name}] pixel=({u:.0f},{v:.0f}) ratio={best_ratio:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = FastSAMColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
