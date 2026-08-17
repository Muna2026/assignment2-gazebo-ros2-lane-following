#!/usr/bin/env python3
"""Simple, explainable lane detector for the Assignment 2 Prius world.

The detector assumes a forward-facing camera. It segments bright/white and
yellow lane markings in the lower region of the image, fits left and right
boundary lines, and publishes a normalized steering error:

    error > 0  -> the detected lane center is left of the image center,
                  so the controller should steer left.
    error < 0  -> the detected lane center is right of the image center,
                  so the controller should steer right.

The implementation is intentionally readable so every team member can explain
it during the demonstration. Thresholds are ROS parameters and should be
calibrated using the actual camera image from the provided world.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


LineModel = Tuple[float, float]  # y = slope*x + intercept


class LaneDetector(Node):
    def __init__(self) -> None:
        super().__init__('lane_detector')

        self.declare_parameter('image_topic', '/prius/front_camera/image_raw')
        self.declare_parameter('lane_error_topic', '/lane_error')
        self.declare_parameter('status_topic', '/lane_status')
        self.declare_parameter('debug_image_topic', '/lane_perception/debug_image')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('roi_top_ratio', 0.42)
        self.declare_parameter('white_value_min', 150)
        self.declare_parameter('white_saturation_max', 100)
        self.declare_parameter('yellow_h_min', 15)
        self.declare_parameter('yellow_h_max', 40)
        self.declare_parameter('yellow_saturation_min', 70)
        self.declare_parameter('yellow_value_min', 70)
        self.declare_parameter('min_line_length', 25.0)
        self.declare_parameter('min_abs_slope', 0.25)
        self.declare_parameter('center_deadband', 0.08)
        self.declare_parameter('assumed_lane_width_ratio', 0.48)

        image_topic = str(self.get_parameter('image_topic').value)
        error_topic = str(self.get_parameter('lane_error_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        debug_topic = str(self.get_parameter('debug_image_topic').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.error_pub = self.create_publisher(Float32MultiArray, error_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)

        self.get_logger().info(f'Subscribing to {image_topic}')
        self.get_logger().info(f'Publishing lane error to {error_topic}')
        if self.publish_debug:
            self.get_logger().info(f'Publishing debug images to {debug_topic}')

    @staticmethod
    def _weighted_line(lines: Sequence[Tuple[float, float, float]]) -> Optional[LineModel]:
        """Return a length-weighted median-ish line model from candidates."""
        if not lines:
            return None
        weights = np.array([max(item[2], 1.0) for item in lines], dtype=np.float64)
        slopes = np.array([item[0] for item in lines], dtype=np.float64)
        intercepts = np.array([item[1] for item in lines], dtype=np.float64)
        slope = float(np.average(slopes, weights=weights))
        intercept = float(np.average(intercepts, weights=weights))
        return slope, intercept

    def _mask_lane_markings(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(
            hsv,
            (0, 0, int(self.get_parameter('white_value_min').value)),
            (180, int(self.get_parameter('white_saturation_max').value), 255),
        )
        yellow = cv2.inRange(
            hsv,
            (int(self.get_parameter('yellow_h_min').value),
             int(self.get_parameter('yellow_saturation_min').value),
             int(self.get_parameter('yellow_value_min').value)),
            (int(self.get_parameter('yellow_h_max').value), 255, 255),
        )
        mask = cv2.bitwise_or(white, yellow)

        height = mask.shape[0]
        top = int(float(self.get_parameter('roi_top_ratio').value) * height)
        roi_mask = np.zeros_like(mask)
        roi_mask[top:, :] = mask[top:, :]
        kernel = np.ones((5, 5), dtype=np.uint8)
        roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_CLOSE, kernel)
        roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, kernel)
        return roi_mask

    def _fit_boundary_lines(
        self, mask: np.ndarray
    ) -> Tuple[Optional[LineModel], Optional[LineModel], np.ndarray]:
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=25,
            minLineLength=int(float(self.get_parameter('min_line_length').value)),
            maxLineGap=30,
        )

        height, width = mask.shape[:2]
        image_center = width / 2.0
        min_slope = float(self.get_parameter('min_abs_slope').value)
        left_candidates: list[Tuple[float, float, float]] = []
        right_candidates: list[Tuple[float, float, float]] = []
        line_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        if lines is not None:
            for raw in lines[:, 0, :]:
                x1, y1, x2, y2 = [float(v) for v in raw]
                dx = x2 - x1
                dy = y2 - y1
                length = math.hypot(dx, dy)
                if length < float(self.get_parameter('min_line_length').value) or abs(dx) < 1.0:
                    continue
                slope = dy / dx
                if abs(slope) < min_slope:
                    continue
                intercept = y1 - slope * x1
                bottom_x = (height - 1.0 - intercept) / slope

                # Reject lines whose extrapolated bottom point is far outside
                # the camera image. The side test separates left/right edges.
                if not (-0.25 * width <= bottom_x <= 1.25 * width):
                    continue
                if slope < 0.0 and bottom_x < image_center + 0.08 * width:
                    left_candidates.append((slope, intercept, length))
                    cv2.line(line_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                elif slope > 0.0 and bottom_x > image_center - 0.08 * width:
                    right_candidates.append((slope, intercept, length))
                    cv2.line(line_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

        return self._weighted_line(left_candidates), self._weighted_line(right_candidates), line_image

    @staticmethod
    def _x_at_y(line: Optional[LineModel], y: float) -> Optional[float]:
        if line is None or abs(line[0]) < 1e-6:
            return None
        return (y - line[1]) / line[0]

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warning(f'Could not convert image: {exc}')
            return

        if frame is None or frame.size == 0:
            return

        height, width = frame.shape[:2]
        mask = self._mask_lane_markings(frame)
        left_line, right_line, line_image = self._fit_boundary_lines(mask)

        y_bottom = float(height - 1)
        y_look = float(int(height * 0.72))
        left_bottom = self._x_at_y(left_line, y_bottom)
        right_bottom = self._x_at_y(right_line, y_bottom)
        left_look = self._x_at_y(left_line, y_look)
        right_look = self._x_at_y(right_line, y_look)

        both = left_look is not None and right_look is not None
        if both:
            lane_center = (left_look + right_look) / 2.0
            confidence = 1.0
        elif left_look is not None:
            assumed_width = float(self.get_parameter('assumed_lane_width_ratio').value) * width
            lane_center = left_look + assumed_width / 2.0
            confidence = 0.45
        elif right_look is not None:
            assumed_width = float(self.get_parameter('assumed_lane_width_ratio').value) * width
            lane_center = right_look - assumed_width / 2.0
            confidence = 0.45
        else:
            lane_center = width / 2.0
            confidence = 0.0

        # Positive error means the lane center is on the left side of the
        # image, therefore the controller should apply positive (left) yaw.
        error = (width / 2.0 - lane_center) / max(width / 2.0, 1.0)
        error = float(np.clip(error, -1.0, 1.0))

        deadband = float(self.get_parameter('center_deadband').value)
        if confidence == 0.0:
            status = 'LOST'
        elif abs(error) <= deadband:
            status = 'CENTERED'
        elif error > 0.0:
            status = 'DRIFT_LEFT'
        else:
            status = 'DRIFT_RIGHT'

        error_msg = Float32MultiArray()
        # [steering_error, confidence, left_detected, right_detected]
        error_msg.data = [
            error,
            float(confidence),
            1.0 if left_look is not None else 0.0,
            1.0 if right_look is not None else 0.0,
        ]
        self.error_pub.publish(error_msg)

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        if self.publish_debug:
            debug = frame.copy()
            top = int(float(self.get_parameter('roi_top_ratio').value) * height)
            cv2.rectangle(debug, (0, top), (width - 1, height - 1), (0, 255, 255), 1)
            debug = cv2.addWeighted(debug, 0.75, line_image, 0.45, 0.0)
            cv2.line(debug, (width // 2, 0), (width // 2, height - 1), (255, 255, 0), 2)
            cv2.circle(debug, (int(lane_center), int(y_look)), 7, (0, 255, 0), -1)
            cv2.putText(
                debug,
                f'{status}  err={error:+.3f}  conf={confidence:.2f}',
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except CvBridgeError as exc:
                self.get_logger().warning(f'Could not publish debug image: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
