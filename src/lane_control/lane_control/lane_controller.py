#!/usr/bin/env python3
"""Closed-loop lane-following controller.

Input `/lane_error` is a Float32MultiArray produced by lane_perception:
    data[0] = normalized steering error in [-1, 1]
    data[1] = confidence in [0, 1]
    data[2] = left boundary detected (0 or 1)
    data[3] = right boundary detected (0 or 1)

The controller publishes geometry_msgs/Twist on `/cmd_vel`. The steering sign
parameter makes it easy to correct the convention if the provided car world
turns in the opposite direction from the default.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class LaneController(Node):
    def __init__(self) -> None:
        super().__init__('lane_controller')

        self.declare_parameter('lane_error_topic', '/lane_error')
        self.declare_parameter('status_topic', '/lane_status')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate_hz', 25.0)
        self.declare_parameter('kp', 1.55)
        self.declare_parameter('ki', 0.02)
        self.declare_parameter('kd', 0.18)
        self.declare_parameter('base_speed', 1.25)
        self.declare_parameter('min_speed', 0.25)
        self.declare_parameter('max_speed', 2.25)
        self.declare_parameter('max_angular_z', 1.60)
        self.declare_parameter('speed_reduction', 0.70)
        self.declare_parameter('min_confidence', 0.30)
        self.declare_parameter('lost_timeout_sec', 0.35)
        self.declare_parameter('integral_limit', 0.80)
        self.declare_parameter('steering_sign', 1.0)

        error_topic = str(self.get_parameter('lane_error_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        rate = max(float(self.get_parameter('control_rate_hz').value), 1.0)

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.error_sub = self.create_subscription(
            Float32MultiArray, error_topic, self.error_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, status_topic, self.status_callback, 10
        )
        self.timer = self.create_timer(1.0 / rate, self.control_callback)

        self.last_error = 0.0
        self.last_confidence = 0.0
        self.last_error_time: Optional[int] = None
        self.last_update_time: Optional[int] = None
        self.error_derivative = 0.0
        self.error_integral = 0.0
        self.last_status = 'WAITING'
        self.last_log_time = self.get_clock().now()

        self.get_logger().info(f'Subscribing to {error_topic}; publishing {cmd_topic}')
        self.get_logger().info(
            'Start condition: a valid lane message is required; the car stops '
            'if perception is lost.'
        )

    def status_callback(self, msg: String) -> None:
        self.last_status = msg.data

    def error_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warning('lane_error must contain at least [error, confidence]')
            return

        now_ns = self.get_clock().now().nanoseconds
        error = float(msg.data[0])
        confidence = float(msg.data[1])
        if not (math.isfinite(error) and math.isfinite(confidence)):
            return

        error = max(-1.0, min(1.0, error))
        confidence = max(0.0, min(1.0, confidence))
        if self.last_update_time is not None:
            dt = (now_ns - self.last_update_time) / 1e9
            if 0.001 < dt < 0.5:
                self.error_derivative = (error - self.last_error) / dt
                self.error_integral += error * dt
        self.last_update_time = now_ns
        self.last_error_time = now_ns
        self.last_error = error
        self.last_confidence = confidence

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def _publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def control_callback(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        timeout = float(self.get_parameter('lost_timeout_sec').value)
        min_conf = float(self.get_parameter('min_confidence').value)

        if self.last_error_time is None:
            self._publish_stop()
            return

        age = (now_ns - self.last_error_time) / 1e9
        if age > timeout or self.last_confidence <= 0.0:
            self._publish_stop()
            self._log_throttled(
                f'Lane lost or stale ({age:.2f}s, confidence={self.last_confidence:.2f}); stopping.'
            )
            return

        # A single boundary is accepted at reduced confidence, but the car is
        # slowed down rather than blindly driving at race speed.
        confidence_scale = max(0.35, min(1.0, self.last_confidence))
        if self.last_confidence < min_conf:
            confidence_scale *= 0.45

        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        integral_limit = abs(float(self.get_parameter('integral_limit').value))
        self.error_integral = max(-integral_limit, min(integral_limit, self.error_integral))

        steering_sign = float(self.get_parameter('steering_sign').value)
        angular = steering_sign * (
            kp * self.last_error + ki * self.error_integral + kd * self.error_derivative
        )
        max_angular = abs(float(self.get_parameter('max_angular_z').value))
        angular = max(-max_angular, min(max_angular, angular))

        base_speed = float(self.get_parameter('base_speed').value)
        min_speed = max(0.0, float(self.get_parameter('min_speed').value))
        max_speed = max(min_speed, float(self.get_parameter('max_speed').value))
        reduction = max(0.0, float(self.get_parameter('speed_reduction').value))
        speed = base_speed * (1.0 - reduction * min(abs(self.last_error), 1.0))
        speed *= confidence_scale
        speed = max(min_speed, min(max_speed, speed))

        # For a very large error, slowing to the minimum is safer than
        # accelerating through a bend. The parameters can be tuned for speed
        # only after the car completes stable laps.
        if abs(self.last_error) > 0.85:
            speed = min_speed

        self._publish_cmd(speed, angular)
        self._log_throttled(
            f'{self.last_status}: error={self.last_error:+.3f}, '
            f'confidence={self.last_confidence:.2f}, '
            f'cmd=({speed:.2f} m/s, {angular:+.2f} rad/s)'
        )

    def _log_throttled(self, text: str) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds >= 1_000_000_000:
            self.get_logger().info(text)
            self.last_log_time = now

    def destroy_node(self):  # type: ignore[override]
        # Send a final zero command before shutting down.
        self._publish_stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
