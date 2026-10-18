#!/usr/bin/env python3
"""Publish a fresh base->end pose from robot_state_publisher for hand-eye servo.

Uses the *same* joint feedback and URDF as MoveIt. No RobotSession/hardware access.
No stale TF is republished as if it were a fresh measurement.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class HandeyeServoPose(Node):
    def __init__(self):
        super().__init__("handeye_servo_pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("pose_topic", "/arm/pose")
        self.declare_parameter("transform_topic", "/arm/tool_transform")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("max_tf_age_sec", 0.25)

        self.base = str(self.get_parameter("base_frame").value).strip()
        self.tool = str(self.get_parameter("tool_frame").value).strip()
        pose_topic = str(self.get_parameter("pose_topic").value).strip()
        transform_topic = str(self.get_parameter("transform_topic").value).strip()
        rate = float(self.get_parameter("publish_rate").value)
        self.max_age = float(self.get_parameter("max_tf_age_sec").value)
        if (not self.base or not self.tool or self.base == self.tool
                or not pose_topic or not transform_topic or pose_topic == transform_topic
                or not math.isfinite(rate) or rate <= 0.0
                or not math.isfinite(self.max_age) or self.max_age <= 0.0):
            raise ValueError("Invalid servo pose frames/topics/rate/max_tf_age_sec")
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.transform_pub = self.create_publisher(TransformStamped, transform_topic, 10)
        self._last_warning_ns = None
        self.create_timer(1.0 / rate, self.publish_fresh_pose)
        self.get_logger().info(
            f"Waiting for fresh {self.base} -> {self.tool} TF; pose={pose_topic} transform={transform_topic}"
        )

    def _warn_throttled(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if self._last_warning_ns is None or now_ns - self._last_warning_ns > int(3e9) or now_ns < self._last_warning_ns:
            self.get_logger().warn(message)
            self._last_warning_ns = now_ns

    def publish_fresh_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.base, self.tool, Time(), timeout=Duration(seconds=0.0))
        except TransformException as exc:
            self._warn_throttled(f"No robot TF {self.base} -> {self.tool}: {exc}")
            return

        stamp = Time.from_msg(transform.header.stamp)
        now = self.get_clock().now()
        age = (now - stamp).nanoseconds * 1e-9
        if not math.isfinite(age) or not (-0.05 <= age <= self.max_age):
            self._warn_throttled(f"Ignoring stale/future robot TF (age={age:.3f}s)")
            return

        xyz = transform.transform.translation
        if not all(math.isfinite(v) for v in (xyz.x, xyz.y, xyz.z)):
            self._warn_throttled("Ignoring invalid TF translation")
            return
        quat = transform.transform.rotation
        norm = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        if not math.isfinite(norm) or abs(norm - 1.0) > 0.01:
            self._warn_throttled("Ignoring invalid TF rotation")
            return

        pose = PoseStamped()
        pose.header = transform.header  # Preserve the joint-derived time, not the timer's time
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_pub.publish(pose)
        self.transform_pub.publish(transform)


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeServoPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
