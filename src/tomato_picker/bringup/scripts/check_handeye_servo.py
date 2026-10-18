#!/usr/bin/env python3
"""Check the hand-eye MoveGroup contract by PLANNING ONLY, never executing.

Usage: ros2 run tomato_picker_bringup check_handeye_servo [--offset-x 0.005]
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


def positive_number(name, value):
    f = float(value)
    if not math.isfinite(f):
        raise ValueError(f"{name} must be finite")
    return f


def plan_goal(base, tip, group, xyz, quaternion):
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = group
    req.allowed_planning_time = 8.0
    req.num_planning_attempts = 2
    req.max_velocity_scaling_factor = 0.05
    req.max_acceleration_scaling_factor = 0.05
    req.start_state.is_diff = True

    region = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.004]
    position = Pose()
    position.position.x, position.position.y, position.position.z = xyz
    position.orientation.w = 1.0
    region.primitives = [sphere]
    region.primitive_poses = [position]

    c = Constraints()
    p = PositionConstraint()
    p.header.frame_id = base
    p.link_name = tip
    p.constraint_region = region
    p.weight = 1.0
    c.position_constraints = [p]

    o = OrientationConstraint()
    o.header.frame_id = base
    o.link_name = tip
    o.orientation = quaternion
    o.absolute_x_axis_tolerance = 0.035
    o.absolute_y_axis_tolerance = 0.035
    o.absolute_z_axis_tolerance = 0.035
    o.weight = 1.0
    c.orientation_constraints = [o]
    req.goal_constraints = [c]

    # Do not set these in a way that can execute on the robot.
    goal.planning_options.plan_only = True
    goal.planning_options.replan = False
    goal.planning_options.look_around = False
    return goal


def await_future(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError("MoveGroup planning response timed out")
    return future.result()


def main():
    parser = argparse.ArgumentParser(description="Non-executing hand-eye servo planning check")
    parser.add_argument("--group", default="handeye_tip")
    parser.add_argument("--action", default="/move_action")
    parser.add_argument("--base", default="base_link")
    parser.add_argument("--tip", default="tip")
    parser.add_argument("--offset-x", type=float, default=0.0, help="plan-only target offset in metres")
    parser.add_argument("--offset-y", type=float, default=0.0)
    parser.add_argument("--offset-z", type=float, default=0.0)
    args = parser.parse_args()
    offsets = [positive_number(key, getattr(args, key)) for key in ("offset_x", "offset_y", "offset_z")]
    if math.sqrt(sum(a*a for a in offsets)) > .03:
        parser.error("dry-run target offset must be <= 30mm")

    rclpy.init()
    node = Node("handeye_servo_plan_only_check")
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    action = ActionClient(node, MoveGroup, args.action)
    handle = None
    try:
        if not action.wait_for_server(timeout_sec=8.0):
            raise RuntimeError(f"MoveGroup Action unavailable: {args.action}")

        # Allow TF listener to receive current joint-based transforms.
        deadline = node.get_clock().now().nanoseconds + 10_000_000_000
        transform = None
        while node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=.10)
            try:
                transform = buffer.lookup_transform(args.base, args.tip, Time(), timeout=Duration(seconds=0.0))
                age = (node.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds * 1e-9
                if -0.05 <= age <= .25:
                    break
            except TransformException:
                pass
            transform = None
        if transform is None:
            raise RuntimeError(f"Fresh TCP TF unavailable: {args.base} -> {args.tip}")
        t = transform.transform.translation
        xyz = [t.x + offsets[0], t.y + offsets[1], t.z + offsets[2]]
        goal = plan_goal(args.base, args.tip, args.group, xyz, transform.transform.rotation)
        if not goal.planning_options.plan_only:
            raise RuntimeError("Refusing to send a non-plan-only request")
        print(f"PLAN ONLY: group={args.group}, link={args.tip}, base={args.base}, target={xyz}")
        handle = await_future(node, action.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveGroup rejected the dry-run goal")
        response = await_future(node, handle.get_result_async(), 20.0)
        result = response.result
        if result.error_code.val != 1:
            raise RuntimeError(f"MoveGroup planning failed (error code {result.error_code.val})")
        points = len(result.planned_trajectory.joint_trajectory.points)
        print(f"PASS: TCP IK and planning succeeded; trajectory contains {points} points; no motion executed")
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        action.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
