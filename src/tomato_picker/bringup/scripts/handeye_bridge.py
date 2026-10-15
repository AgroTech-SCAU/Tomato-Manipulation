#!/usr/bin/env python3
"""Hand-eye robot session with guarded recovery and ordered shutdown"""

import json
import math
import os
import signal
import threading
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from serial_arm import (
    JointImpedanceMode,
    RobotSession,
    RobotState,
    load_robot_profile_core,
)

CLEAR_FAULT_SERVICE = "/handeye/fault/clear"
COMPLIANT_RECOVERY_SERVICE = "/handeye/fault/compliant_recovery"
RIGID_HOLD_SERVICE = "/handeye/fault/rigid_hold"
ENABLE_DRAG_SERVICE = "/handeye/drag/enable"
RECOVER_DRAG_SERVICE = "/handeye/recover_drag"
STATUS_TOPIC = "/handeye/status"


def transform_matrix_to_pose(transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Convert a homogeneous transform into a position and xyzw quaternion"""
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("Invalid tool transform")
    rotation = matrix[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation[0, 1] + rotation[1, 0]) / scale
        qz = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / scale
        qx = (rotation[0, 1] + rotation[1, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale
    quat = np.asarray([qx, qy, qz, qw], dtype=float)
    norm = float(np.linalg.norm(quat))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid tool rotation")
    return matrix[:3, 3].copy(), quat / norm


def _optional_text(value: str) -> Optional[str]:
    return value.strip() or None


def _resource_path_list(value: str):
    return [s.strip() for s in value.split(os.pathsep) if s.strip()]


def _optional_baudrate(value: str) -> Optional[int]:
    value = value.strip()
    if not value:
        return None
    result = int(value)
    if result <= 0:
        raise ValueError("baudrate must be positive")
    return result


def _state_name(state):
    return getattr(state, "name", str(state).split(".")[-1])


def _infer_command_mode(snapshot, controller_config):
    """Infer the generated joint command mode from its gain vectors

    This is command feedback, not a hardware readback or the FAULT hold mode
    """
    if not snapshot.valid or snapshot.last_error:
        return "UNKNOWN"
    try:
        command = snapshot.cycle.joint_cmd
        kp = np.asarray(command.kp, dtype=float)
        kd = np.asarray(command.kd, dtype=float)
        if kp.ndim != 1 or kd.shape != kp.shape or not kp.size:
            return "UNKNOWN"
        if not np.all(np.isfinite(kp)) or not np.all(np.isfinite(kd)):
            return "UNKNOWN"
        candidates = (
            ("COMPLIANT_DRAG", "compliant_drag_gains"),
            ("RIGID_HOLD", "rigid_hold_gains"),
            ("COMPLIANT_HOLD", "compliant_hold_gains"),
            ("RIGID_TRACKING", "rigid_tracking_gains"),
            ("COMPLIANT_TRACKING", "compliant_tracking_gains"),
        )
        matches = []
        for label, attribute in candidates:
            gains = getattr(controller_config, attribute)
            expected_kp = np.asarray(gains.kp, dtype=float)
            expected_kd = np.asarray(gains.kd, dtype=float)
            if expected_kp.shape != kp.shape or expected_kd.shape != kd.shape:
                continue
            if np.allclose(kp, expected_kp, rtol=1e-4, atol=1e-5) and np.allclose(
                kd, expected_kd, rtol=1e-4, atol=1e-5
            ):
                matches.append(label)
        if len(matches) == 1:
            return matches[0]
        return "AMBIGUOUS" if matches else "UNKNOWN"
    except (AttributeError, TypeError, ValueError):
        return "UNKNOWN"


class HandeyeBridge(Node):
    """Exclusive hardware owner for hand-eye calibration"""

    def __init__(self):
        super().__init__("handeye_bridge")
        for key, default in (
            ("robot_profile", "tomato_picker"),
            ("profile_file", ""),
            ("resource_paths", ""),
            ("serial_port", ""),
            ("baudrate", ""),
            ("bus", ""),
            ("pose_topic", "/arm/pose"),
            ("transform_topic", "/arm/tool_transform"),
            ("publish_rate", 30.0),
            ("recovery_wait_s", 3.0),
            ("park_on_shutdown", True),
        ):
            self.declare_parameter(key, default)

        def param(name):
            return self.get_parameter(name).value

        profile_name = str(param("robot_profile"))
        profile_file = _optional_text(str(param("profile_file")))
        resource_paths = _resource_path_list(str(param("resource_paths")))
        pose_topic = str(param("pose_topic")).strip()
        transform_topic = str(param("transform_topic")).strip()
        rate = float(param("publish_rate"))
        self._recovery_wait_s = float(param("recovery_wait_s"))
        self._park_on_shutdown = bool(param("park_on_shutdown"))
        if not profile_name or not pose_topic or not transform_topic or pose_topic == transform_topic or not math.isfinite(rate) or rate <= 0:
            raise ValueError("Invalid robot profile, unique pose/transform topics or publish rate")
        if not math.isfinite(self._recovery_wait_s) or not (0.25 <= self._recovery_wait_s <= 15.0):
            raise ValueError("recovery_wait_s must be within 0.25 and 15 seconds")

        self._session_started = False
        self._shutdown_failed = False
        self._park_failed = False
        self._stopped = False
        self._shutting_down = False
        self._stop_lock = threading.Lock()
        self._last_robot_state = None
        self._last_error = None
        self._worker_stopped_logged = False
        self._drag_requested = False
        self._requested_mode = "RIGID_HOLD"
        self._session = None

        profile = load_robot_profile_core(profile_name, profile_file or "", resource_paths)
        self._session = RobotSession(
            profile.core_config_path,
            profile.hardware_plugin,
            profile.hardware_config_path,
            serial_port=_optional_text(str(param("serial_port"))),
            baudrate=_optional_baudrate(str(param("baudrate"))),
            bus=_optional_text(str(param("bus"))),
        )
        self._base_frame = self._session.config.dynamics.base_frame
        self._tool_frame = self._session.config.dynamics.tool_frame
        if not self._base_frame or not self._tool_frame:
            raise RuntimeError("Missing base or tool frame in Robot Profile")
        self._publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self._transform_publisher = self.create_publisher(TransformStamped, transform_topic, 10)
        self._status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self._services = [
            self.create_service(Trigger, CLEAR_FAULT_SERVICE, self._clear_fault),
            self.create_service(Trigger, COMPLIANT_RECOVERY_SERVICE, self._enter_fault_compliant_recovery),
            self.create_service(Trigger, RIGID_HOLD_SERVICE, self._return_to_fault_rigid_hold),
            self.create_service(Trigger, ENABLE_DRAG_SERVICE, self._enable_drag),
            self.create_service(Trigger, RECOVER_DRAG_SERVICE, self._recover_drag),
        ]

        try:
            self._session.start()
            self._session_started = True
            self._session.set_impedance_mode(JointImpedanceMode.COMPLIANT_DRAG)
            self._drag_requested = True
            self._requested_mode = "COMPLIANT_DRAG"
        except BaseException:
            self.stop_session()
            raise
        self._pose_timer = self.create_timer(1.0 / rate, self._publish_pose)
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f"Hand-eye ready | profile={profile_name} | {self._base_frame}->{self._tool_frame} | transform={transform_topic} | pose={pose_topic}"
        )
        self.get_logger().info(
            f"Recovery: ros2 service call {RECOVER_DRAG_SERVICE} std_srvs/srv/Trigger '{{}}'"
        )
        if self._park_on_shutdown:
            self.get_logger().warning("Shutdown parking enabled | Ctrl+C will move to configured park pose before disabling | verify safe zero and clear workspace")
        else:
            self.get_logger().info("Shutdown parking explicitly disabled | Ctrl+C will stop control and disable motors")

    def _respond(self, response, success, message):
        response.success = bool(success)
        response.message = str(message)
        return response

    def _ready(self):
        return not self._shutting_down and self._session_started and self._session.running

    def _log_state_transition(self, state, error):
        if state != self._last_robot_state:
            self._last_robot_state = state
            if state == RobotState.FAULT:
                self._drag_requested = False
                self._requested_mode = "RIGID_HOLD"
                self.get_logger().warning(
                    f"FAULT | {error or 'Inspect robot safety state'} | press r in handeye_operator to recover"
                )
            elif state == RobotState.ACTIVE:
                self.get_logger().info("ACTIVE | robot available")
            else:
                self.get_logger().info(f"Robot state = {_state_name(state)}")
        if error and error != self._last_error:
            self.get_logger().error(f"Robot worker: {error}")
        self._last_error = error

    def _publish_status(self):
        if self._shutting_down or self._session is None:
            return
        snapshot = self._session.snapshot
        state = self._session.state
        error = str(snapshot.last_error or "")
        self._log_state_transition(state, error)
        active = state == RobotState.ACTIVE and self._session.running and not error
        observed = _infer_command_mode(snapshot, self._session.config.ctrller) if active else "UNKNOWN"
        msg = String()
        msg.data = json.dumps({
            "state": _state_name(state),
            "running": bool(self._session.running),
            "snapshot_valid": bool(snapshot.valid),
            "pose_ready": bool(active and snapshot.valid),
            "error": error,
            "fault_hold_mode": _state_name(self._session.fault_hold_mode) if state == RobotState.FAULT else None,
            "requested_mode": self._requested_mode if active else "RIGID_HOLD",
            "observed_command_mode": observed,
            "drag_requested": bool(self._drag_requested and active),
        }, ensure_ascii=False)
        self._status_publisher.publish(msg)

    def _publish_pose(self):
        if self._shutting_down or not self._ready():
            return
        snapshot = self._session.snapshot
        if self._session.state != RobotState.ACTIVE or snapshot.last_error or not snapshot.valid:
            return
        try:
            position, quaternion = transform_matrix_to_pose(snapshot.dynamics.tool_pose)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        # Publish one kinematics snapshot with an identical timestamp on both interfaces
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._base_frame
        transform.child_frame_id = self._tool_frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, position)
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, quaternion)
        self._transform_publisher.publish(transform)

        # Keep the existing PoseStamped topic for applications not yet migrated
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, quaternion)
        self._publisher.publish(msg)

    def _clear_fault(self, _request, response):
        if not self._ready() or self._session.state != RobotState.FAULT:
            return self._respond(response, False, "A running FAULT session is required")
        try:
            self._session.clear_fault()
        except Exception as error:
            return self._respond(response, False, f"Core rejected fault clear: {error}")
        self._drag_requested = False
        self._requested_mode = "RIGID_HOLD"
        return self._respond(response, True, "FAULT cleared | wait for fresh ACTIVE snapshot before resuming drag")

    def _enter_fault_compliant_recovery(self, _request, response):
        if not self._ready() or self._session.state != RobotState.FAULT:
            return self._respond(response, False, "A running FAULT session is required")
        try:
            self._session.enter_fault_compliant_recovery()
        except Exception as error:
            return self._respond(response, False, f"Core rejected limited recovery: {error}")
        return self._respond(response, True, "Limited compliant recovery entered | move only to a safe pose, then request rigid hold")

    def _return_to_fault_rigid_hold(self, _request, response):
        if not self._ready() or self._session.state != RobotState.FAULT:
            return self._respond(response, False, "A running FAULT session is required")
        try:
            self._session.return_to_fault_rigid_hold()
        except Exception as error:
            return self._respond(response, False, f"Core rejected rigid hold: {error}")
        return self._respond(response, True, "FAULT rigid hold restored")

    def _request_drag(self, response):
        if not self._ready() or self._session.state != RobotState.ACTIVE:
            return self._respond(response, False, "ACTIVE and running RobotSession required")
        snapshot = self._session.snapshot
        if not snapshot.valid or snapshot.last_error:
            return self._respond(response, False, "A fresh valid ACTIVE snapshot is required")
        try:
            self._session.set_impedance_mode(JointImpedanceMode.COMPLIANT_DRAG)
        except Exception as error:
            return self._respond(response, False, f"Core rejected drag: {error}")
        self._drag_requested = True
        self._requested_mode = "COMPLIANT_DRAG"
        return self._respond(response, True, "COMPLIANT_DRAG requested | continue hand-eye sampling")

    def _enable_drag(self, _request, response):
        return self._request_drag(response)

    def _recover_drag(self, _request, response):
        """An explicit one-shot recovery request, never an automatic fault bypass"""
        if not self._ready():
            return self._respond(response, False, "Robot worker stopped | inspect hardware and restart")
        if self._session.state == RobotState.FAULT:
            if "COMPLIANT" in _state_name(self._session.fault_hold_mode):
                return self._respond(response, False, "Return to FAULT rigid hold before clearing")
            try:
                self._session.clear_fault()
            except Exception as error:
                return self._respond(
                    response, False,
                    f"Core rejected fault clear: {error} | inspect limits and motor state | use restricted recovery only when Core allows it",
                )
            self._drag_requested = False
            self._requested_mode = "RIGID_HOLD"
        elif self._session.state != RobotState.ACTIVE:
            return self._respond(response, False, "Robot must be ACTIVE or in a recoverable FAULT")

        deadline = time.monotonic() + self._recovery_wait_s
        while time.monotonic() < deadline and self._ready():
            snap = self._session.snapshot
            if self._session.state == RobotState.ACTIVE and snap.valid and not snap.last_error:
                return self._request_drag(response)
            if self._session.state == RobotState.FAULT:
                return self._respond(response, False, f"Safety fault remains | {snap.last_error}")
            time.sleep(0.04)
        return self._respond(response, False, "No fresh ACTIVE snapshot | drag remains disabled")

    def _park_if_requested(self):
        """Run the configured controlled shutdown move before stopping the Core worker"""
        if not self._park_on_shutdown:
            self.get_logger().warning("Shutdown park skipped by operator setting")
            return False
        if not self._session.running or self._session.state != RobotState.ACTIVE:
            self.get_logger().error(
                "Shutdown park unavailable | RobotSession is not ACTIVE | support arm before deactivation"
            )
            return False

        cfg = self._session.config.shutdown
        if not cfg.park_before_disable:
            self.get_logger().warning("Shutdown park disabled by Robot Profile")
            return False

        target = np.asarray(cfg.park_pos, dtype=float)
        dof = len(self._session.config.joint_names)
        if target.shape != (dof,) or not np.all(np.isfinite(target)):
            self.get_logger().error("Shutdown park target is invalid | cannot attempt homing")
            return False

        speed = float(cfg.speed_scale)
        timeout = float(cfg.timeout_s)
        position_tol = float(cfg.position_tolerance)
        velocity_tol = float(cfg.velocity_tolerance)
        settle_time = float(cfg.settle_time_s)
        if (not all(map(math.isfinite, (speed, timeout, position_tol, velocity_tol, settle_time)))
                or not 0.0 < speed <= 1.0 or timeout <= 0.0
                or position_tol <= 0.0 or velocity_tol <= 0.0 or settle_time < 0.0):
            self.get_logger().error("Shutdown park parameters are invalid")
            return False

        try:
            snap = self._session.snapshot
            if not snap.valid or snap.last_error:
                raise RuntimeError("Missing healthy robot snapshot")
            measured = np.asarray(snap.cycle.joint_state.pos, dtype=float)
            if measured.shape != target.shape or not np.all(np.isfinite(measured)):
                raise RuntimeError("Measured joint positions unavailable")

            self.get_logger().warning(
                "Ctrl+C | returning to configured shutdown joint zero | do not cut power during movement"
            )
            self._session.set_impedance_mode(JointImpedanceMode.COMPLIANT_TRACKING)
            self._requested_mode = "COMPLIANT_TRACKING"
            self._drag_requested = False
            # Mode requests and trajectory execution run in the independent C++ control worker
            self._session.move_to(target, speed)
            deadline = time.monotonic() + timeout
            settled_at = None
            while time.monotonic() < deadline:
                if not self._session.running or self._session.state != RobotState.ACTIVE:
                    raise RuntimeError("Robot left ACTIVE state during shutdown park")
                snap = self._session.snapshot
                if not snap.valid or snap.last_error:
                    raise RuntimeError("Core reported invalid feedback or safety fault")
                q = np.asarray(snap.cycle.joint_state.pos, dtype=float)
                dq = np.asarray(snap.cycle.joint_state.vel, dtype=float)
                if (q.shape != target.shape or dq.shape != target.shape or
                        not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq))):
                    raise RuntimeError("Invalid joint feedback during shutdown park")
                at_target = (
                    np.max(np.abs(q - target)) <= position_tol
                    and np.max(np.abs(dq)) <= velocity_tol
                )
                if at_target:
                    if settled_at is None:
                        settled_at = time.monotonic()
                    if time.monotonic() - settled_at >= settle_time:
                        self.get_logger().info("Shutdown park reached and settled | now disabling motors")
                        return True
                else:
                    settled_at = None
                time.sleep(0.04)
            raise TimeoutError(f"Shutdown park did not settle within {timeout:.1f} s")
        except Exception as error:
            # A FAULT cannot be cleared safely by an OS signal handler
            self.get_logger().error(
                f"Shutdown park incomplete | {error} | use external support before motor deactivation"
            )
            return False

    def stop_session(self):
        """Finish shutdown and motor disable before destroying ROS node"""
        with self._stop_lock:
            if self._stopped:
                return
            self._stopped = True
            self._shutting_down = True
        if not self._session_started or self._session is None:
            return
        self.get_logger().warning("Shutdown requested | parking then disabling | keep arm area clear")
        try:
            parked = self._park_if_requested()
            if self._park_on_shutdown and not parked:
                self._park_failed = True
                self.get_logger().error(
                    "Configured shutdown pose was not reached | physical support is required | stopping control under Core safety policy"
                )
        finally:
            try:
                # Core stops and joins the control worker, then stops and disables the motors
                self._session.stop()
                self.get_logger().info("Core worker stopped | motor deactivation completed | ROS shutdown follows")
            except Exception as error:
                self._shutdown_failed = True
                self.get_logger().error(f"Core shutdown failed | motor disable is unconfirmed | {error}")
            finally:
                self._session_started = False
                self._drag_requested = False

    def destroy_node(self):
        self.stop_session()
        result = super().destroy_node()
        self._session = None
        return result


def main():
    # Preserve process and ROS resources while the Core session performs synchronous stop
    stop_event = threading.Event()
    def ask_shutdown(_signum, _frame):
        stop_event.set()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous_sigint = signal.signal(signal.SIGINT, ask_shutdown)
    previous_sigterm = signal.signal(signal.SIGTERM, ask_shutdown)
    node = None
    exit_code = 0
    try:
        node = HandeyeBridge()
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        stop_event.set()
    except Exception as error:
        exit_code = 1
        if node is not None:
            node.get_logger().error(f"Hand-eye node failed | {error}")
        else:
            print(f"Hand-eye startup failed | {error}", flush=True)
    finally:
        # ROS stays alive until Core has stopped and motor deactivation has finished
        if node is not None:
            node.stop_session()
            if node._park_failed or node._shutdown_failed:
                exit_code = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
