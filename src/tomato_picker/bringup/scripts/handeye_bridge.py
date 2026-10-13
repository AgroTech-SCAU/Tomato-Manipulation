#!/usr/bin/env python3

import math
import os
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
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
RESUME_DRAG_SERVICE = "/handeye/drag/resume"
RESUME_DRAG_TIMEOUT_S = 5.0


def transform_matrix_to_pose(transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Convert a 4x4 homogeneous transform to position and xyzw quaternion."""
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"tool_pose must have shape (4, 4), got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("tool_pose contains NaN or Inf")

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
        qy = 0.25 * scale
        qz = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale

    quaternion = np.asarray([qx, qy, qz, qw], dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("tool_pose rotation produced an invalid quaternion")
    quaternion /= norm

    return matrix[:3, 3].copy(), quaternion


def _optional_text(value: str) -> Optional[str]:
    value = value.strip()
    return value if value else None




def _resource_path_list(value: str):
    """Parse an os.pathsep-separated resource path string for SerialArm-Core v0.5.2."""
    return [item.strip() for item in value.split(os.pathsep) if item.strip()]

def _is_cycle_timeout_fault(message: str) -> bool:
    """Accept only the specific recoverable control-period fault from the active Core contract."""
    return "RobotErr=20;" in message and "SafetyErr=2;" in message


def _optional_baudrate(value: str) -> Optional[int]:
    value = value.strip()
    if not value:
        return None
    baudrate = int(value)
    if baudrate <= 0:
        raise ValueError("baudrate must be a positive integer")
    return baudrate


class HandeyeBridge(Node):
    """Own the SerialArm session and publish safe hand-eye calibration pose data."""

    def __init__(self) -> None:
        super().__init__("handeye_bridge")

        self.declare_parameter("robot_profile", "tomato_picker")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("resource_paths", "")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", "")
        self.declare_parameter("bus", "")
        self.declare_parameter("pose_topic", "/arm/pose")
        self.declare_parameter("publish_rate", 30.0)

        robot_profile = str(self.get_parameter("robot_profile").value)
        profile_file = _optional_text(str(self.get_parameter("profile_file").value))
        resource_paths = _resource_path_list(str(self.get_parameter("resource_paths").value))
        serial_port = _optional_text(str(self.get_parameter("serial_port").value))
        baudrate = _optional_baudrate(str(self.get_parameter("baudrate").value))
        bus = _optional_text(str(self.get_parameter("bus").value))
        pose_topic = str(self.get_parameter("pose_topic").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        if not robot_profile:
            raise ValueError("robot_profile must not be empty")
        if not pose_topic:
            raise ValueError("pose_topic must not be empty")
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("publish_rate must be > 0")

        profile = load_robot_profile_core(robot_profile, profile_file or "", resource_paths)
        session = RobotSession(
            profile.core_config_path,
            profile.hardware_plugin,
            profile.hardware_config_path,
            serial_port=serial_port,
            baudrate=baudrate,
            bus=bus,
        )
        self._session = session
        self._session_started = False
        self._stopped = False
        self._waiting_for_snapshot_logged = False
        self._snapshot_error_logged = ""
        self._worker_stopped_logged = False
        self._last_robot_state = None
        self._resume_drag_pending = False
        self._resume_drag_started_at = 0.0

        self._base_frame = session.config.dynamics.base_frame
        self._tool_frame = session.config.dynamics.tool_frame
        if not self._base_frame or not self._tool_frame:
            raise RuntimeError(
                "SerialArm dynamics base_frame/tool_frame must not be empty"
            )

        self._publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self._clear_fault_service = self.create_service(
            Trigger, CLEAR_FAULT_SERVICE, self._clear_fault
        )
        self._compliant_recovery_service = self.create_service(
            Trigger, COMPLIANT_RECOVERY_SERVICE, self._enter_fault_compliant_recovery
        )
        self._rigid_hold_service = self.create_service(
            Trigger, RIGID_HOLD_SERVICE, self._return_to_fault_rigid_hold
        )
        self._enable_drag_service = self.create_service(
            Trigger, ENABLE_DRAG_SERVICE, self._enable_drag
        )
        self._resume_drag_service = self.create_service(
            Trigger, RESUME_DRAG_SERVICE, self._resume_drag
        )

        try:
            session.start()
            self._session_started = True
            session.set_impedance_mode(JointImpedanceMode.COMPLIANT_DRAG)
        except Exception:
            self.stop_session()
            raise

        self._timer = self.create_timer(1.0 / publish_rate, self._publish_pose)
        self.get_logger().info(
            f"Handeye bridge ready: profile={robot_profile}, profile_file={profile_file or 'auto'}, resource_paths={len(resource_paths)}, mode=COMPLIANT_DRAG, "
            f"pose={pose_topic}, frame={self._base_frame}->{self._tool_frame}, rate={publish_rate:.1f} Hz"
        )
        self.get_logger().info(
            "Recovery services: "
            f"clear={CLEAR_FAULT_SERVICE}, "
            f"compliant_recovery={COMPLIANT_RECOVERY_SERVICE}, "
            f"rigid_hold={RIGID_HOLD_SERVICE}, "
            f"enable_drag={ENABLE_DRAG_SERVICE}, "
            f"resume_drag={RESUME_DRAG_SERVICE}"
        )

    def _log_state_transition(self, state) -> None:
        if state == self._last_robot_state:
            return

        previous = self._last_robot_state
        self._last_robot_state = state

        if state == RobotState.FAULT:
            self.get_logger().warning(
                "SerialArm entered FAULT. /arm/pose is paused while Core maintains the "
                "configured fault hold. For an ordinary recoverable fault, wait for the "
                f"hold to stabilize and call {CLEAR_FAULT_SERVICE}."
            )
            return

        if state == RobotState.ACTIVE and previous == RobotState.FAULT:
            self.get_logger().info(
                "SerialArm FAULT cleared: robot is ACTIVE + RIGID_HOLD. "
                "Wait for a fresh valid /arm/pose sample, then explicitly call "
                f"{ENABLE_DRAG_SERVICE} to resume COMPLIANT_DRAG."
            )
            return

        self.get_logger().info(f"SerialArm state changed to {state}")

    def _publish_pose(self) -> None:
        state = self._session.state
        snapshot = self._session.snapshot
        self._log_state_transition(state)
        self._complete_pending_drag_resume(state, snapshot)

        if not self._session.running:
            if not self._worker_stopped_logged:
                self.get_logger().error(
                    "SerialArm worker is not running; online recovery is unavailable. "
                    "Stop calibration and inspect the hardware/Core error before restarting."
                )
                self._worker_stopped_logged = True
            return
        self._worker_stopped_logged = False

        if snapshot.last_error:
            error_text = str(snapshot.last_error)
            if error_text != self._snapshot_error_logged:
                self.get_logger().error(f"SerialArm worker error: {error_text}")
                self._snapshot_error_logged = error_text
        else:
            self._snapshot_error_logged = ""

        # Never publish stale pre-FAULT data. v0.5.2 keeps the v0.5.1 snapshot invalidation semantics on
        # clear_fault(), and the first new ACTIVE cycle makes it valid again.
        if state != RobotState.ACTIVE:
            return
        if snapshot.last_error:
            return
        if not snapshot.valid:
            if not self._waiting_for_snapshot_logged:
                self.get_logger().warning(
                    "Waiting for a fresh valid SerialArm snapshot before publishing /arm/pose"
                )
                self._waiting_for_snapshot_logged = True
            return
        self._waiting_for_snapshot_logged = False

        try:
            position, quaternion = transform_matrix_to_pose(snapshot.dynamics.tool_pose)
        except ValueError as error:
            self.get_logger().error(f"Invalid SerialArm tool pose: {error}")
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        self._publisher.publish(message)

    def _complete_pending_drag_resume(self, state, snapshot) -> None:
        """Finish a single operator-authorized recovery after a new valid control cycle."""
        if not self._resume_drag_pending:
            return

        if not self._session.running or state != RobotState.ACTIVE or snapshot.last_error:
            self._resume_drag_pending = False
            self.get_logger().error(
                "Drag resume cancelled because the session stopped, faulted again or reported an error"
            )
            return

        if time.monotonic() - self._resume_drag_started_at > RESUME_DRAG_TIMEOUT_S:
            self._resume_drag_pending = False
            self.get_logger().error(
                "Drag resume timed out waiting for a fresh ACTIVE snapshot; inspect the robot"
            )
            return

        if not snapshot.valid:
            return

        self._resume_drag_pending = False
        try:
            self._session.set_impedance_mode(JointImpedanceMode.COMPLIANT_DRAG)
        except Exception as error:
            self.get_logger().error(f"Operator-requested drag resume failed: {error}")
            return

        self.get_logger().info(
            "Fresh ACTIVE snapshot received; operator-requested COMPLIANT_DRAG submitted"
        )

    def _resume_drag(self, _request, response):
        """One explicit request to recover a transient cycle fault and resume drag."""
        if not self._session.running:
            response.success = False
            response.message = "RobotSession worker is not running"
            return response
        if self._resume_drag_pending:
            response.success = False
            response.message = "Drag resume is already pending"
            return response

        state = self._session.state
        snapshot = self._session.snapshot

        if state == RobotState.ACTIVE:
            if not snapshot.valid or snapshot.last_error:
                response.success = False
                response.message = "A fresh, error-free ACTIVE snapshot is required"
                return response
            try:
                self._session.set_impedance_mode(JointImpedanceMode.COMPLIANT_DRAG)
            except Exception as error:
                response.success = False
                response.message = f"COMPLIANT_DRAG rejected: {error}"
                return response
            response.success = True
            response.message = "COMPLIANT_DRAG request submitted"
            return response

        if state != RobotState.FAULT:
            response.success = False
            response.message = "Robot must be ACTIVE or in a recoverable FAULT"
            return response

        if not _is_cycle_timeout_fault(str(snapshot.last_error)):
            response.success = False
            response.message = (
                "Only the known control-cycle INVALID_DT fault supports one-step drag resume; "
                "inspect other faults and use the explicit recovery services"
            )
            return response

        try:
            # Core checks live actuator state, velocity and fault-hold stability.
            # Never soften fault hold while the robot is still in FAULT.
            self._session.clear_fault()
        except Exception as error:
            response.success = False
            response.message = f"Safety-controlled fault clear was rejected: {error}"
            return response

        self._resume_drag_pending = True
        self._resume_drag_started_at = time.monotonic()
        response.success = True
        response.message = (
            "Fault cleared by Core; waiting for a fresh ACTIVE snapshot before "
            "submitting COMPLIANT_DRAG. Another fault cancels this request"
        )
        return response

    def _clear_fault(self, _request, response):
        """Clear a recoverable FAULT without automatically re-entering drag mode."""
        if not self._session.running:
            response.success = False
            response.message = "RobotSession worker is not running; online FAULT recovery is unavailable"
            return response
        if self._session.state != RobotState.FAULT:
            response.success = False
            response.message = "Robot is not in FAULT"
            return response

        self._resume_drag_pending = False
        try:
            self._session.clear_fault()
        except Exception as error:
            response.success = False
            response.message = (
                f"clear_fault rejected: {error}. Fault hold remains active; "
                "retry only after the safety hold has stabilized if the fault is recoverable"
            )
            return response

        response.success = True
        response.message = (
            "FAULT cleared. Robot is ACTIVE + RIGID_HOLD. Wait for a fresh valid pose, "
            f"then call {ENABLE_DRAG_SERVICE} to resume COMPLIANT_DRAG"
        )
        return response

    def _enter_fault_compliant_recovery(self, _request, response):
        """Request Core's restricted FAULT compliant recovery mode."""
        if not self._session.running:
            response.success = False
            response.message = "RobotSession worker is not running"
            return response
        if self._session.state != RobotState.FAULT:
            response.success = False
            response.message = "Robot is not in FAULT"
            return response

        try:
            self._session.enter_fault_compliant_recovery()
        except Exception as error:
            response.success = False
            response.message = (
                f"Compliant FAULT recovery rejected by SerialArm-Core: {error}"
            )
            return response

        response.success = True
        response.message = (
            "FAULT compliant recovery enabled. Use it only to move out of an allowed unsafe "
            f"configuration, then call {RIGID_HOLD_SERVICE} before clearing the fault"
        )
        return response

    def _return_to_fault_rigid_hold(self, _request, response):
        """Return an in-FAULT compliant recovery session to rigid fault hold."""
        if not self._session.running:
            response.success = False
            response.message = "RobotSession worker is not running"
            return response
        if self._session.state != RobotState.FAULT:
            response.success = False
            response.message = "Robot is not in FAULT"
            return response

        try:
            self._session.return_to_fault_rigid_hold()
        except Exception as error:
            response.success = False
            response.message = f"Return to FAULT rigid hold failed: {error}"
            return response

        response.success = True
        response.message = "Robot returned to FAULT + RIGID_HOLD"
        return response

    def _enable_drag(self, _request, response):
        """Explicitly re-enter compliant drag after a healthy ACTIVE cycle."""
        if not self._session.running:
            response.success = False
            response.message = "RobotSession worker is not running"
            return response
        if self._session.state != RobotState.ACTIVE:
            response.success = False
            response.message = (
                "Robot must be ACTIVE before COMPLIANT_DRAG can be enabled"
            )
            return response

        snapshot = self._session.snapshot
        if snapshot.last_error or not snapshot.valid:
            response.success = False
            response.message = (
                "Wait for a fresh valid ACTIVE snapshot before enabling COMPLIANT_DRAG"
            )
            return response

        self._resume_drag_pending = False
        try:
            self._session.set_impedance_mode(JointImpedanceMode.COMPLIANT_DRAG)
        except Exception as error:
            response.success = False
            response.message = f"Failed to request COMPLIANT_DRAG: {error}"
            return response

        response.success = True
        response.message = (
            "COMPLIANT_DRAG request submitted; hand-eye dragging may continue"
        )
        return response

    def stop_session(self) -> None:
        """Stop SerialArm exactly once using the Core lifecycle."""
        if self._stopped:
            return
        self._stopped = True
        self._resume_drag_pending = False
        if not getattr(self, "_session_started", False):
            return
        try:
            self._session.stop()
        except Exception as error:
            self.get_logger().error(
                f"Failed to stop SerialArm session cleanly: {error}"
            )
        finally:
            self._session_started = False

    def destroy_node(self):
        self.stop_session()
        return super().destroy_node()


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = HandeyeBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_session()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
