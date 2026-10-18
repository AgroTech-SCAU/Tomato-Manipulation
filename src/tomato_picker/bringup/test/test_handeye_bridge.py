import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest

import numpy as np


BRIDGE = Path(__file__).resolve().parents[1] / "scripts" / "handeye_bridge.py"
LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "handeye.launch.py"
CMAKE = Path(__file__).resolve().parents[1] / "CMakeLists.txt"
PACKAGE_XML = Path(__file__).resolve().parents[1] / "package.xml"


def _install_import_stubs():
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    from types import SimpleNamespace

    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=None, y=None, z=None),
                orientation=SimpleNamespace(x=None, y=None, z=None, w=None),
            )

    class TransformStamped:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.child_frame_id = ""
            self.transform = SimpleNamespace(
                translation=SimpleNamespace(x=None, y=None, z=None),
                rotation=SimpleNamespace(x=None, y=None, z=None, w=None),
            )

    geometry_msgs_msg.PoseStamped = PoseStamped
    geometry_msgs_msg.TransformStamped = TransformStamped
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node.Node = Node
    rclpy.node = rclpy_node
    rclpy_signals = types.ModuleType("rclpy.signals")

    class SignalHandlerOptions:
        NO = object()

    rclpy_signals.SignalHandlerOptions = SignalHandlerOptions
    rclpy.signals = rclpy_signals
    sys.modules["rclpy.signals"] = rclpy_signals

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class JointState:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.name = []
            self.position = []
            self.velocity = []
            self.effort = []

    sensor_msgs_msg.JointState = JointState
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self):
            self.data = ""

    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")

    class Trigger:
        pass

    std_srvs_srv.Trigger = Trigger
    std_srvs.srv = std_srvs_srv
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs_srv

    serial_arm = types.ModuleType("serial_arm")

    class JointImpedanceMode:
        RIGID_HOLD = object()
        COMPLIANT_DRAG = object()

    class RobotState:
        ACTIVE = object()
        FAULT = object()

    class RobotSession:
        pass

    serial_arm.JointImpedanceMode = JointImpedanceMode
    serial_arm.RobotState = RobotState
    serial_arm.RobotSession = RobotSession
    serial_arm.load_robot_profile_core = lambda *_args, **_kwargs: None
    sys.modules["serial_arm"] = serial_arm


def _load_bridge_module():
    _install_import_stubs()
    spec = importlib.util.spec_from_file_location("handeye_bridge_under_test", BRIDGE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class HandeyeBridgeContractTest(unittest.TestCase):
    def test_bridge_file_exists(self):
        self.assertTrue(BRIDGE.is_file())

    def test_identity_transform_becomes_identity_pose(self):
        module = _load_bridge_module()
        position, quaternion = module.transform_matrix_to_pose(np.eye(4))
        np.testing.assert_allclose(position, [0.0, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(quaternion, [0.0, 0.0, 0.0, 1.0], atol=1e-12)

    def test_transform_conversion_keeps_translation_and_normalizes_quaternion(self):
        module = _load_bridge_module()
        angle = math.pi / 2.0
        transform = np.array(
            [
                [math.cos(angle), -math.sin(angle), 0.0, 0.12],
                [math.sin(angle), math.cos(angle), 0.0, -0.34],
                [0.0, 0.0, 1.0, 0.56],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        position, quaternion = module.transform_matrix_to_pose(transform)
        np.testing.assert_allclose(position, [0.12, -0.34, 0.56], atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0, places=12)
        np.testing.assert_allclose(np.abs(quaternion), [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)], atol=1e-12)

    def test_bridge_uses_core_session_drag_and_cached_fk(self):
        source = BRIDGE.read_text(encoding="utf-8")
        self.assertIn("load_robot_profile_core(profile_name, profile_file or \"\", resource_paths)", source)
        self.assertIn("RobotSession(", source)
        self.assertIn("JointImpedanceMode.COMPLIANT_DRAG", source)
        self.assertIn("snapshot.dynamics.tool_pose", source)
        self.assertIn("self._session.config.dynamics.base_frame", source)
        self.assertIn("PoseStamped", source)
        self.assertIn('"/arm/pose"', source)
        self.assertIn("self._session.stop()", source)
        self.assertNotIn("joint_trajectory_controller", source)
        self.assertNotIn("move_group", source)

    def test_handeye_launch_is_dedicated_and_minimal(self):
        self.assertTrue(LAUNCH.is_file())
        source = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"robot_profile": "tomato_picker"', source)
        self.assertIn('robot_profiles.yaml', source)
        self.assertIn('resource_paths', source)
        self.assertIn('"pose_topic": "/arm/pose"', source)
        self.assertIn('"publish_rate": "30.0"', source)
        self.assertIn('executable="handeye_bridge"', source)
        self.assertIn('"joint_state_topic": "/arm/joint_states"', source)
        for forbidden in (
            "moveit.launch.py",
            "arm_motion_node",
            "pick_task_node",
            "eef_controller",
            "cloud_preprocessor",
            "wrist_target_gui",
        ):
            self.assertNotIn(forbidden, source)


    def test_v052_fault_recovery_services_are_exposed(self):
        source = BRIDGE.read_text(encoding="utf-8")
        self.assertIn("RobotState", source)
        self.assertIn("Trigger", source)
        self.assertIn('"/handeye/fault/clear"', source)
        self.assertIn('"/handeye/fault/compliant_recovery"', source)
        self.assertIn('"/handeye/fault/rigid_hold"', source)
        self.assertIn('"/handeye/drag/enable"', source)
        self.assertIn("self._session.clear_fault()", source)
        self.assertIn("self._session.enter_fault_compliant_recovery()", source)
        self.assertIn("self._session.return_to_fault_rigid_hold()", source)

    def test_pose_publication_requires_active_valid_snapshot(self):
        source = BRIDGE.read_text(encoding="utf-8")
        self.assertIn("RobotState.ACTIVE", source)
        self.assertIn("snapshot.valid", source)
        self.assertIn("snapshot.last_error", source)
        self.assertIn("self._session.state != RobotState.ACTIVE", source)

    def test_fault_clear_does_not_automatically_reenable_drag(self):
        source = BRIDGE.read_text(encoding="utf-8")
        clear_start = source.index("def _clear_fault")
        clear_end = source.index("def _enter_fault_compliant_recovery", clear_start)
        clear_body = source[clear_start:clear_end]
        self.assertIn("self._session.clear_fault()", clear_body)
        self.assertNotIn("JointImpedanceMode.COMPLIANT_DRAG", clear_body)

    def test_drag_enable_requires_fresh_active_snapshot(self):
        source = BRIDGE.read_text(encoding="utf-8")
        drag_start = source.index("def _request_drag")
        drag_end = source.index("def _enable_drag", drag_start)
        drag_body = source[drag_start:drag_end]
        self.assertIn("RobotState.ACTIVE", drag_body)
        self.assertIn("snapshot.valid", drag_body)
        self.assertIn("JointImpedanceMode.COMPLIANT_DRAG", drag_body)

    def test_joint_state_uses_same_snapshot_and_timestamp_as_pose(self):
        module = _load_bridge_module()
        from types import SimpleNamespace

        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = module.HandeyeBridge.__new__(module.HandeyeBridge)
        node._shutting_down = False
        node._session_started = True
        node._base_frame = "base_link"
        node._tool_frame = "tool0"
        node._joint_names = [f"joint{i}" for i in range(1, 7)]
        node._publisher = Publisher()
        node._transform_publisher = Publisher()
        node._joint_state_publisher = Publisher()
        stamp = SimpleNamespace(sec=123, nanosec=456)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp))
        errors = []
        node.get_logger = lambda: SimpleNamespace(error=errors.append)
        feedback = SimpleNamespace(pos=[0.1*i for i in range(6)], vel=[0.0]*6, tor=[0.2]*6)
        snapshot = SimpleNamespace(valid=True, last_error="", dynamics=SimpleNamespace(tool_pose=np.eye(4)),
                                   cycle=SimpleNamespace(joint_state=feedback))
        node._session = SimpleNamespace(snapshot=snapshot, state=module.RobotState.ACTIVE, running=True)
        node._publish_pose()

        self.assertFalse(errors)
        self.assertEqual(len(node._publisher.messages), 1)
        self.assertEqual(len(node._transform_publisher.messages), 1)
        self.assertEqual(len(node._joint_state_publisher.messages), 1)
        joints = node._joint_state_publisher.messages[0]
        self.assertEqual(joints.name, ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        np.testing.assert_allclose(joints.position, feedback.pos)
        np.testing.assert_allclose(joints.velocity, feedback.vel)
        np.testing.assert_allclose(joints.effort, feedback.tor)
        self.assertIs(joints.header.stamp, stamp)
        self.assertIs(node._publisher.messages[0].header.stamp, stamp)
        self.assertIs(node._transform_publisher.messages[0].header.stamp, stamp)

        feedback.pos[2] = float("nan")
        node._publish_pose()
        self.assertEqual(len(node._joint_state_publisher.messages), 1)
        self.assertEqual(len(node._publisher.messages), 1)
        self.assertEqual(len(node._transform_publisher.messages), 1)

    def test_build_metadata_installs_bridge_and_declares_direct_dependencies(self):
        cmake = CMAKE.read_text(encoding="utf-8")
        package_xml = PACKAGE_XML.read_text(encoding="utf-8")
        self.assertIn("scripts/handeye_bridge.py", cmake)
        self.assertIn("RENAME handeye_bridge", cmake)
        self.assertIn("<exec_depend>geometry_msgs</exec_depend>", package_xml)
        self.assertIn("<exec_depend>sensor_msgs</exec_depend>", package_xml)
        self.assertIn("<exec_depend>serial_arm_core</exec_depend>", package_xml)
        self.assertIn("<exec_depend>python3-numpy</exec_depend>", package_xml)


if __name__ == "__main__":
    unittest.main()
