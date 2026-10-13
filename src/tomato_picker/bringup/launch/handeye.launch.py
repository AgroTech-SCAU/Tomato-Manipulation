from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    names = (
        "robot_profile", "profile_file", "resource_paths", "serial_port",
        "baudrate", "bus", "pose_topic", "transform_topic", "publish_rate", "recovery_wait_s",
        "park_on_shutdown",
    )
    config = {name: LaunchConfiguration(name) for name in names}
    bridge = Node(
        package="tomato_picker_bringup",
        executable="handeye_bridge",
        name="handeye_bridge",
        output="screen",
        emulate_tty=True,
        # Let the Python bridge complete Core shutdown before launch terminates it
        sigterm_timeout="35",
        sigkill_timeout="10",
        parameters=[{
            key: ParameterValue(config[key], value_type=(
                bool if key == "park_on_shutdown" else
                float if key in ("publish_rate", "recovery_wait_s") else str
            ))
            for key in names
        }],
    )
    defaults = {
        "robot_profile": "tomato_picker",
        "profile_file": PathJoinSubstitution([
            FindPackageShare("tomato_picker_bringup"), "config", "robot_profiles.yaml"
        ]),
        "resource_paths": "",
        "serial_port": "",
        "baudrate": "",
        "bus": "",
        "pose_topic": "/arm/pose",
        "transform_topic": "/arm/tool_transform",
        "publish_rate": "30.0",
        "recovery_wait_s": "3.0",
        # Ctrl+C parks at the configured shutdown position before deactivation
        "park_on_shutdown": "true",
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
        bridge,
    ])
