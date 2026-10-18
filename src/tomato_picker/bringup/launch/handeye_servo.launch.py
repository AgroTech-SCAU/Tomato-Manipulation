"""Minimal, exclusive ros2_control / MoveIt stack for hand-eye RGB-D servo validation.

Unlike handeye.launch.py this does not start a second RobotSession or enable drag.
The application talks to the normal MoveGroup action and observes /arm/pose.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    profile_args = {
        "robot_profile": LaunchConfiguration("robot_profile"),
        "profile_file": LaunchConfiguration("profile_file"),
        "resource_paths": LaunchConfiguration("resource_paths"),
        "serial_port": LaunchConfiguration("serial_port"),
        "baudrate": LaunchConfiguration("baudrate"),
        "bus": LaunchConfiguration("bus"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "controller_manager_name": LaunchConfiguration("controller_manager_name"),
    }
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("serial_arm_ros2_control"), "launch", "hardware.launch.py",
        ])),
        launch_arguments=profile_args.items(),
    )
    # Start only move_group, not RViz, task/EEF, perception or a second robot driver.
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("tomato_picker_moveit_config"), "launch", "move_group.launch.py",
        ])),
    )
    pose_bridge = Node(
        package="tomato_picker_bringup",
        executable="handeye_servo_pose",
        name="handeye_servo_pose",
        output="screen",
        parameters=[{
            "base_frame": LaunchConfiguration("base_frame"),
            "tool_frame": LaunchConfiguration("end_frame"),
            "pose_topic": LaunchConfiguration("pose_topic"),
            "transform_topic": LaunchConfiguration("transform_topic"),
            "publish_rate": ParameterValue(LaunchConfiguration("pose_rate"), value_type=float),
            "max_tf_age_sec": ParameterValue(LaunchConfiguration("max_tf_age_sec"), value_type=float),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )
    wrist_camera = GroupAction(
        condition=IfCondition(LaunchConfiguration("start_wrist_camera")),
        actions=[
            PushRosNamespace("camera"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("orbbec_camera"), "launch", "gemini_330_series.launch.py",
                ])),
                launch_arguments={
                    "camera_name": "wrist",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "color_width": "1280",
                    "color_height": "800",
                    "color_fps": "30",
                    "depth_width": "1280",
                    "depth_height": "800",
                    "depth_fps": "30",
                    "depth_registration": "true",
                    "align_mode": "SW",
                    "align_target_stream": "COLOR",
                    "enable_frame_sync": "true",
                    "enable_point_cloud": "false",
                    "enable_colored_point_cloud": "false",
                }.items(),
            ),
        ],
    )
    ready_gate = Node(
        package="tomato_picker_bringup",
        executable="wait_for_arm_ready",
        name="handeye_servo_ready",
        output="screen",
        parameters=[{
            "controller_manager": LaunchConfiguration("controller_manager_name"),
            "move_action": "/move_action",
            "timeout_sec": 60.0,
            "stable_sec": 1.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_profile", default_value="tomato_picker"),
        DeclareLaunchArgument("profile_file", default_value=PathJoinSubstitution([
            FindPackageShare("tomato_picker_bringup"), "config", "robot_profiles.yaml",
        ])),
        DeclareLaunchArgument("resource_paths", default_value=""),
        DeclareLaunchArgument("serial_port", default_value=""),
        DeclareLaunchArgument("baudrate", default_value=""),
        DeclareLaunchArgument("bus", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("controller_manager_name", default_value="/controller_manager"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("end_frame", default_value="tool0"),
        DeclareLaunchArgument("pose_topic", default_value="/arm/pose"),
        DeclareLaunchArgument("transform_topic", default_value="/arm/tool_transform"),
        DeclareLaunchArgument("pose_rate", default_value="30.0"),
        DeclareLaunchArgument("max_tf_age_sec", default_value="0.25"),
        DeclareLaunchArgument("start_wrist_camera", default_value="true"),
        LogInfo(msg="Hand-eye servo mode: ros2_control owns hardware; no drag bridge or picking task"),
        hardware,
        move_group,
        pose_bridge,
        wrist_camera,
        ready_gate,
    ])
