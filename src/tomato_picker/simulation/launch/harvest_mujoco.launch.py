"""MuJoCo dynamics, production MoveIt, contact-based gripper and harvest observations."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
import yaml


def setup(context):
    share = Path(get_package_share_directory('tomato_picker_mujoco'))
    bringup = Path(get_package_share_directory('tomato_picker_bringup'))
    description = {'robot_description': xacro.process_file(
        str(share / 'urdf/tomato_picker.mujoco.ros2_control.xacro'),
        mappings={'headless': LaunchConfiguration('headless').perform(context)}).toxml()}
    config = (MoveItConfigsBuilder('Tomato-Picker', package_name='tomato_picker_moveit_config')
              .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner']).to_moveit_configs())
    parameters = config.to_dict()
    parameters.update(description)
    joint_limits = yaml.safe_load((share / "config/joint_limits.yaml").read_text())
    parameters["robot_description_planning"] = joint_limits
    parameters.update(yaml.safe_load((share / "config/planning.yaml").read_text()))
    parameters.update({
        'use_sim_time': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'trajectory_execution.allowed_start_tolerance': .01,
    })
    manager = Node(package='mujoco_ros2_control', executable='ros2_control_node', output='screen',
                   parameters=[description, str(share / 'config/controllers.yaml'),
                               str(share / 'config/plugins.yaml'), {'use_sim_time': True}],
                   remappings=[('~/robot_description', '/robot_description')])
    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                           output='screen', parameters=[description, {'use_sim_time': True}])
    spawners = [Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=[name, '--controller-manager', '/controller_manager',
                                '--param-file', str(share / 'config/controllers.yaml')])
                for name in ('joint_state_broadcaster', 'joint_trajectory_controller', 'gripper_controller')]
    move_group = Node(package='moveit_ros_move_group', executable='move_group',
                      output='screen', parameters=[parameters])
    scene = Node(package='tomato_picker_bringup', executable='servo_simulation_scene', output='screen',
                 parameters=[{'scene_file': str(share / 'config/harvest_scene.yaml'), 'use_sim_time': True,
                              'physics_state_topic': '/simulation/physics/state', 'fruit_radius_m': .025}])
    vision = Node(package='vision_target_server', executable='mujoco_target_server', output='screen',
                  parameters=[{'use_sim_time': True, 'calibration_file': str(share / 'config/handeye.yaml'),
                               'harvest_scene_file': str(share / 'config/harvest_scene.yaml'),
                               'planning_scene_revision_topic': '/simulation/physics/planning_scene_revision'}])
    ready = Node(package='tomato_picker_bringup', executable='wait_for_arm_ready', output='screen',
                 parameters=[{'timeout_sec': 120., 'use_sim_time': True}])
    motion = Node(package='tomato_picker_motion', executable='arm_motion_node', output='screen',
                  parameters=[str(share / 'config/motion.yaml'),
                              {'robot_description_planning': joint_limits}])
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', str(bringup / 'config/servo_simulation/servo.rviz')],
                parameters=[parameters])

    def after_ready(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='MuJoCo controllers or MoveIt failed readiness'))]
        return [motion]

    critical = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='MuJoCo component exited'))]))
        for node in (manager, state_publisher, move_group, motion, scene, vision)]
    return [*critical, manager, state_publisher, *spawners, move_group, scene, vision, rviz,
            RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_ready)), ready]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true', description='Hide MuJoCo native viewer'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
