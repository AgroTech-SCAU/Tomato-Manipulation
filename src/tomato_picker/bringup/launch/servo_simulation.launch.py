"""Kinematic arm simulation using the production MoveIt and trajectory controller."""
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


def setup(context):
    bringup = Path(get_package_share_directory('tomato_picker_bringup'))
    description_share = Path(get_package_share_directory('tomato_picker_description'))
    initial_file = LaunchConfiguration('initial_positions_file').perform(context)
    description = {'robot_description': xacro.process_file(
        str(description_share / 'urdf/tomato_picker.sim.ros2_control.xacro'),
        mappings={'initial_positions_file': initial_file}).toxml()}
    config = (MoveItConfigsBuilder('Tomato-Picker', package_name='tomato_picker_moveit_config')
              .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    parameters = config.to_dict()
    parameters.update(description)
    parameters.update({
        'use_sim_time': False,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    })
    manager = Node(package='controller_manager', executable='ros2_control_node', output='screen',
                   parameters=[description, str(bringup / 'config/ros2_controllers.yaml'),
                               {'use_sim_time': False}])
    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                           output='screen', parameters=[description, {'use_sim_time': False}])
    broadcaster = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    trajectory = Node(package='controller_manager', executable='spawner', output='screen',
                      arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'])
    eef = Node(package='tomato_picker_eef', executable='eef_mock_node', output='screen',
               parameters=[{'publish_joint_states': True, 'use_sim_time': False}])
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[parameters])
    ready = Node(package='tomato_picker_bringup', executable='wait_for_arm_ready', output='screen',
                 parameters=[{'timeout_sec': 30.0}])
    motion = Node(package='tomato_picker_motion', executable='arm_motion_node', output='screen',
                  parameters=[str(bringup / 'config/servo_simulation/motion.yaml')])
    scene = Node(package='tomato_picker_bringup', executable='servo_simulation_scene', output='screen',
                 parameters=[{'scene_file': LaunchConfiguration('scene_file'), 'use_sim_time': False}])
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', str(bringup / 'config/servo_simulation/servo.rviz')],
                parameters=[parameters])

    def after_ready(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Simulation controllers or MoveIt failed readiness'))]
        return [motion]

    critical = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason='Simulation component exited'))]))
        for node in (manager, state_publisher, move_group, scene, motion, eef)]
    return [*critical, manager, state_publisher, broadcaster, trajectory, eef, move_group, scene, rviz,
            RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_ready)), ready]


def generate_launch_description():
    share = Path(get_package_share_directory('tomato_picker_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('initial_positions_file', default_value=str(
            Path(get_package_share_directory('tomato_picker_description')) / 'config/simulation_initial_positions.yaml')),
        DeclareLaunchArgument('scene_file', default_value=str(share / 'config/servo_simulation/scene.yaml')),
        OpaqueFunction(function=setup),
    ])
