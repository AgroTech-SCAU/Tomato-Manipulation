"""RViz viewer for an already running physical harvesting workflow."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def generate_launch_description():
    share = Path(get_package_share_directory('tomato_picker_mujoco'))
    config = (MoveItConfigsBuilder('Tomato-Picker', package_name='tomato_picker_moveit_config')
              .planning_pipelines(pipelines=['ompl']).to_moveit_configs()).to_dict()
    config.update(robot_description=xacro.process_file(
        str(share / 'urdf/tomato_picker.mujoco.ros2_control.xacro')).toxml(), use_sim_time=True)
    return LaunchDescription([Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', str(Path(get_package_share_directory('tomato_picker_bringup')) /
                             'config/servo_simulation/servo.rviz')], parameters=[config])])
