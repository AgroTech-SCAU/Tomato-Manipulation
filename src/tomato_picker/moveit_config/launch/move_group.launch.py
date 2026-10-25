from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("Tomato-Picker", package_name="tomato_picker_moveit_config")
        .to_moveit_configs()
    )
    description = generate_move_group_launch(moveit_config)
    description.entities.insert(0, SetParameter(name='use_sim_time', value=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)))
    description.entities.insert(0, DeclareLaunchArgument('use_sim_time', default_value='false'))
    return description
