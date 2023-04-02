import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import yaml

def generate_launch_description():

    config_filepath = os.path.join(
        get_package_share_directory('bringup'), 
        'config'
    )

    joy_config_filepath = os.path.join(
        config_filepath,
        "joy.config.yaml"
    )

    teleop_config_filepath = os.path.join(
        config_filepath,
        "teleop.config.yaml"
    )

    robot_localization_config_filepath = os.path.join(
        config_filepath,
        "robot_localization.config.yaml"
    )

    return launch.LaunchDescription(
        [
			launch_ros.actions.Node(
				package='joy', 
				executable='joy_node', 
				name='joy_node',
				parameters=[joy_config_filepath]
			),

			launch_ros.actions.Node(
				package='teleop_twist_joy', 
				executable='teleop_node',
				name='teleop_twist_joy_node',
				parameters=[teleop_config_filepath]
			),

			launch_ros.actions.Node(
				package='motion', 
				executable='motion',
				name='motion_node'
				#parameters=[teleop_config_filepath]
			),

			launch_ros.actions.Node(
				package='robot_localization',
				executable='ekf_node',
				name='ekf_filter_node',
				output='screen',
				parameters=[robot_localization_config_filepath]
			),
            

		]
    ) # return LD
