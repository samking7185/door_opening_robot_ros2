
import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            parameters=[]),
        launch_ros.actions.Node(
            package='do_teleop', 
            executable='do_teleop_node',
            name='do_teleop_node', 
            parameters=[],
            ),
    ])