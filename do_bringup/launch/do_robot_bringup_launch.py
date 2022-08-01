import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    

    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('do_description'),'launch'),
            '/mobile_robot_launch.py'
        ])
    )

    gazebo_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('do_gazebosim'),'launch'),
            '/mobile_robot_gazebo_launch.py'
        ])
    )

    teleop_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('do_teleop'), 'launch'),
            '/do_teleop_launch.py'
        ])
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(robot_model)
    ld.add_action(gazebo_model)
    ld.add_action(teleop_node)
    
    return ld