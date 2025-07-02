from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = [
        os.path.join(get_package_share_directory('rover_control'), 'controllers', 'joystick.yaml'),
        {'use_sim_time': LaunchConfiguration('use_sim_time')}
    ]

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=joy_params,
         )

    teleop_node = Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=joy_params,
            remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
            )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        joy_node,
        teleop_node
    ])