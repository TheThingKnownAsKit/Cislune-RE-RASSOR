import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'view_model_config.rviz'])

    # Call the separate robot state publisher launcher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([(pkg_share), 'launch', 'robot_state_publisher.launch.py'])]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz = Node(package='rviz2',
             executable='rviz2',
             output='screen',
             name='view_model_rviz',
             arguments=['-d', rviz_config_file])

    return LaunchDescription([
        rsp,
        rviz
    ])
