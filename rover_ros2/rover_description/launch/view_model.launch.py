from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os, xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'rerassor.xacro.urdf'])
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz2', 'view_model_config.rviz'])

    robot_description_content = Command(
        ['xacro ', xacro_file])
    robot_description = ParameterValue(
        robot_description_content, value_type=str)
    
    params = [{'robot_description': robot_description}]

    return LaunchDescription([
        # publishes TF and robot_description
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=params,
             output='screen'),

        # GUI to tweak joint positions
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             parameters=params,
             output='screen'),

        # Open RViz
        Node(package='rviz2',
             executable='rviz2',
             output='screen',
             name='view_model_rviz',
             arguments=['-d', rviz_config_file])
    ])
