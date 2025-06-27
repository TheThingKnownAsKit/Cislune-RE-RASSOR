from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = os.path.join('/', get_package_share_directory('rover_description'))

    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'rerassor.urdf.xacro'])
    robot_description = {'robot_description': Command(['xacro', xacro_path])}

    return LaunchDescription([
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             output='screen',
             parameters=[robot_description]),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[robot_description]),
        Node(package='rviz2', executable='rviz2', output='screen')
    ])
