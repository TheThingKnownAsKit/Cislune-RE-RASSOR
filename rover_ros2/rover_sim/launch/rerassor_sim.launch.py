from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # ----- Directories
    pkg_sim = get_package_share_directory('rover_sim')
    pkg_control = get_package_share_directory('rover_control')

    # ----- Nodes

    # Initialize Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_sim, 'launch', 'gazebo.launch.py'])
            ]))

    # Initialize controls with sim_time as true
    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'controller.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'true'
            }.items())

    return LaunchDescription([
        gazebo_node,
        controller_node
    ])

