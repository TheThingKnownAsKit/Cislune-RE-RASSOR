import os, xacro, tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # ----- Launch configurations


    # ----- Directories
    pkg_description = get_package_share_directory('rover_description')
    pkg_sim = get_package_share_directory('rover_sim')
    pkg_rosgz = get_package_share_directory('rosgz')
    pkg_bridge = get_package_share_directory('ros_gz_bridge')

    world_path = PathJoinSubstitution([pkg_sim, 'worlds', LaunchConfiguration('world')])

    # ----- Create nodes
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([(pkg_description), 'launch', 'robot_state_publisher.launch.py'])]),
        launch_arguments={
            'use_sim_time': 'true'
            }.items())

    # Use Gazebo's pre-built sim launching node
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rosgz, 'launch', 'gz_sim.launch.py'])),
            launch_arguments={
                'gz_args': ['-r ', world_path]
            }.items())
    
    # Use Gazebo's pre-built ros bridge node
    rosgz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bridge, 'launch', 'ros_gz_bridge.launch.py'])
        ))

    # Use Gazebo's pre-built robot spawner
    spawn_rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(pkg_rosgz, 'launch', 'gz_spawn_model.launch.py')
        ))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty_plane.world'),
        rsp,
        gz_sim,
        rosgz_bridge,
        spawn_rover
    ])

