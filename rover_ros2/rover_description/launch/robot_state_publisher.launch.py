from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # ----- Directories
    pkg_share = get_package_share_directory('rover_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'rerassor.xacro.urdf'])

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # ----- Nodes
    rs_pub = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters= [{
                    'robot_description': robot_description,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'use_ros2_control': LaunchConfiguration('use_ros2_control')
                }])
    
    rjs_gui_pub = Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                output='screen',
                parameters= [{
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }],
                arguments=[
                    xacro_file
                ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ros2_control', default_value='false'),

        rjs_gui_pub,
        rs_pub
    ])