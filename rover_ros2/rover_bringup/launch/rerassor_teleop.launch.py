from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os, xacro, tempfile
from launch.actions import ExecuteProcess

def generate_launch_description():

    # ----- Directories
    pkg_description = get_package_share_directory('rover_description')
    pkg_control = get_package_share_directory('rover_control')

    # ----- Create nodes
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_description, 'launch', 'robot_state_publisher.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'use_ros2_control': 'true'
            }.items())

    # Create joy node for joystick input
    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'joy.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Create the teleop node to listen to joy
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'teleop.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Create controller manager nodes
    joint_broad_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', '-c', '/controller_manager'
    ])
    diff_cont_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '-c', '/controller_manager',
            '--param-file', PathJoinSubstitution([
                pkg_control, 'controllers', 'diff_cont.yaml'
            ]),
            '--ros-args', '-p', 'use_sim_time:=false'
        ]
    )

    spawn_micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        joint_broad_node,
        diff_cont_node,
        joy,
        teleop,
        spawn_micro_ros_agent
    ])
