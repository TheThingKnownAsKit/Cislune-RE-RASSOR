import os

from ament_index_python.packages import get_package_share_directory

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_control = get_package_share_directory('rover_control')
    use_sim_time = LaunchConfiguration('use_sim_time')
    diff_cont_file = os.path.join(pkg_control,'config','diff_cont.yaml')

    robot_description_content = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Create joy node for joystick input
    teleop_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'joystick.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('use_teleop'))
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node', # Changed
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
            }, diff_cont_file
        ]
    )
    
    diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive]
        )
    )

    delayed_joint_broad = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_teleop', default_value='false'),

        teleop_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad
    ])