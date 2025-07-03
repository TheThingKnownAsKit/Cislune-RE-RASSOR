from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
import os

from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Load and process the Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('rover_description'),
        'urdf',
        'rover.urdf.xacro'
    )
    robot_desc = xacro.process_file(xacro_file, mappings={'use_sim': 'false'})
    robot_description = {'robot_description': robot_desc.toxml()}

    # Load controller parameters
    controller_yaml = os.path.join(
        get_package_share_directory('rover_bringup'),
        'config',
        'diff_conts.yaml'
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # ros2_control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_yaml],
        output='screen'
    )

    # Joystick nodes
    joy = Node(package='joy', executable='joy_node', output='screen',
          parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05}])
    joy_teleop_cfg = os.path.join(get_package_share_directory('rover_bringup'), 'config', 'joy_teleop.yaml')
    teleop = Node(package='teleop_twist_joy', executable='teleop_node', output='screen',
              parameters=[joy_teleop_cfg],
              remappings=[('/cmd_vel', '/diff_cont/cmd_vel_stamped')])


    # Spawner for diff_cont
    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    # Spawner for joint_state_broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Delay control startup
    delayed_control = TimerAction(period=3.0, actions=[control_node])

    # Ensure spawners wait for control node
    diff_spawner_after = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[diff_spawner]
        )
    )

    jsb_spawner_after = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[jsb_spawner]
        )
    )

    return LaunchDescription([
        rsp,
        delayed_control,
        diff_spawner_after,
        jsb_spawner_after
    ])
