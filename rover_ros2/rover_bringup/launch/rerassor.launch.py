from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # ----- Directories
    pkg_control = get_package_share_directory('rover_control')
    pkg_nav = get_package_share_directory('rover_navigation')
    pkg_description = get_package_share_directory('rover_description')

    # ----- Parameters
    mode = LaunchConfiguration('mode') # Possible values: teleop, autonomy, or dual

    # ----- Nodes

    # Robot description
    robot_state_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_description, 'launch', 'robot_state_publisher.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'use_ros2_control': 'true',
                'use_gazebo': 'false',
                'use_js_gui': 'false'
            }.items())

    # Initialize Navigation

    # Initialize controls
    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'controller.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'use_teleop': PythonExpression(["'", mode, "' != 'autonomy'"])
            }.items())

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='teleop',
                              description="Mode: teleop, autonomy, or dual (teleop and autonomy with toggle)"),

        robot_state_node,
        controller_node
    ])

