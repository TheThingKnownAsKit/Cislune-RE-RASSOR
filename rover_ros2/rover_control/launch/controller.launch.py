from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_control = get_package_share_directory('rover_control')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create joy node for joystick input
    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'joy.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Create joint broadcaster node for /joint_states
    joint_broad_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', '-c', '/controller_manager'
    ])

    # This is the actual controller manager
    diff_cont_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '-c', '/controller_manager',
            '--param-file', PathJoinSubstitution([
                pkg_control, 'controllers', 'diff_cont.yaml'
            ])
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        joy,
        joint_broad_node,
        diff_cont_node
    ])