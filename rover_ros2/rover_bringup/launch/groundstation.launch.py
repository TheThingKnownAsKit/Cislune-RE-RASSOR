from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # ----- Directories
    pkg_control = get_package_share_directory('rover_control')
    rviz_config_file = PathJoinSubstitution([pkg_control, 'rviz', 'groundstation.rviz'])

    # ----- Nodes

    # Teleop with joystick
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_control, 'launch', 'joystick.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # RViz node
    rviz = Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file]
        )

    return LaunchDescription([
        teleop_launch,
        rviz
    ])
