from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # ----- Directories and paths
    pkg_nav = get_package_share_directory('rover_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    map_path  = PathJoinSubstitution([pkg_nav, 'maps', 'placeholder.yaml'])
    nav_config_path    = PathJoinSubstitution([pkg_nav, 'config', 'nav2.yaml'])
    slam_config_path = PathJoinSubstitution([pkg_nav, 'config', 'slam.yaml'])

    # ----- Nodes

    # Launch localization (use prebuilt map)
    localization_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'localization_launch.py'])]),
        launch_arguments={
            'map': map_path,
            'params_file': nav_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_prebuilt_map'))
    )

    # Launch navigation
    navigation_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])]),
        launch_arguments={
            'params_file': nav_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true'
        }.items()
    )

    # Launch slam toolbox (only if using real-time slam aka use_prebuilt_map = false)
    slam_toolbox_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_slam, 'launch', 'online_async_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time':LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_config_path
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_prebuilt_map'))
    )

    # Rviz launch here somewhere

    # Need to send waypoint data here also. IncludesWaypointMissionand Waypoint Task Executor

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_prebuilt_map', default_value='true'),

        localization_bringup,
        navigation_bringup,
        slam_toolbox_bringup
    ])

