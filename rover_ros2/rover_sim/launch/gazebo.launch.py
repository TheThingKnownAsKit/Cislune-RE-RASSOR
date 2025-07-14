from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os, xacro, tempfile

def generate_launch_description():

    # ----- Directories
    pkg_description = get_package_share_directory('rover_description')
    pkg_sim = get_package_share_directory('rover_sim')
    pkg_rosgz = get_package_share_directory('ros_gz_sim')
    pkg_control = get_package_share_directory('rover_control')


    # ----- Files
    world_file = PythonExpression(["'", LaunchConfiguration('world'), ".world'"])
    world_path = PathJoinSubstitution([pkg_sim, 'worlds', world_file])
    bridge_config_path = PathJoinSubstitution([pkg_sim, 'config', 'rosgz_bridge.yaml'])
    twist_mux_params = os.path.join(pkg_control, 'config', 'twist_mux.yaml')


    # ----- Create nodes
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_description, 'launch', 'robot_state_publisher.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'use_ros2_control': 'true',
                'use_gazebo': 'true',
                'use_js_gui': 'false'
            }.items())

    # Use Gazebo's pre-built sim launching node
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_rosgz, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )


    # Create a node for the rosgz bridge (there is an object named RosGzBridge that does this but it's bugged)  
    rosgz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )

    # Twist mux helps manage the cmd_vel topic
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )
    
    # Use Gazebo's pre-built robot spawner
    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rerassor',
        arguments=[
            '-world', LaunchConfiguration('world'),
            '-name',  'rerassor',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '-c', '/controller_manager',
            '--param-file', PathJoinSubstitution([
                pkg_control, 'config', 'diff_cont.yaml'
            ])
        ]
    )

    joint_broad = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', '-c', '/controller_manager'
    ])

    teleop_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'joystick.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty_plane'),

        rsp,
        gz_sim,
        rosgz_bridge,
        spawn_rover,
        diff_drive,
        joint_broad,
        teleop_node,
        twist_mux
    ])

