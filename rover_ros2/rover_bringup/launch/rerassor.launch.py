from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():

    # ----- Directories
    pkg_control = get_package_share_directory('rover_control')
    pkg_description = get_package_share_directory('rover_description')

    twist_mux_params = os.path.join(pkg_control, 'config', 'twist_mux.yaml')

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

    # Initialize controls
    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'controller.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'use_teleop': 'false'
            }.items())
    
    # Camera node for RealSense D455 launch with ROS2 wrapper
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            # ---- core ----
            'camera_name':            'd455',            # TF prefix
            # If you run multiple cameras, set serial_no:=<camera SN>
            'enable_sync':            'true',            # sync all enabled streams
            'initial_reset':          'true',            # HW reset at start-up

            # ---- video profiles ----
            'depth_module.profile':   '640x480x30',
            'color_module.profile':   '640x480x30',

            # ---- IMU (D455 has gyro+accel) ----
            'enable_gyro':            'true',
            'enable_accel':           'true',
            # imu fusion method 2 – linear interpolation between frames
            'unite_imu_method':       '2',               # see README

            # ---- point cloud & alignment ----
            'pointcloud.enable':      'true',
            'align_depth.enable':     'true',

            # ---- frame rate / performance tweaks ----
            # 'wait_for_device_timeout': '10.0',         # s, optional
            'publish_odom_tf':        'true',

            # pass through launch arg controlling whether camera should start
        }.items()
    )

    # Twist mux helps manage the cmd_vel topic
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )
    

    return LaunchDescription([
        robot_state_node,
        controller_node,
        camera_node
    ])

