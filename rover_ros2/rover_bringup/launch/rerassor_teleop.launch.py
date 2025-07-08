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

    # expand Xacro → URDF *once* and write to a temp file
    #   (literally the only reason we need to do this is because RosGzBridge is bugged)
    xacro_file = os.path.join(pkg_description, 'urdf', 'rerassor.xacro.urdf')
    urdf_xml   = xacro.process_file(xacro_file, mappings={'use_ros2_control': 'true', 'use_gazebo': 'false'}).toxml()
    tmp_urdf   = tempfile.NamedTemporaryFile(delete=False,
                                             suffix='.urdf',
                                             prefix='rerassor_')
    tmp_urdf.write(urdf_xml.encode())
    tmp_urdf.close()                       # keep the file on disk
    urdf_path = tmp_urdf.name              # path we’ll hand to the spawner


    # ----- Create nodes
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_description, 'launch', 'robot_state_publisher.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'use_ros2_control': 'true',
                'use_gazebo': 'false'
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

    return LaunchDescription([
        rsp,
        joint_broad_node,
        diff_cont_node,
        joy,
        teleop
    ])
