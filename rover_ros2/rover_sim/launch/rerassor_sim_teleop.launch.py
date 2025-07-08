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
    pkg_control = get_package_share_directory('rover_control')
    pkg_rosgz = get_package_share_directory('ros_gz_sim')


    # ----- Files
    world_file = PythonExpression(["'", LaunchConfiguration('world'), ".world'"])
    world_path = PathJoinSubstitution([pkg_sim, 'worlds', world_file])
    bridge_config_path = PathJoinSubstitution([pkg_sim, 'config', 'rosgz_bridge.yaml'])

    # expand Xacro → URDF *once* and write to a temp file
    #   (literally the only reason we need to do this is because RosGzBridge is bugged)
    xacro_file = os.path.join(pkg_description, 'urdf', 'rerassor.xacro.urdf')
    urdf_xml   = xacro.process_file(xacro_file, mappings={'use_ros2_control': 'true', 'use_gazebo': 'true'}).toxml()
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
                'use_sim_time': 'true',
                'use_ros2_control': 'true',
                'use_gazebo': 'true',
                'use_js_gui': 'false'
            }.items())

    # # Use Gazebo's pre-built sim launching node
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_rosgz, 'launch', 'gz_sim.launch.py'])
            ]),
            launch_arguments={
                'gz_args': ['-r ', world_path]
            }.items())

    # Create a node for the rosgz bridge (there is an object named RosGzBridge that does this but it's bugged)  
    rosgz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )
    
    # Use Gazebo's pre-built robot spawner
    spawn_rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_rosgz, 'launch', 'gz_spawn_model.launch.py'])
            ]),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'file': urdf_path,
                'entity_name': 'rerassor'
            }.items())

    # Initialize controls with sim_time as true
    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'controller.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'true'
            }.items())

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty_plane'),

        rsp,
        gz_sim,
        spawn_rover,
        rosgz_bridge,
        controller_node
    ])

