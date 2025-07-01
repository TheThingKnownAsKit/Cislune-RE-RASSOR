from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # ----- Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')


    # ----- Directories
    pkg_share = get_package_share_directory('rover_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'rerassor.xacro.urdf'])

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)


    # ----- Declare common params
    params = [{
        'robot_description': robot_description,
        'use_sim_time': use_sim_time,
        'use_ros2_control': use_ros2_control
    }]

    
    # ----- Nodes
    rs_pub = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=params
                )
    
    rjs_pub = Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=params
                )

    return LaunchDescription([
        rs_pub,
        rjs_pub
    ])