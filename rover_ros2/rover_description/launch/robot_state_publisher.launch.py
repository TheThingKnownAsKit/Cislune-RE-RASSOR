from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'rerassor.xacro.urdf'])

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    return LaunchDescription([
        # publishes TF and robot_description
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{
                'robot_description': robot_description
             }],
             output='screen')
    ])