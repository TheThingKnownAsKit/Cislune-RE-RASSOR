from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Path to your Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('rover_description'),
        'urdf',
        'rover.urdf.xacro'  # or whatever it's named
    )

    # Process the Xacro file
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('rover_description'),
                'rviz',
                'display.rviz')],
            output='screen'
        )
    ])
