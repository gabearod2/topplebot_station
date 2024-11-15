from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the URDF file
    package_dir = get_package_share_directory('viz')
    urdf_file = os.path.join(package_dir, 'urdf', 'topplebot.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='viz',
            executable='tb_tf_broadcaster',
            name='imu_to_tf_broadcaster',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_dir, 'config', 'topplebot.rviz')]
        )
    ])
