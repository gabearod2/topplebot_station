from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('src/viz/topplebot.urdf').read()}]
        ),
        Node(
            package='viz',
            executable='imu_to_tf_broadcaster',
            name='imu_to_tf_broadcaster',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'src/viz/config/topplebot.rviz']
        )
    ])
