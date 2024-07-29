from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adaptive_clustering',
            executable='adaptive_clustering_node',
            name='adaptive_clustering',
            output='screen',
            parameters=[{'print_fps': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', '/path/to/your/adaptive_clustering.rviz']
        )
    ])
