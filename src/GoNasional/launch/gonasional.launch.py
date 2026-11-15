from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='GoNasional', executable='state', output='screen'),
        Node(package='GoNasional', executable='trigonocak', output='screen'),
    ])
