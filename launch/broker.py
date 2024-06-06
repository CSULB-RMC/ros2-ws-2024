from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors',
            executable='sensors_sub'
            ),
        Node(
            package='db_broker',
            executable='db_broker_pub',
        ),
    ])
