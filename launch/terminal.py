from launch import LaunchDescription
from launch.descriptions import executable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
        ),
        Node(
            package='joy_control',
            executable='control_ex',
        ),        
        Node(
            package='camera_test',
            executable='listener'
        ),        
        Node(
            package='camera_test',
            executable='listener2'
        ),
        Node(
            package='ui',
            executable='ui_health',
            output='screen',
            prefix='xterm -e'
        ),
    ])
