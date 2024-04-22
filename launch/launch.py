from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            namespace='joy_cb',
            executable='joy_node',
            name='joy_node_cb',
            value='/dev/input/js0'
        ),
         Node(
            package='joy',
            namespace='joy_ex',
            executable='joy_node',
            name='joy_node_ex',
            value='/dev/input/js1'
            ),
        
    ])
