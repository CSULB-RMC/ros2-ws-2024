from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive_cb',
            executable='drivetrain_cb',
        ),
        Node(
            package='camera_test',
            executable='talker2'
        ),
    ])
