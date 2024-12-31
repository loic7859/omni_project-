from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omni_robot_controller',
            executable='omni_control.py',
            name='omni_controller',
            output='screen'
        )
    ])