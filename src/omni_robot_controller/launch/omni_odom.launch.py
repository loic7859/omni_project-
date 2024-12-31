from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    controller_params = os.path.join(
        get_package_share_directory('omni_robot_controller'),
        'config',
        'odom.yaml'
    )

    return LaunchDescription([

        Node(
            package='omni_robot_controller',
            executable='odom_control.py',
            name='node_odem',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),




      


    ])




"""
        Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_node',
    output='screen',
    parameters=[controller_params],
    )
"""