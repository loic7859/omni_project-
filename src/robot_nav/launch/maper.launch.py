# slam_toolbox_mapping.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

   
def generate_launch_description():
    maper_params = os.path.join(
            get_package_share_directory('robot_nav'),
            'config',
            'maper_param.yaml'
        )






    return LaunchDescription([

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # pour un SLAM synchrone
            name='slam_toolbox',
            output='screen',
            parameters=[maper_params,{'use_sim_time': True }], 
                  
        )
    ])
