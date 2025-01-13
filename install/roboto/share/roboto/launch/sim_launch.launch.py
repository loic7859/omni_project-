from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource





def generate_launch_description():
    
   
    pkg_name='roboto'
    file_subpath='robot_description/rviz_bot/robot.urdf.xacro'
    file_subpath2='robot_description/sim_bot/robots/robot_core.urdf.xacro'
    
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    

    robot_file =os.path.join(get_package_share_directory(pkg_name),file_subpath2)
    robot_description_raw2 = xacro.process_file(robot_file).toxml()

    
    test_file=robot_file =os.path.join(get_package_share_directory(pkg_name),'robot_description/sim_bot/robots/test.urdf')
    with open(test_file, 'r') as file:
        robot_test = file.read()
    
    rviz_file=robot_file =os.path.join(get_package_share_directory(pkg_name),'robot_description/rviz_bot/robot_rviz.urdf')
    with open(rviz_file, 'r') as file:
        robot_rviz = file.read()
    
    """
    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_test,
        'use_sim_time': True}], # add other parameters here if required
        remappings=[('/robot_description', '/rviz_description')]
    )
    """

    # Configure the node
    node_robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_test,
        'use_sim_time': True}], # add other parameters here if required
        remappings=[('/robot_description', '/gazebo_description')]
        
    )



    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        
        launch_arguments={'world': os.path.join(
        get_package_share_directory('roboto'), 'world', 'finalground.sdf')}.items()
    )
    


    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'gazebo_description','-entity', 'roboto'],
                        parameters=[{'use_sim_time': True}],     
                
                        output='screen')




    controller_params = os.path.join(
        get_package_share_directory('roboto'),
        'config',
        'my_controllers.yaml'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params,{'use_sim_time': True}],
        output='screen'
    )

    forward_position_controller = ExecuteProcess(
    cmd=[
        'ros2', 'control', 'load_controller', '--set-state', 'active', 
        'forward_velocity_controller'
        
    ],
    output='screen'
    )   

    joint_state_broadcaster = ExecuteProcess(
    cmd=[
        'ros2', 'control', 'load_controller', '--set-state', 'active', 
        'joint_state_broadcaster'
        
    ],
    output='screen'
    )



    # Run the node
    return LaunchDescription([
      gazebo , #node_robot_state_publisher,
        node_robot_state_publisher2,
        spawn_entity,controller_manager,joint_state_broadcaster,forward_position_controller
    ])

    
