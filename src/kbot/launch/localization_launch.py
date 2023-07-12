from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, ExecuteProcess


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('kbot'))
 
    params_slam_localization = os.path.join(pkg_path, 'config', 'slam_localization.yaml')
    

    map_server_node = Node(
                        package='nav2_map_server',
                        executable='map_server',
                        parameters=[params_slam_localization]
                       )
                       
    amcl_node       = Node(
                        package='nav2_amcl',
                        executable='amcl',
                        parameters=[params_slam_localization]
                       )                

    
    map_server_lifecycle_process = ExecuteProcess(
                                  cmd=[['ros2 run nav2_util lifecycle_bringup map_server']], 
                                  shell = True
                                )
                                
    amcl_lifecycle_process       = ExecuteProcess(
                                  cmd=[['ros2 run nav2_util lifecycle_bringup amcl']], 
                                  shell = True
                                )
                                
    
    delayed_map_server_lifecycle_process = TimerAction(period=2.0, actions=[map_server_lifecycle_process])
    delayed_amcl_lifecycle_process       = TimerAction(period=2.0, actions=[amcl_lifecycle_process])                   
    
    
    
    return LaunchDescription([
        map_server_node,
        amcl_node,
        delayed_map_server_lifecycle_process,
        delayed_amcl_lifecycle_process
    ])
