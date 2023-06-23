import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('kbot'))
 
    params = os.path.join(pkg_path, 'config', 'slam_amcl_params.yaml')

    map_server_node = Node(
                        package='nav2_map_server',
                        executable='map_server',
                        parameters=[params]
                       )
                       
    amcl_node       = Node(
                        package='nav2_amcl',
                        executable='amcl'
                       )
                       
                       
    map_server_lifecycle_node = Node(
                                  package='nav2_util',
                                  executable='lifecycle_bringup',
                                  parameters=["map_server"]
                                )
                                
    amcl_lifecycle_node       = Node(
                                  package='nav2_util',
                                  executable='lifecycle_bringup',
                                  parameters=["amcl"]
                                )
                       
    
    
    
    return LaunchDescription([
        map_server_node,
        amcl_node,
    ])
