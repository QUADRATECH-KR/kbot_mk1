import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('kbot'))
 
    params = os.path.join(pkg_path, 'config', 'rplidar_params.yaml')

    rplidar_node = Node(
                        package='rplidar_ros',
                        executable='rplidar_composition',
                        parameters=[params]
                       )
    
    
    return LaunchDescription([
        rplidar_node
    ])
