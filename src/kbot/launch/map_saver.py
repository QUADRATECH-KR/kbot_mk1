import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    print(os.getcwd())
    if(not os.path.isdir(os.path.join(os.getcwd(),'maps'))):
        os.mkdir('maps')

    pkg_path = os.path.join(get_package_share_directory('kbot'))
 
    map_saver_params = os.path.join(get_package_share_directory('kbot'),'config','map_saver_params.yaml')
    
    # Create a map_saver node
    
    node_map_saver = Node(
        package='nav2_map_server',
        node_executable='map_saver_cli',
        parameters=[map_saver_params]
    )
    


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_map_saver
    ])
