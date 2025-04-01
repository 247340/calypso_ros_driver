import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'calypso_ros_driver'
    config_file = os.path.join(
        get_package_share_directory(package_name), 
        'config', 
        'parameters.yaml'  
    )

    return LaunchDescription([
        Node(
            package=package_name,  
            executable='calypso_driver',  
            
            name='calypso_ros',  
            parameters=[config_file],
           
        ),
    ])
