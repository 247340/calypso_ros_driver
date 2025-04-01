import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'calypso_ros_driver'
    # Cesta k YAML souboru s parametry
    config_file = os.path.join(
        get_package_share_directory(package_name),  # Získání sdíleného adresáře balíčku
        'config',  # Předpokládá se, že je ve složce 'config'
        'parameters.yaml'  # Název YAML souboru s parametry
    )

    return LaunchDescription([
        # Node pro spouštění uzlu, který bude publikovat zprávu WindSpeed
        Node(
            package=package_name,  # Název balíčku
            executable='calypso_driver',  # Název spustitelného souboru (node)
            
            name='calypso_ros',  # Název uzlu
            parameters=[config_file],
           
        ),
    ])
