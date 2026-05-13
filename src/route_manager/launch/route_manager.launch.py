from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory('route_manager')
    route_file = os.path.join(pkg_dir, 'config', 'route.yaml')

    return LaunchDescription([
        Node(
            package='route_manager',
            executable='route_manager',
            name='route_manager',
            output='screen',
            parameters=[{'route_file': route_file}]
        )
    ])