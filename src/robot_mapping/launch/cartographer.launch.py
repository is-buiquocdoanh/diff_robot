from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cartographer_pkg = get_package_share_directory('robot_mapping')
    rviz_config = os.path.join(cartographer_pkg, 'rviz', 'cartographer_mapping.rviz')

    configuration_directory = os.path.join(cartographer_pkg, 'config')
    configuration_basename = 'lds_2d.lua'  # tên file lua

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                # ('/odom', '/odom_rf2o'),
                ('/scan', '/scan')
            ],
            parameters=[{'use_sim_time': False}]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05,
                'publish_period_sec': 1.0
            }]
        ),
        
        Node (
            package= 'rviz2',
            executable= 'rviz2',
            name= 'rviz2',
            output= 'screen',
            arguments= ['-d', rviz_config],
            parameters= [{'use_sim_time': False}]
        )
    ])
