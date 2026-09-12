import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('a3_bringup'), 'config', 'ekf.yaml'
    )

    return LaunchDescription([
        # ekf_node (robot_localization) mặc định publish odometry đã lọc ra
        # topic `odometry/filtered` - remap về /odom cho khớp với những gì
        # slam_toolbox/nav2 mong đợi, đồng thời tự broadcast TF odom ->
        # base_footprint (publish_tf: true trong ekf.yaml).
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', 'odom')],
        ),
    ])
