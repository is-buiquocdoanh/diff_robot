from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # robot_joy
    robot_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_joy'),
                'launch',
                'joystick.launch.py'
            )
        )
    )

    # laser_filter
    laser_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('laser_filter'),
                'launch',
                'laser_filter.launch.py'
            )
        )
    )
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '0', '0', '0',   # vị trí
            '3.1415', '0', '0',  # roll pitch yaw (xoay 180 độ quanh Z)
            'base_link',
            'laser'
        ],
    )

    return LaunchDescription([
        robot_joy_launch,
        laser_filter_launch,
        tf2_node
    ])

 # save map
    #ros2 run nav2_map_server map_saver_cli -f map