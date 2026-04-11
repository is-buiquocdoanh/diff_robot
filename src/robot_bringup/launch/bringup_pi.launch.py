from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # serial esp32
    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_control'),
                'launch',
                'robot_control.launch.py'
            )
        )
    )

    # Rplidar A1M8
    rplidar_a1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            )
        )
    )


    return LaunchDescription([
        robot_control,
        rplidar_a1_launch,
    ])

    # save map
        #ros2 run nav2_map_server map_saver_cli -f map
