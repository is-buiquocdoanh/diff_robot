from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # diff_serial_bridge
    diff_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('diff_serial_bridge'),
                'launch',
                'diff_driver.launch.py'
            )
        )
    )
    
    # ydlidar x3pro
    ylidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            )
        )
    )

    return LaunchDescription([
        diff_driver,
        ylidar_launch,
    ])

 # save map
    #ros2 run nav2_map_server map_saver_cli -f map