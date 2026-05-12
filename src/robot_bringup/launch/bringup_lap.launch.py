from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # robot_joy
    robot_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_joy'),
                'launch',
                'robot_joy.launch.py'
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

    return LaunchDescription([
        robot_joy_launch,
        laser_filter_launch
    ])

 # save map
    #ros2 run nav2_map_server map_saver_cli -f map