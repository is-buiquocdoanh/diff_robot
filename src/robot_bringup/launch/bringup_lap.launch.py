from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # mecanum_joy
    mecanum_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mecanum_joy'),
                'launch',
                'mecanum_joy.launch.py'
            )
        )
    )

    # rf2o_laser_odometry (chạy trên laptop)
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rf2o_laser_odometry'),
                'launch',
                'rf2o_laser_odometry.launch.py'
            )
        )
    )

    return LaunchDescription([
        mecanum_joy_launch,
        rf2o_launch
    ])

 # save map
    #ros2 run nav2_map_server map_saver_cli -f map