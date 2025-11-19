from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # robot description (khởi chạy mô hình robot trên rviz)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_simulation'),
                'launch',
                'display.launch.py'
            )
        )
    )
    # joystick (điều khiển robot bằng tay qua joystick)
    robot_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_joy'),
                'launch',
                'joystick.launch.py'
            )
        )
    )
    
    # kinematic (chạy các node chuyển đổi encoder sang joint_states và odom)
    robot_kinematic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_kinematic'),
                'launch',
                'kinematic.launch.py'
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
        robot_description_launch,
        robot_joy,
        robot_kinematic,
        # rf2o_launch
    ])

 # save map
    #ros2 run nav2_map_server map_saver_cli -f map