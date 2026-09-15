"""Robot A3 tự vẽ bản đồ bằng slam_toolbox.

CHỈ chạy slam_toolbox + RViz -- KHÔNG tự bringup robot. Bringup (driver ESP32,
EKF, RPLidar...) là tầng riêng chạy trước, SLAM chỉ "tiêu thụ" /scan + /odom + TF
do a3_bringup cung cấp:

    # Terminal 1: bringup robot thật
    ros2 launch a3_bringup bringup.launch.py
    # Terminal 2: SLAM
    ros2 launch atlas_slam slam.launch.py
    # Terminal 3: lái đi khắp khu vực cần vẽ map
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Tắt RViz nếu chỉ cần chạy nền: `ros2 launch atlas_slam slam.launch.py rviz:=false`

Sau khi đi hết khu vực, lưu bản đồ (map_frame phải đã xuất hiện trong TF, tức
slam_toolbox đã chạy được ít nhất vài giây và nhận dữ liệu /scan) -- định dạng
.pgm/.yaml, dùng cho AMCL (navigation.launch.py):

    ros2 run nav2_map_server map_saver_cli -f src/atlas_slam/maps/<tên_map>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_pkg_share = get_package_share_directory("atlas_slam")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Có mở RViz xem map trực tiếp hay không",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Dùng đồng hồ mô phỏng (chỉ true khi chạy Gazebo, xem a3_description/gazebo.launch.py)",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": os.path.join(slam_pkg_share, "config", "mapper_params.yaml"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(slam_pkg_share, "rviz", "slam_toolbox_default.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            rviz_arg,
            use_sim_time_arg,
            slam_toolbox_node,
            rviz_node,
        ]
    )
