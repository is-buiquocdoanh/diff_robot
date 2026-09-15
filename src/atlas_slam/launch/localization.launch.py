"""Định vị (localization) cho robot A3 thật: map_server + AMCL.

Chỉ dùng AMCL (particle filter) -- không còn tùy chọn slam_toolbox localization
(chế độ đó cần file config/params riêng chưa từng tồn tại trong package này).
Tách khỏi navigation.launch.py để 2 tầng độc lập: định vị (biết robot đang ở
đâu trên map có sẵn) và điều hướng (lập kế hoạch + bám theo path) có thể chạy/
config riêng mà không đụng vào nhau.

    ros2 launch atlas_slam localization.launch.py map:=/đường/dẫn/map.yaml

Thường được navigation.launch.py include sẵn -- không cần gọi tay trừ khi chỉ
cần định vị (không cần Nav2 planner/controller).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    slam_pkg_share = get_package_share_directory("atlas_slam")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(slam_pkg_share, "maps", "maze_map.yaml"),
        description="Đường dẫn file .yaml của bản đồ đã lưu (map_saver_cli).",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(slam_pkg_share, "config", "atlas_localization.yaml"),
        description="File params cho map_server + amcl",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Dùng đồng hồ mô phỏng (chỉ true khi chạy Gazebo, xem a3_description/gazebo.launch.py)",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value="True", description="Tự động chuyển các lifecycle node sang active"
    )

    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    # RewrittenYaml tiêm use_sim_time/autostart/yaml_filename vào đúng chỗ trong params_file
    # lúc launch, thay vì phải sửa tay file config mỗi khi đổi map.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("params_file"),
            root_key="",
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "yaml_filename": map_yaml_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    # /tf, /tf_static remap về dạng tương đối -- convention chuẩn của nav2_bringup.
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    localization_nodes = GroupAction(
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["map_server", "amcl"],
                    }
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            map_arg,
            params_file_arg,
            use_sim_time_arg,
            autostart_arg,
            localization_nodes,
        ]
    )
