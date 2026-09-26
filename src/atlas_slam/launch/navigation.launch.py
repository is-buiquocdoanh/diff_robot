"""Nav2 (costmap/planner/controller/BT) cho robot A3 thật + AMCL localization.

Chỉ chạy tầng Nav2 core (điều hướng) -- KHÔNG tự bringup robot. Bringup (driver
ESP32, EKF, RPLidar...) là tầng riêng chạy trước, xem a3_bringup:

    # Terminal 1: bringup robot thật
    ros2 launch a3_bringup bringup.launch.py
    # Terminal 2: định vị (AMCL) + Nav2
    ros2 launch atlas_slam navigation.launch.py
    # Đặt "2D Pose Estimate" trong RViz để AMCL biết vị trí ban đầu, sau đó
    # dùng "Nav2 Goal" trong RViz để gửi điểm đến

Định vị luôn dùng AMCL (xem launch/localization.launch.py, được include sẵn ở
dưới) -- map lưu bằng map_saver_cli (.pgm + .yaml, xem maps/README.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    slam_pkg_share = get_package_share_directory("atlas_slam")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(slam_pkg_share, "maps", "maze_map.yaml"),
        description="Đường dẫn file .yaml của bản đồ đã lưu (map_saver_cli), truyền cho localization.launch.py",
    )
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="dwb",
        description="Thuật toán controller: dwb | rpp | mppi (ứng với config/nav2_params_<controller>.yaml)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Dùng đồng hồ mô phỏng (chỉ true khi chạy Gazebo, xem a3_description/gazebo.launch.py)",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value="True", description="Tự động chuyển các lifecycle node sang active"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Có mở RViz (dùng config mặc định) hay không",
    )
    collision_monitor_arg = DeclareLaunchArgument(
        "collision_monitor",
        default_value="true",
        description="Bật Collision Monitor (chặn /cmd_vel dựa thẳng vào /scan, độc lập controller/costmap)",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    params_file = PathJoinSubstitution(
        [slam_pkg_share, "config", ["nav2_params_", LaunchConfiguration("controller"), ".yaml"]]
    )

    # RewrittenYaml tiêm use_sim_time/autostart vào đúng chỗ trong file params lúc launch,
    # thay vì phải sửa tay 3 file config mỗi khi bật/tắt sim_time.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    configured_collision_params = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(slam_pkg_share, "config", "collision_monitor_params.yaml"),
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Định vị (map_server + AMCL) -- luôn dùng AMCL, xem localization.launch.py.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    # ============================================================
    # Navigation core: costmap/planner/controller/BT
    # ============================================================
    navigation_nodes = GroupAction(
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                # velocity_smoother nhận cmd_vel_nav (do controller_server phát ra phía trên) -> lọc
                # mượt -> phát ra cmd_vel_raw, KHÔNG phải /cmd_vel thật -- Collision Monitor bên dưới
                # mới là chặng chặn cuối cùng trước khi lệnh xuống robot. a3_bringup hiện chưa có
                # twist_mux -- nếu sau này thêm teleop chạy song song, chèn twist_mux giữa
                # velocity_smoother và collision_monitor để làm trọng tài nguồn lệnh.
                remappings=remappings + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel_raw")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": [
                            "controller_server",
                            "smoother_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "waypoint_follower",
                            "velocity_smoother",
                        ],
                    }
                ],
            ),
        ]
    )

    # ============================================================
    # Collision Monitor: chặn/giảm tốc /cmd_vel dựa thẳng vào /scan (nav2_collision_monitor),
    # tách khỏi navigation_nodes ở trên -- lifecycle manager riêng để bật/tắt độc lập controller/BT
    # (vd. tắt tạm khi cần lùi sát vật cản để đỗ, mà không đụng tới cả cụm điều hướng).
    # ============================================================
    collision_monitor_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("collision_monitor")),
        actions=[
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[configured_collision_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_collision_monitor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["collision_monitor"],
                    }
                ],
            ),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(slam_pkg_share, "rviz", "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            map_arg,
            controller_arg,
            use_sim_time_arg,
            autostart_arg,
            rviz_arg,
            collision_monitor_arg,
            localization,
            navigation_nodes,
            collision_monitor_group,
            rviz_node,
        ]
    )
