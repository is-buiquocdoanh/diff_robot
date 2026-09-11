# diff_robot (v3)

Xây dựng lại bộ điều khiển robot 2 bánh vi sai (differential drive) từ đầu, dựa trên
kiến trúc driver của dự án mecanum trước đó.

## Cấu trúc

- `src/diff_drive_ros` — firmware ESP32 (PlatformIO): đọc frame lệnh từ ROS qua Serial/UART2,
  điều khiển động cơ qua BTS7960, đọc encoder qua PCNT. Xem `src/diff_drive_ros/README.md`.
- `src/a3_driver` — package ROS2:
  - `kinematic.py`: subscribe `/cmd_vel`, tính động học vi sai 2 bánh, publish `Velquery`.
  - `serial_bridge_node.py`: node duy nhất dùng chung 1 cổng USB cho cả 2 chiều —
    subscribe `Velquery` để đóng gói frame CAN-serial (14 byte:
    `0x2A | id(4) | data(8) | 0x23`) ghi xuống ESP32, đồng thời đọc dòng `IMU,...`
    ESP32 gửi lên và publish `sensor_msgs/Imu` trên `/imu/data`.
  - `odom_publisher_node.py`: publish `/odom` + TF `odom -> base_footprint` mà KHÔNG
    dùng encoder — lấy vận tốc (vx, wz) từ `/cmd_vel` (coi robot bám sát lệnh) và
    lấy hướng (yaw) từ `/imu/data`. Chỉ là odometry tạm/xấp xỉ, dùng cho tới khi
    driver có vòng điều khiển tốc độ (PID) + encoder feedback thật.
- `src/a3_description` — URDF/xacro của robot A3 (khung, bánh, RPLidar) + launch RViz/Gazebo.
- `src/a3_bringup` — launch bringup:
  - `lidar.launch.py`: chạy RPLidar A1M8 (`rplidar_ros`) và lọc bỏ các tia `/scan`
    gần hơn 30cm (4 trụ đỡ tầng trên che lidar) bằng `laser_filters` trước khi
    publish `/scan` cho slam_toolbox/nav2.
  - `odom.launch.py`: chạy `odom_publisher_node.py` (xem `a3_driver` ở trên).
  - `bringup.launch.py`: khởi chạy toàn bộ robot thật — `robot_state_publisher` +
    `joint_state_publisher` (URDF `a3_description`), `kinematic.py` +
    `serial_bridge_node.py`, `odom.launch.py`, `lidar.launch.py`. Dùng
    `ros2 launch a3_bringup bringup.launch.py` là đủ khởi động cả robot.
