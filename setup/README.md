# setup — cài đặt môi trường

`install_dependencies.sh` cài toàn bộ phụ thuộc để build và chạy dự án trên **Ubuntu 22.04 + ROS 2 Humble**.
Chạy từ thư mục gốc workspace (chạy từ đâu cũng được), không cần `sudo` — script tự gọi khi cần.

```bash
./setup/install_dependencies.sh                     # cài đủ để build + chạy robot và web
./setup/install_dependencies.sh --with-platformio   # thêm PlatformIO (nạp firmware ESP32 từ máy này)
./setup/install_dependencies.sh --with-mesh-tools   # thêm open3d cho a3_description/scripts (tuỳ chọn)
./setup/install_dependencies.sh --dry-run           # chỉ in các lệnh sẽ chạy, không cài gì
```

Sau đó build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a3_web web.launch.py        # mở http://localhost:8080
```

## Script làm gì

| Bước | Nội dung |
|---|---|
| 1 | Nếu chưa có ROS 2 Humble: thêm kho apt và cài `ros-humble-desktop` |
| 2 | Công cụ build: `build-essential`, `cmake`, `git`, `pip`, `colcon`, `rosdep`, `vcstool` |
| 3 | Gói của dự án (apt): `rplidar-ros`, `laser-filters`, `robot-localization`, `xacro`, `joint-state-publisher(-gui)`, `robot-state-publisher`, `rviz2`, `teleop-twist-keyboard`, `gazebo-ros-pkgs`, `slam-toolbox`, `navigation2`, `nav2-bringup` + các controller DWB / RPP / MPPI, `eigen3-cmake-module`, `libeigen3-dev`, `python3-serial`, `python3-aiohttp`, `numpy`, `yaml`, `pytest` |
| 4 | `rosdep install --from-paths src` để bắt phần còn sót theo `package.xml` (báo thiếu chỉ cảnh báo, không dừng) |
| 5 | Thêm user vào nhóm `dialout` để đọc `/dev/ttyUSB*` (ESP32, RPLidar) |
| 6 | Tuỳ chọn: PlatformIO, open3d (`pip --user`) |

Chạy lại nhiều lần được — gói đã có sẽ được bỏ qua.

## Lưu ý

- Vừa được thêm vào nhóm `dialout` thì cần **đăng xuất/đăng nhập lại** mới có hiệu lực.
- `a3_driver` mặc định mở `/dev/esp32`. Hoặc tạo udev symlink cho ESP32 (cần vendor/product ID của board bạn),
  hoặc chạy với `serial_port:=/dev/ttyUSB0` (xem `src/diff_drive_ros/README.md`).
- Máy không phải Ubuntu 22.04 (jammy): script cảnh báo và hỏi trước khi tiếp tục.
- PlatformIO cài vào `~/.local/bin` — đảm bảo thư mục này có trong `PATH` để dùng lệnh `pio`.
