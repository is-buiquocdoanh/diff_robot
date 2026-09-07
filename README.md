# diff_robot (v3)

Xây dựng lại bộ điều khiển robot 2 bánh vi sai (differential drive) từ đầu, dựa trên
kiến trúc driver của dự án mecanum trước đó.

## Cấu trúc

- `src/diff_drive_ros` — firmware ESP32 (PlatformIO): đọc frame lệnh từ ROS qua Serial/UART2,
  điều khiển động cơ qua BTS7960, đọc encoder qua PCNT. Xem `src/diff_drive_ros/README.md`.
- `src/a3_driver` — package ROS2:
  - `kinematic.py`: subscribe `/cmd_vel`, tính động học vi sai 2 bánh, publish `Velquery`.
  - `ros_serial_bridge.py`: subscribe `Velquery`, đóng gói frame CAN-serial (14 byte:
    `0x2A | id(4) | data(8) | 0x23`) và ghi ra serial cho ESP32.
