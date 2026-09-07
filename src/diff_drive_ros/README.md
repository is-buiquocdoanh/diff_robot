diff_drive_ros — ESP32 differential-drive controller (BTS7960 / L298N)

This repository contains ESP32 firmware and helper code used to run a two-wheel differential robot with motor drivers such as the BTS7960 (H-bridge) or L298N. It also explains how the ROS2 side (`kinematic_serial.py`) sends commands to the ESP32 using a small CAN-like serial frame.

This README explains:
- board pinouts and wiring for BTS7960 and L298N
- serial protocol and data flow between ROS2 and ESP32
- how to build/flash and how to test
- useful parameters, safety and calibration notes
- ASCII debug commands available on the ESP32

---

1) Pin mappings used in the firmware (defaults)

These pin assignments come from `src/main_bts7960_test.cpp` and `main.cpp` (legacy). If you changed pins in the code, use the code mapping.

Encoders (quadrature)
- Left  A: GPIO 4
- Left  B: GPIO 15
- Right A: GPIO 23
- Right B: GPIO 22

IMU (BNO055, I2C) — see section 12 for full details
- SDA: GPIO 21
- SCL: GPIO 19 (moved off the ESP32 default GPIO22, which is already used by Right encoder B)

BTS7960 (current code uses LEDC PWM channels)
- Left forward (A):  PWM pin GPIO 25  (LEDC channel 0)
- Left backward (B): PWM pin GPIO 26  (LEDC channel 1)
- Right forward (A): PWM pin GPIO 32  (LEDC channel 2)
- Right backward (B): PWM pin GPIO 33  (LEDC channel 3)

Connect BTS7960
- VCC/ GND: 5V/GND
- R_IS/L_IS: bỏ qua
- R_EN/L_EN: 3.3V/3.3V
- R_PMW/L_PMW: pin 25/26 (đọng cơ trái)
- R_PMW/L_PMW: pin 32/33 (động cơ phải)

Serial ports
- USB (Serial) — used for debug/ASCII console and can also receive binary CAN-serial frames
- UART2 (Serial2) — default pins RX=16, TX=17 (used for a Raspberry Pi / external controller)

Notes: LEDC channel numbers are set up in code; the driver functions call ledcWrite(channel, duty) where duty is 0..255 (8-bit resolution).

2) Wiring guidance

BTS7960 wiring (recommended)
- BTS7960 has two control inputs per motor: RPWM and LPWM (or similar named). Each input is driven by a PWM signal; to move forward you enable one side, to reverse you drive the other side.
- Power lines:
  - Connect motor power supply (VM) to BTS7960 VM (check driver voltage and motor rating).
  - Connect BTS7960 GND to ESP32 GND and to motor battery negative.
- Control lines (example mapping to the firmware pins above):
  - Left motor RPWM <- GPIO 25 (L_PWM_PIN_A)
  - Left motor LPWM <- GPIO 26 (L_PWM_PIN_B)
  - Right motor RPWM <- GPIO 32 (R_PWM_PIN_A)
  - Right motor LPWM <- GPIO 33 (R_PWM_PIN_B)
- Enable pins and current sensing on BTS7960: connect as needed, set jumpers per module doc. Ensure grounds are common.
- Encoders: connect the encoder A/B signals to the ESP32 encoder pins and enable pull-ups (firmware sets INPUT_PULLUP). If encoder outputs are open-collector, pull-ups are required.

L298N wiring (alternative)
- L298N typically expects two direction pins and one enable (PWM) per motor or combinations of IN1/IN2 + ENA.
- Example mapping (adapt in firmware if you use L298N):
  - Left IN1 <- GPIO X (digital)
  - Left IN2 <- GPIO Y (digital)
  - Left ENA <- GPIO Z (PWM)
  - Right IN3 <- GPIO A (digital)
  - Right IN4 <- GPIO B (digital)
  - Right ENB <- GPIO C (PWM)
- Note: L298N drops more voltage and dissipates heat; choose driver based on motor current.
- If you use L298N, you must adapt `set_bts7960_pwm()` in the firmware to drive the direction pins and PWM on the enable pin.

3) Serial/CAN-serial frame format (used between ROS and ESP32)

We use a very small custom framing protocol to send command frames (this is *not* real CAN on the bus—only a frame format):

Frame layout (14 bytes total):
- Header: 0x2A (1 byte)
- ID   : 4 bytes (little-endian) — a 32-bit packet id (unused except for debug)
- Data : 8 bytes — data[0] .. data[7]
- Tail : 0x23 (1 byte)

Our convention for motor control data layout (current firmware):
- data[0] = left_dir   (0=stop, 1=forward, 2=backward)
- data[1] = left_pwm   (0..255)
- data[2] = right_dir  (0=stop,1=forward,2=backward)
- data[3] = right_pwm  (0..255)
- data[4..7] = reserved (0)

This packet can be sent on either USB serial (Serial) or UART2 (Serial2). The ESP32 code attempts to read packets from either port.

4) Data flow overview (ROS -> ESP32 -> motors / sensors -> ROS)

- ROS2 node `kinematic_serial.py` subscribes to `/cmd_vel`.
- It computes wheel target RPM using differential-drive kinematics, clips to `rpm_max` (parameter), maps RPM → PWM (0..255), builds the CAN-serial frame above and writes it to the configured serial port (USB or /dev/ttyUSBx).
- ESP32 (`main_bts7960_test.cpp`) reads the binary packet, extracts dir/pwm values and calls `set_bts7960_pwm()` which sets LEDC PWM channels accordingly.
- Encoder counts are read on the ESP32 using PCNT (pulse counter) and printed to Serial periodically for debugging (firmware prints RPM estimates). The ROS side can also request encoder frames if implemented.

5) ASCII debug console (USB Serial)

When connected to USB Serial (monitor) you can send simple ASCII commands. The firmware reads them when there is no binary packet received:
- L <pwm> <dir>   — set left motor (e.g. "L 150 1")
- R <pwm> <dir>   — set right motor
- S               — stop both motors
- D               — prints debug information (pin states, duty)

The firmware now uses a fixed buffer (no Arduino String usage) and a safety timeout so the serial console won't crash the parser.

6) Build / flash / monitor

From the `diff_drive_ros` folder (PlatformIO):

Build:

```bash
platformio run
```

Upload (auto detects upload_port from platformio.ini or set via env):

```bash
platformio run --target upload
```

Monitor serial output (USB):

```bash
platformio device monitor
# or specify port
platformio device monitor --port /dev/ttyUSB0
```

7) ROS side: `kinematic_serial.py` (quick usage)

- The ROS node is in your ROS package (path in your workspace). It has parameters:
  - `serial_port` (default: `/dev/ttyUSB0` or `/dev/ttyUSB1` depending on your system)
  - `baudrate` (default 115200)
  - `rate` (send rate, default 20 Hz)
  - `rpm_max` (the maximum wheel RPM the node will command — default configurable)
  - `log_rate` (how often to log PWM values)
  - `deadband_*`, `stop_timeout`, `min_pwm_threshold` for safety

Start node (example):

```bash
ros2 run robot_control kinematic_serial
```

Or run directly with Python (if not installed as a package):

```bash
python3 /path/to/kinematic_serial.py
```

Publish a velocity to test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 5
```

8) Quick serial test (Python) — send a single frame

If you want to test sending a frame manually, you can use a small Python snippet that writes the 14-byte frame via pyserial:

```python
import serial, struct
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
header = b'\x2a'
pid = struct.pack('<I', 1)
data = bytearray(8)
data[0] = 1   # left dir
data[1] = 150 # left pwm
data[2] = 1   # right dir
data[3] = 150 # right pwm
tail = b'\x23'
ser.write(header + pid + bytes(data) + tail)
ser.close()
```

Or adapt `test_send_loop.py` in your ROS package to continuously send frames for testing.

9) Safety & Troubleshooting

- Stalling / "stops accepting commands":
  - We added a PACKET_TIMEOUT on the ESP32: if no binary packet arrives within the timeout, firmware will stop the motors and print a safety message. This prevents runaway when the ROS node crashes.
  - The firmware also has a watchdog-style heartbeat and uses a non-blocking packet parser — this reduces the chances the serial parser gets stuck on garbage.
- Encoder issues:
  - If an encoder channel reads zero while the motor spins, check wiring and pull-ups. The firmware sets internal pull-ups; if your encoder is open-collector you must have pull-ups.
  - PCNT filter: if pulses are too short or noisy, adjust `pcnt_set_filter_value(...)` in the firmware.
- PWM drift (motors turning slowly when joystick released):
  - Use `deadband_linear`, `deadband_angular` and `min_pwm_threshold` params on the ROS node to prevent sending very small PWM values.
- Memory: avoid Arduino `String` in embedded code (firmware has been updated to use fixed buffers). If you see heap fragmentation you can print `ESP.getFreeHeap()` (firmware already logs free heap occasionally).

10) Calibration: measuring `rpm_physical_max` and setting `rpm_max`

- `rpm_physical_max`: measure by sending PWM 255 and reading the encoder RPM printed by the ESP32 (or logging encoder counts). That value is the motor's free-run RPM with your current gearbox and load.
- `rpm_max` (in ROS node): set to the maximum wheel RPM you want to allow (safety limit). Typically set `rpm_max` ≤ `rpm_physical_max`.

11) IMU (BNO055) integration — feasibility and setup

Có khả thi không? Có. BNO055 tự làm sensor fusion (accelerometer + gyroscope +
magnetometer) ngay trên chip và trả về quaternion đã hiệu chỉnh, nên ESP32 chỉ
cần đọc qua I2C và chuyển tiếp — không cần chạy thuật toán fusion (Madgwick/
Mahony...) trên ESP32. Đây là cách nhẹ nhàng nhất để có `orientation` phục vụ
`robot_localization`/EKF tính odometry (kết hợp với odometry bánh xe từ
encoder).

Kiến trúc dữ liệu:
- Kênh điều khiển động cơ (Pi → ESP32): giữ nguyên, dùng USB Serial, frame nhị
  phân CAN-serial như mô tả ở mục 3.
- Kênh IMU (ESP32 → Pi): dùng UART2 (Serial2, GPIO16/17), gửi dòng văn bản ASCII
  riêng, tách biệt hoàn toàn khỏi frame nhị phân để không bao giờ lẫn lộn hai
  luồng dữ liệu trên cùng một cổng.

Wiring BNO055 (I2C)
- VCC: 3.3V (board BNO055 thường chấp nhận 3.3V hoặc 5V, kiểm tra module cụ thể)
- GND: GND
- SDA: GPIO 21
- SCL: GPIO 19
- Lưu ý: GPIO22 (chân SCL mặc định của ESP32) đã bị chiếm bởi encoder phải
  (ENC_RIGHT_FRONT_B), nên I2C được remap sang GPIO19 bằng `Wire.begin(21, 19)`
  trong `setup()`. Nếu đổi chân encoder thì có thể đổi lại I2C về mặc định.
- Địa chỉ I2C mặc định BNO055 là 0x28 (chân ADR nối GND). Nếu ADR nối 3.3V thì
  địa chỉ là 0x29 — khi đó sửa `BNO055_ADDRESS_A` thành `BNO055_ADDRESS_B`
  trong `main.cpp`.
- Đặt IMU cách xa động cơ/dây dẫn công suất lớn (BTS7960, dây động cơ) vì
  nhiễu từ trường có thể ảnh hưởng magnetometer → làm trôi hướng (heading).

Frame gửi lên Pi (ASCII, kết thúc bằng `\n`), tần suất ~50Hz (mỗi 20ms):
```
IMU,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,cal_sys,cal_gyro,cal_accel,cal_mag
```
- `qw,qx,qy,qz`: quaternion orientation (đã fusion sẵn)
- `gx,gy,gz`: vận tốc góc (rad/s)
- `ax,ay,az`: gia tốc dài, đã trừ trọng lực (m/s²)
- `cal_sys,cal_gyro,cal_accel,cal_mag`: trạng thái hiệu chuẩn 0-3 (3 = đã hiệu
  chuẩn đầy đủ). Nên kiểm tra 4 giá trị này = 3 trước khi tin dữ liệu orientation.

Build/flash: thư viện `Adafruit BNO055` + `Adafruit Unified Sensor` đã được
thêm vào `platformio.ini` (`lib_deps`), PlatformIO sẽ tự tải khi build.

Phía ROS2: node `a3_driver/scripts/imu_serial_node.py` mở cổng UART2 (mặc định
tham số `serial_port` là `/dev/ttyAMA0` — đổi theo cổng UART thật của Raspberry
Pi bạn dùng để nối với GPIO16/17 của ESP32), parse dòng `IMU,...` và publish
`sensor_msgs/Imu` trên topic `/imu/data`.

Chạy thử:
```bash
ros2 run a3_driver imu_serial_node.py --ros-args -p serial_port:=/dev/ttyAMA0
ros2 topic echo /imu/data
```

Kiểm tra hiệu chuẩn (calibration): BNO055 cần được "học" hiệu chuẩn mỗi lần
mất nguồn hoàn toàn (offset không lưu tự động trừ khi bạn tự đọc/ghi lại offset
qua `bno.getSensorOffsets()`/`setSensorOffsets()` — chưa làm trong bản này).
Để hiệu chuẩn nhanh: xoay robot chậm quanh cả 3 trục vài vòng (gyro),
để yên vài giây ở nhiều tư thế khác nhau (accel), và xoay hình số 8 trong
không khí (magnetometer) cho đến khi node log không còn cảnh báo
"Calibration ... (3=full)".

Việc còn lại để có odometry hoàn chỉnh (chưa nằm trong yêu cầu hiện tại):
- Chạy `robot_localization` (hoặc tương tự) với 2 nguồn: `/imu/data` (orientation
  + angular velocity) và odometry tính từ encoder bánh xe, fusion ra `/odom`.
- Căn chỉnh trục IMU khớp với khung `base_link` của robot (IMU có thể lắp lệch
  hướng vật lý so với "trước" của robot — cần bù bằng `static_transform_publisher`
  hoặc offset quaternion trong code nếu lắp lệch).

12) If you want me to (choose one):
- Add `rpm_physical_max` param and change mapping to clip by `rpm_max` but calculate pwm using `rpm_physical_max` (recommended for more accurate mapping).
- Create a calibration routine that sends PWM=255, reads RPM back from ESP32 and writes `rpm_physical_max` to a param or file.
- Archive or remove old unused CAN/Ctrl files so the repo is smaller (I can create a branch and move them into an `archive/` folder).

---

If you want, I can now:
- add a short calibration routine to `kinematic_serial.py`, or
- add a `README` entry describing exact commands to measure `rpm_physical_max` using your current firmware logs, or
- create a small convenience script to perform the calibration automatically.

Which would you like next?