# diff_drive_ros — ESP32 Differential-Drive Controller (BTS7960 / L298N)

Firmware ESP32 + code hỗ trợ để điều khiển robot 2 bánh vi sai, dùng driver
động cơ BTS7960 (H-bridge) hoặc L298N. Tài liệu này cũng mô tả cách phía ROS2
(`kinematic.py` + `serial_bridge_node.py`) gửi lệnh xuống ESP32 và nhận dữ
liệu IMU ngược lại, tất cả qua **1 cổng USB** dùng chung.

## Mục lục

1. [Sơ đồ chân (pinout)](#1-sơ-đồ-chân-pinout)
2. [Đấu dây](#2-đấu-dây)
3. [Giao thức frame Serial/CAN](#3-giao-thức-frame-serialcan)
4. [Luồng dữ liệu](#4-luồng-dữ-liệu)
5. [Console debug ASCII](#5-console-debug-ascii)
6. [Build / flash / monitor](#6-build--flash--monitor)
7. [Phía ROS2: `kinematic.py` + `serial_bridge_node.py`](#7-phía-ros2-kinematicpy--serial_bridge_nodepy)
8. [Test gửi frame thủ công (Python)](#8-test-gửi-frame-thủ-công-python)
9. [An toàn & xử lý sự cố](#9-an-toàn--xử-lý-sự-cố)
10. [Hiệu chuẩn `rpm_physical_max` / `rpm_max`](#10-hiệu-chuẩn-rpm_physical_max--rpm_max)
11. [Tích hợp IMU (BNO055)](#11-tích-hợp-imu-bno055)

---

## 1. Sơ đồ chân (pinout)

Các chân dưới đây lấy từ `src/main_bts7960_test.cpp` (và `main.cpp` — bản cũ).
Nếu bạn đổi chân trong code, code là nguồn chính xác nhất.

**Encoder (quadrature)**

| Encoder | Pin |
|---|---|
| Left A | GPIO 4 |
| Left B | GPIO 15 |
| Right A | GPIO 23 |
| Right B | GPIO 22 |

**IMU (BNO055, I2C)** — chi tiết đầy đủ ở [mục 11](#11-tích-hợp-imu-bno055)

| I2C | Pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 19 (dời khỏi GPIO 22 mặc định vì đã dùng cho Right encoder B) |

**BTS7960** (dùng LEDC PWM)

| Tín hiệu | Pin | LEDC channel |
|---|---|---|
| Left forward (A) | GPIO 25 | 0 |
| Left backward (B) | GPIO 26 | 1 |
| Right forward (A) | GPIO 32 | 2 |
| Right backward (B) | GPIO 33 | 3 |

Đấu nối BTS7960:
- VCC/GND: 5V/GND
- R_IS/L_IS: bỏ qua
- R_EN/L_EN: 3.3V/3.3V
- Động cơ trái → R_PWM/L_PWM = GPIO 25/26
- Động cơ phải → R_PWM/L_PWM = GPIO 32/33

**Serial**

| Cổng | Vai trò |
|---|---|
| USB (`Serial`) | Kênh chính: nhận frame nhị phân lệnh động cơ, gửi dòng debug + IMU (`IMU,...`), console ASCII |
| UART2 (`Serial2`) | Mặc định RX=16, TX=17 — cổng nhận lệnh động cơ dự phòng, không bắt buộc nếu chỉ dùng 1 cổng USB |

> LEDC channel được cấu hình sẵn trong code; `ledcWrite(channel, duty)` nhận
> duty 0..255 (độ phân giải 8-bit).

## 2. Đấu dây

**BTS7960 (khuyến nghị)**

- Mỗi động cơ có 2 chân điều khiển RPWM/LPWM: bật một bên để tiến, bên còn lại để lùi.
- Nguồn: VM động cơ → BTS7960 VM (kiểm tra điện áp/dòng của driver và động cơ); GND BTS7960 → GND ESP32 và cực âm pin động cơ.
- Điều khiển (theo pinout ở trên): Left RPWM←GPIO25, Left LPWM←GPIO26, Right RPWM←GPIO32, Right LPWM←GPIO33.
- Enable pin / current sensing: đấu theo tài liệu module, đảm bảo GND chung.
- Encoder: nối A/B vào chân encoder ESP32, bật pull-up (firmware đã set `INPUT_PULLUP`) — bắt buộc nếu encoder dạng open-collector.

**L298N (phương án thay thế)**

- Thường cần 2 chân direction + 1 chân enable (PWM) mỗi động cơ (IN1/IN2 + ENA...).
- Ví dụ mapping (tự chọn chân, sửa lại trong firmware):
  - Left: IN1←GPIO X, IN2←GPIO Y, ENA←GPIO Z (PWM)
  - Right: IN3←GPIO A, IN4←GPIO B, ENB←GPIO C (PWM)
- L298N sụt áp nhiều hơn và tỏa nhiệt hơn — chọn driver theo dòng điện động cơ.
- Nếu dùng L298N, cần sửa `set_bts7960_pwm()` trong firmware để điều khiển chân direction + PWM enable.

## 3. Giao thức frame Serial/CAN

Frame nhị phân tự định nghĩa (không phải CAN bus thật, chỉ mượn cấu trúc), tổng
**14 byte**:

| Header | ID | Data | Tail |
|---|---|---|---|
| `0x2A` (1 byte) | 4 byte, little-endian (packet id, chỉ dùng debug) | 8 byte | `0x23` (1 byte) |

Layout `data[0..7]` cho điều khiển động cơ:

| Byte | Ý nghĩa |
|---|---|
| `data[0]` | left_dir (0=stop, 1=forward, 2=backward) |
| `data[1]` | left_pwm (0..255) |
| `data[2]` | right_dir (0=stop, 1=forward, 2=backward) |
| `data[3]` | right_pwm (0..255) |
| `data[4..7]` | reserved (0) |

Frame có thể gửi qua USB Serial hoặc UART2 — ESP32 đọc song song cả hai cổng.

## 4. Luồng dữ liệu

```
ROS2 (/cmd_vel) → kinematic.py → /vel_query → serial_bridge_node.py → frame CAN-serial ┐
                                                                                        ▼
                                                                    USB Serial (1 cổng, 2 chiều)
                                                                                        │
                          ESP32 (main.cpp): BTS7960 ← lệnh động cơ  ◄──────────────────┘
                                            PCNT đọc encoder → log RPM (debug)
                                            BNO055 → dòng "IMU,..." ─────────────────┐
                                                                                      ▼
                                                          serial_bridge_node.py → /imu/data
```

1. `kinematic.py` subscribe `/cmd_vel`, tính RPM mục tiêu mỗi bánh theo động học vi sai, clip theo `rpm_max`, map RPM → PWM (0..255), publish `Velquery`.
2. `serial_bridge_node.py` subscribe `Velquery`, đóng gói frame ở [mục 3](#3-giao-thức-frame-serialcan) rồi ghi xuống ESP32 qua **cùng 1 cổng USB**; đồng thời đọc mọi dòng text ESP32 gửi lên trên cổng đó, dòng nào bắt đầu bằng `IMU,` thì parse và publish `sensor_msgs/Imu` trên `/imu/data` (các dòng debug khác bị bỏ qua). Xem chi tiết ở [mục 7](#7-phía-ros2-kinematicpy--serial_bridge_nodepy) và [mục 11](#11-tích-hợp-imu-bno055).
3. `main.cpp` đọc packet nhị phân, tách dir/pwm, gọi `set_bts7960_pwm()` để set các kênh LEDC; đồng thời stream dòng IMU ra cùng cổng ở nhịp ~50 Hz.
4. Encoder được đọc bằng PCNT (pulse counter), in định kỳ ra Serial để debug (ước lượng RPM) — dữ liệu debug này không được ROS đọc lại, chỉ để xem qua `platformio device monitor`.

> Vì cả điều khiển động cơ và IMU dùng chung 1 cổng, **chỉ được có duy nhất 1
> process mở cổng serial này** (`serial_bridge_node.py`) — hai process cùng mở
> một device sẽ khiến byte đọc về bị chia ngẫu nhiên và hỏng cả hai luồng.

## 5. Console debug ASCII (USB Serial)

Khi mở monitor USB Serial, có thể gửi lệnh ASCII (firmware đọc khi không có packet nhị phân):

| Lệnh | Ý nghĩa |
|---|---|
| `L <pwm> <dir>` | set động cơ trái, vd `L 150 1` |
| `R <pwm> <dir>` | set động cơ phải |
| `S` | dừng cả hai động cơ |
| `D` | in thông tin debug (trạng thái chân, duty) |

Firmware dùng buffer cố định (không dùng Arduino `String`) và có timeout an toàn để parser không bị treo.

## 6. Build / flash / monitor

Chạy trong thư mục `diff_drive_ros` (PlatformIO):

```bash
# Build
platformio run

# Upload (tự nhận upload_port từ platformio.ini, hoặc set qua env)
platformio run --target upload

# Monitor
platformio device monitor
platformio device monitor --port /dev/ttyUSB0   # hoặc chỉ định cổng
```

## 7. Phía ROS2: `kinematic.py` + `serial_bridge_node.py`

Hai node phối hợp qua topic nội bộ `/vel_query` (msg `Velquery`):

**`kinematic.py`** — subscribe `/cmd_vel`, tính động học vi sai, publish `Velquery`.

| Tham số | Ý nghĩa |
|---|---|
| `rate` | tần suất publish, mặc định 100 Hz |
| `rpm_max` | RPM tối đa được phép gửi |
| `deadband_*`, `stop_timeout`, `min_pwm_threshold`... | các tham số an toàn (xem code để biết danh sách đầy đủ) |

**`serial_bridge_node.py`** — node duy nhất mở cổng serial: ghi frame lệnh động cơ xuống ESP32 và đọc dòng `IMU,...` từ ESP32 để publish `/imu/data`.

| Tham số | Ý nghĩa |
|---|---|
| `serial_port` | mặc định `/dev/esp32` (khuyến nghị tạo udev symlink cố định thay vì `/dev/ttyUSBx` hay đổi số) |
| `baudrate` | mặc định 115200 |
| `reconnect_interval` | giây giữa các lần thử kết nối lại khi mất cổng, mặc định 1.5 |
| `imu_frame_id` | frame_id gắn vào `sensor_msgs/Imu`, mặc định `imu_link` |
| `orientation_stddev`, `angular_velocity_stddev`, `linear_acceleration_stddev` | độ lệch chuẩn dùng để điền covariance cho `/imu/data` |

Chạy:

```bash
ros2 run a3_driver kinematic.py
ros2 run a3_driver serial_bridge_node.py --ros-args -p serial_port:=/dev/esp32
```

Test gửi vận tốc và xem IMU:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 5
ros2 topic echo /imu/data
```

## 8. Test gửi frame thủ công (Python)

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

Có thể dùng/sửa `test_send_loop.py` để gửi liên tục phục vụ test.

## 9. An toàn & xử lý sự cố

**Robot ngừng nhận lệnh / "đứng hình"**
- Firmware có `PACKET_TIMEOUT`: nếu không nhận packet nhị phân trong thời gian timeout, motor tự dừng và in cảnh báo — tránh chạy loạn khi node ROS crash.
- Có cơ chế heartbeat kiểu watchdog và parser non-blocking để tránh kẹt khi nhận dữ liệu rác.

**Encoder lỗi**
- Nếu một kênh encoder đọc về 0 dù động cơ vẫn quay, kiểm tra dây và pull-up. Firmware đã bật pull-up nội bộ; nếu encoder dạng open-collector bắt buộc phải có pull-up ngoài.
- Nếu xung quá ngắn/nhiễu, chỉnh `pcnt_set_filter_value(...)` trong firmware.

**PWM trôi (động cơ vẫn quay chậm khi thả joystick)**
- Dùng `deadband_linear`, `deadband_angular`, `min_pwm_threshold` ở phía ROS để tránh gửi giá trị PWM quá nhỏ.

**Bộ nhớ**
- Tránh dùng Arduino `String` trong code nhúng (firmware đã chuyển sang buffer cố định). Có thể theo dõi `ESP.getFreeHeap()` nếu nghi ngờ phân mảnh heap (firmware đã log định kỳ).

## 10. Hiệu chuẩn `rpm_physical_max` / `rpm_max`

- `rpm_physical_max`: đo bằng cách set PWM=255 và đọc RPM từ log encoder trên ESP32 — đây là RPM chạy không tải thực tế với hộp số/tải hiện tại.
- `rpm_max` (tham số ROS): giới hạn an toàn RPM tối đa cho phép, nên đặt `rpm_max` ≤ `rpm_physical_max`.

## 11. Tích hợp IMU (BNO055)

BNO055 tự làm sensor fusion (accelerometer + gyroscope + magnetometer) ngay
trên chip và trả về quaternion đã hiệu chỉnh, nên ESP32 chỉ cần đọc qua I2C và
forward — không cần chạy thuật toán fusion (Madgwick/Mahony...) trên ESP32.
Đây là cách nhẹ nhàng nhất để có `orientation` phục vụ `robot_localization`/EKF
tính odometry (kết hợp với odometry bánh xe từ encoder).

**Kiến trúc dữ liệu**
- Kênh điều khiển động cơ (Pi → ESP32): USB Serial, frame nhị phân CAN-serial ([mục 3](#3-giao-thức-frame-serialcan)).
- Kênh IMU (ESP32 → Pi): dùng **cùng cổng USB Serial**, gửi dòng văn bản ASCII riêng biệt (`IMU,...\n`). Hai chiều không lẫn nhau vì Pi → ESP32 luôn là nhị phân, còn ESP32 → Pi luôn là text theo dòng — phía Pi chỉ cần lọc dòng bắt đầu bằng `IMU,`, các dòng debug khác của ESP32 bị bỏ qua.
- UART2 (Serial2, GPIO16/17) trên ESP32 vẫn còn trong code như một cổng nhận lệnh động cơ dự phòng, nhưng không dùng cho IMU nữa và không bắt buộc phải đấu dây nếu chỉ dùng 1 cổng USB.

**Đấu dây BNO055 (I2C)**

| Chân | Nối tới |
|---|---|
| VCC | 3.3V (kiểm tra module cụ thể, một số hỗ trợ cả 5V) |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 19 |

- GPIO22 (chân SCL mặc định của ESP32) đã bị chiếm bởi encoder phải (`ENC_RIGHT_FRONT_B`), nên I2C được remap sang GPIO19 bằng `Wire.begin(21, 19)` trong `setup()`. Nếu đổi chân encoder, có thể trả I2C về mặc định.
- Địa chỉ I2C mặc định là `0x28` (chân ADR nối GND). Nếu ADR nối 3.3V thì địa chỉ là `0x29` — sửa `BNO055_ADDRESS_A` thành `BNO055_ADDRESS_B` trong `main.cpp`.
- Đặt IMU cách xa động cơ/dây công suất lớn (BTS7960, dây động cơ) vì nhiễu từ trường có thể làm trôi hướng (heading) đọc từ magnetometer.

**Frame gửi lên Pi** (ASCII, kết thúc `\n`, tần suất ~50 Hz / 20 ms):

```
IMU,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,cal_sys,cal_gyro,cal_accel,cal_mag
```

| Trường | Ý nghĩa |
|---|---|
| `qw,qx,qy,qz` | quaternion orientation (đã fusion sẵn) |
| `gx,gy,gz` | vận tốc góc (rad/s) |
| `ax,ay,az` | gia tốc dài, đã trừ trọng lực (m/s²) |
| `cal_sys,cal_gyro,cal_accel,cal_mag` | trạng thái hiệu chuẩn 0-3 (3 = đầy đủ) — nên kiểm tra cả 4 giá trị = 3 trước khi tin dữ liệu orientation |

Build/flash: thư viện `Adafruit BNO055` + `Adafruit Unified Sensor` đã được
thêm vào `platformio.ini` (`lib_deps`), PlatformIO tự tải khi build.

Phía ROS2: `serial_bridge_node.py` (cùng node ghi lệnh động cơ) đọc dòng
`IMU,...` trên cổng USB và publish `sensor_msgs/Imu` trên `/imu/data` — xem
tham số và cách chạy ở [mục 7](#7-phía-ros2-kinematicpy--serial_bridge_nodepy).

**Hiệu chuẩn (calibration)**: BNO055 cần "học" lại hiệu chuẩn mỗi lần mất
nguồn hoàn toàn (offset không tự lưu, trừ khi tự đọc/ghi qua
`bno.getSensorOffsets()`/`setSensorOffsets()` — chưa làm trong bản này). Cách
hiệu chuẩn nhanh: xoay robot chậm quanh cả 3 trục vài vòng (gyro), để yên vài
giây ở nhiều tư thế khác nhau (accel), xoay hình số 8 trong không khí
(magnetometer) — cho đến khi node không còn cảnh báo "Calibration ... (3=full)".

**Việc còn lại để có odometry hoàn chỉnh** (chưa nằm trong scope hiện tại):
- Chạy `robot_localization` (hoặc tương tự) fusion `/imu/data` (orientation + angular velocity) với odometry tính từ encoder bánh xe, ra `/odom`.
- Căn chỉnh trục IMU khớp với khung `base_link` của robot (nếu IMU lắp lệch hướng vật lý so với "trước" của robot, cần bù bằng `static_transform_publisher` hoặc offset quaternion trong code).
