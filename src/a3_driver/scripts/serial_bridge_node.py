#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 node: cầu nối serial dùng CHUNG 1 cổng USB cho cả 2 chiều với ESP32.

  - Pi -> ESP32: lệnh động cơ (subscribe `/vel_query`), đóng gói frame nhị phân
    CAN-serial 14 byte (0x2A | id(4) | data(8) | 0x23) và ghi ra serial.
  - ESP32 -> Pi: đọc mọi dòng text ESP32 gửi về; dòng nào bắt đầu bằng "IMU,"
    được parse và publish `sensor_msgs/Imu` trên `/imu/data`, các dòng debug/log
    khác của ESP32 bị bỏ qua.

Gộp từ ros_serial_bridge.py + imu_serial_node.py (trước đây IMU dùng UART2 dùng
riêng qua /dev/ttyAMA0). Hai chiều dữ liệu không đụng nhau vì Pi -> ESP32 luôn
là nhị phân, ESP32 -> Pi luôn là text theo dòng.

CHỈ được có một node mở serial_port này. Nếu 2 process cùng mở một device thì
byte đọc về sẽ bị chia ngẫu nhiên giữa 2 process và hỏng cả 2 luồng.
"""
import sys
import time

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
import serial

from sensor_msgs.msg import Imu
from a3_driver.msg import Velquery


class SerialBridgeNode(LifecycleNode):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.get_logger().warn("ROS 2 Node serial_bridge_node Initialized!")
        self.killnode = 0

        self.declare_parameters('', [
            ('serial_port', "/dev/esp32"),
            ('baudrate', 115200),
            ('reconnect_interval', 1.5),
            ('imu_frame_id', 'imu_link'),
            # phương sai (variance) mặc định cho các trường IMU - chỉnh lại sau
            # khi đã đánh giá độ nhiễu thực tế của cảm biến
            ('orientation_stddev', 0.02),
            ('angular_velocity_stddev', 0.02),
            ('linear_acceleration_stddev', 0.05),
        ])

        self.PORT = self.get_parameter("serial_port").value
        self.BAUDRATE = self.get_parameter("baudrate").value
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.imu_frame_id = self.get_parameter('imu_frame_id').value

        ori_var = float(self.get_parameter('orientation_stddev').value) ** 2
        ang_var = float(self.get_parameter('angular_velocity_stddev').value) ** 2
        acc_var = float(self.get_parameter('linear_acceleration_stddev').value) ** 2
        self.orientation_cov = [ori_var, 0.0, 0.0, 0.0, ori_var, 0.0, 0.0, 0.0, ori_var]
        self.angular_velocity_cov = [ang_var, 0.0, 0.0, 0.0, ang_var, 0.0, 0.0, 0.0, ang_var]
        self.linear_acceleration_cov = [acc_var, 0.0, 0.0, 0.0, acc_var, 0.0, 0.0, 0.0, acc_var]

        # -- Subscriber lệnh động cơ / Publisher IMU
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.sub_velquery = self.create_subscription(
            Velquery,
            'vel_query',
            self.callback_velquery,
            10
        )

        self.ser = None
        self.rx_buf = bytearray()
        self.last_cal_log = 0.0
        self.last_reconnect_attempt = time.time()
        self.connect_serial()

        # Đọc + ghi serial đều non-blocking trong timer trên cùng 1 thread, để
        # tránh cần khóa (lock) giữa đọc và ghi trên cùng một cổng.
        self.create_timer(0.005, self.poll_serial)
        self.reconnect_timer = self.create_timer(0.2, self.maybe_reconnect)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.PORT, self.BAUDRATE, timeout=0)
            self.get_logger().info(f"[Serial] Connected to {self.PORT} baudrate {self.BAUDRATE}")
        except serial.SerialException as e:
            self.get_logger().warn(f"[Serial] Failed to open port {self.PORT}: {e}")
            self.ser = None

    def maybe_reconnect(self):
        if self.ser is None or not self.ser.is_open:
            now = time.time()
            if now - self.last_reconnect_attempt > self.reconnect_interval:
                self.last_reconnect_attempt = now
                self.connect_serial()
        if self.killnode:
            sys.exit(0)

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    # ---- Pi -> ESP32: lệnh động cơ ----
    def callback_velquery(self, data):
        if self.ser is None or not self.ser.is_open:
            return

        frame = [0x2a]
        frame.append((data.id >> 0) & 0xFF)
        frame.append((data.id >> 8) & 0xFF)
        frame.append((data.id >> 16) & 0xFF)
        frame.append((data.id >> 24) & 0xFF)
        frame.extend([
            data.byte0, data.byte1, data.byte2, data.byte3,
            data.byte4, data.byte5, data.byte6, data.byte7
        ])
        frame.append(0x23)

        try:
            self.ser.write(bytes(frame))
        except Exception as e:
            self.get_logger().warn(f"[Serial] Write failed: {e}")
            self.ser = None

    # ---- ESP32 -> Pi: đọc dòng text (chỉ giữ lại dòng "IMU,...") ----
    def poll_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            n = self.ser.in_waiting
            if n:
                self.rx_buf += self.ser.read(n)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"[Serial] Read error: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return

        while b'\n' in self.rx_buf:
            line, _, self.rx_buf = self.rx_buf.partition(b'\n')
            text = line.decode('ascii', errors='ignore').strip()
            if text:
                self.handle_line(text)

        # Phòng trường hợp nhận toàn rác không có '\n', tránh rx_buf phình vô hạn
        if len(self.rx_buf) > 4096:
            self.rx_buf.clear()

    def handle_line(self, line: str):
        if not line.startswith('IMU,'):
            return  # dòng debug/log khác từ ESP32, bỏ qua

        fields = line.split(',')
        if len(fields) != 15:
            self.get_logger().warn(f'[IMU] Malformed line, expected 15 fields, got {len(fields)}: {line}')
            return

        try:
            (qw, qx, qy, qz,
             gx, gy, gz,
             ax, ay, az,
             cal_sys, cal_gyro, cal_accel, cal_mag) = (float(f) for f in fields[1:])
        except ValueError:
            self.get_logger().warn(f'[IMU] Failed to parse line: {line}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id

        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation_covariance = self.orientation_cov

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = self.angular_velocity_cov

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = self.linear_acceleration_cov

        self.pub_imu.publish(msg)

        # Cảnh báo định kỳ nếu IMU chưa hiệu chuẩn xong
        now = time.time()
        if now - self.last_cal_log > 10.0:
            self.last_cal_log = now
            if min(cal_sys, cal_gyro, cal_accel, cal_mag) < 3:
                self.get_logger().warn(
                    f'[IMU] Calibration sys={int(cal_sys)} gyro={int(cal_gyro)} '
                    f'accel={int(cal_accel)} mag={int(cal_mag)} (3=full)')

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')


if __name__ == '__main__':
    main()
