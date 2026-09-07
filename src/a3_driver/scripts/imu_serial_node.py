#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 node: đọc dữ liệu IMU (BNO055) do ESP32 gửi qua UART2 và publish sensor_msgs/Imu.

Frame (ASCII, 1 dòng, kết thúc bằng '\\n'):
  IMU,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,cal_sys,cal_gyro,cal_accel,cal_mag

  qw,qx,qy,qz : quaternion orientation (đã fusion sẵn trên chip BNO055)
  gx,gy,gz    : angular velocity (rad/s)
  ax,ay,az    : linear acceleration, đã trừ trọng lực (m/s^2)
  cal_*       : trạng thái hiệu chuẩn 0-3 (3 = hiệu chuẩn đầy đủ)

Đây là kênh serial RIÊNG với kênh điều khiển động cơ (ros_serial_bridge.py),
để tránh trộn dữ liệu ASCII (IMU) với frame nhị phân (CAN-serial) trên cùng
một cổng.
"""
import time

import rclpy
from rclpy.node import Node
import serial

from sensor_msgs.msg import Imu


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')

        self.declare_parameters('', [
            ('serial_port', '/dev/ttyAMA0'),
            ('baudrate', 115200),
            ('frame_id', 'imu_link'),
            ('reconnect_interval', 1.5),
            # phương sai (variance) mặc định cho các trường - chỉnh lại sau khi
            # đã đánh giá độ nhiễu thực tế của cảm biến
            ('orientation_stddev', 0.02),
            ('angular_velocity_stddev', 0.02),
            ('linear_acceleration_stddev', 0.05),
        ])

        self.port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)

        ori_var = float(self.get_parameter('orientation_stddev').value) ** 2
        ang_var = float(self.get_parameter('angular_velocity_stddev').value) ** 2
        acc_var = float(self.get_parameter('linear_acceleration_stddev').value) ** 2
        self.orientation_cov = [ori_var, 0.0, 0.0, 0.0, ori_var, 0.0, 0.0, 0.0, ori_var]
        self.angular_velocity_cov = [ang_var, 0.0, 0.0, 0.0, ang_var, 0.0, 0.0, 0.0, ang_var]
        self.linear_acceleration_cov = [acc_var, 0.0, 0.0, 0.0, acc_var, 0.0, 0.0, 0.0, acc_var]

        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        self.ser = None
        self.last_reconnect_attempt = 0.0
        self.last_cal_log = 0.0
        self.connect_serial()

        # Đọc serial không chặn (non-blocking) trong timer, tránh spin() bị block bởi readline()
        self.create_timer(0.005, self.poll_serial)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0)
            self.get_logger().info(f'[IMU] Connected to {self.port} @ {self.baudrate}')
        except serial.SerialException as e:
            self.get_logger().warn(f'[IMU] Failed to open {self.port}: {e}')
            self.ser = None

    def poll_serial(self):
        now = time.time()
        if self.ser is None or not self.ser.is_open:
            if now - self.last_reconnect_attempt > self.reconnect_interval:
                self.last_reconnect_attempt = now
                self.connect_serial()
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line:
                    self.handle_line(line)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'[IMU] Serial read error: {e}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

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
        msg.header.frame_id = self.frame_id

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

        # Cảnh báo định kỳ nếu IMU chưa hiệu chuẩn xong (giúp phát hiện sớm khi cắm mới)
        now = time.time()
        if now - self.last_cal_log > 10.0:
            self.last_cal_log = now
            if min(cal_sys, cal_gyro, cal_accel, cal_mag) < 3:
                self.get_logger().warn(
                    f'[IMU] Calibration sys={int(cal_sys)} gyro={int(cal_gyro)} '
                    f'accel={int(cal_accel)} mag={int(cal_mag)} (3=full)')


def main():
    rclpy.init()
    node = ImuSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
