#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 node: publish /odom + TF odom -> base_footprint mà KHÔNG dùng encoder.

Vì driver ESP32 hiện chưa có vòng điều khiển tốc độ (không PID), robot chưa
đo được vận tốc bánh thực tế đáng tin cậy để tính wheel odometry chuẩn. Node
này dùng 2 nguồn thay thế:

  - Vận tốc (vx, wz): lấy trực tiếp từ lệnh `/cmd_vel` đang gửi cho robot,
    coi như robot bám sát lệnh (không có phản hồi thực tế từ encoder).
  - Hướng (yaw): lấy từ quaternion đã fusion sẵn trên `/imu/data` (BNO055),
    đáng tin hơn nhiều so với tích phân wz từ cmd_vel vì không bị trôi theo
    thời gian.

Vị trí (x, y) được suy ra bằng cách tích phân vx theo hướng đo từ IMU:
  x += vx * cos(yaw) * dt
  y += vx * sin(yaw) * dt

Đây là odometry xấp xỉ (không phản ánh trượt bánh, tải lệch...) — đủ dùng
tạm cho slam_toolbox/nav2 trong lúc chưa có encoder feedback thật sự. Khi
driver có PID + publish vận tốc bánh đo được, nên thay `/cmd_vel` ở đây bằng
odometry tính từ encoder thật.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher_node')

        self.declare_parameters('', [
            ('rate', 50.0),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_footprint'),
            ('publish_tf', True),
            ('cmd_vel_timeout', 0.5),
            ('pose_stddev', 0.05),
            ('twist_stddev', 0.05),
        ])

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)

        pose_var = float(self.get_parameter('pose_stddev').value) ** 2
        twist_var = float(self.get_parameter('twist_stddev').value) ** 2
        self.pose_cov = [pose_var if i % 7 == 0 else 0.0 for i in range(36)]
        self.twist_cov = [twist_var if i % 7 == 0 else 0.0 for i in range(36)]

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_imu = False

        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.imu_wz = 0.0
        self.last_cmd_time = 0.0

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.on_imu, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_tick = self.get_clock().now()
        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / rate, self.on_tick)

    def on_cmd_vel(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z
        self.last_cmd_time = time.time()

    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.imu_wz = msg.angular_velocity.z
        self.have_imu = True

    def on_tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        if dt <= 0.0:
            return

        # Không nhận /cmd_vel gần đây -> coi như robot đã dừng, tránh trôi vị
        # trí do dùng vx cũ mãi khi node điều khiển cmd_vel đã ngừng gửi.
        vx = self.cmd_vx
        if time.time() - self.last_cmd_time > self.cmd_vel_timeout:
            vx = 0.0

        if self.have_imu:
            self.x += vx * math.cos(self.yaw) * dt
            self.y += vx * math.sin(self.yaw) * dt
            wz = self.imu_wz
        else:
            # Chưa có dữ liệu IMU: tạm tích phân wz từ cmd_vel để không đứng
            # yên hoàn toàn, chấp nhận trôi hướng cho tới khi IMU sẵn sàng.
            self.yaw += self.cmd_wz * dt
            self.x += vx * math.cos(self.yaw) * dt
            self.y += vx * math.sin(self.yaw) * dt
            wz = self.cmd_wz

        stamp = now.to_msg()
        half_yaw = self.yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = self.pose_cov
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = self.twist_cov
        self.pub_odom.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
