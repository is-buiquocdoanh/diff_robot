#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 node: quy đổi /cmd_vel thành nav_msgs/Odometry (vx + vyaw), làm nguồn
"wheel odometry" giả cho robot_localization EKF (xem odom.launch.py).

Driver ESP32 hiện chưa có encoder feedback + PID đáng tin cậy, nên chưa có
wheel odometry thật. Node này chỉ đứng vai "cảm biến vx/vyaw" (coi robot bám
sát lệnh cmd_vel), KHÔNG tự tích phân vị trí/hướng và KHÔNG broadcast TF —
việc đó nay do EKF đảm nhận, fusion với /odom_rf2o (xem config/ekf.yaml).
Tách trách nhiệm thế này để EKF lọc mượt theo covariance thay vì ghi đè yaw
thô mỗi tick như node odom_publisher_node.py cũ (gây rung/giật khi IMU nhiễu).

vyaw ở đây CHỦ ĐỘNG đặt tin cậy hơn vyaw của rf2o_laser_odometry (xem
angular_stddev bên dưới) -- rf2o so khớp scan nên khi robot QUAY TẠI CHỖ, toàn
bộ điểm quét dịch chuyển giống nhau (thiếu parallax để so khớp chính xác),
dễ trôi hơn hẳn so với lúc tịnh tiến. cmd_vel ngược lại luôn đúng ý định quay
(dù không đo được có bị trượt bánh hay không), nên khi quay ưu tiên tin theo
lệnh hơn.
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')

        self.declare_parameters('', [
            ('rate', 50.0),
            ('odom_topic', '/wheel/odom'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_footprint'),
            ('cmd_vel_timeout', 0.5),
            ('linear_stddev', 0.05),
            # 0.093 rad/s (~5.3 độ/s) -- chọn để khi kết hợp với var(vyaw) cố định
            # 0.02 của rf2o (xem CLaserOdometry2DNode.cpp), tỉ lệ tin cậy giữa 2
            # nguồn xấp xỉ 70% wheel / 30% rf2o lúc quay (EKF càng gần fusion theo
            # nghịch đảo variance: w_wheel = var_rf2o / (var_rf2o + var_wheel)).
            # Muốn đổi tỉ lệ p (0-1): var_wheel = var_rf2o * (1-p)/p.
            ('angular_stddev', 0.093),
        ])

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)

        linear_var = float(self.get_parameter('linear_stddev').value) ** 2
        angular_var = float(self.get_parameter('angular_stddev').value) ** 2
        self.twist_cov = [0.0] * 36
        self.twist_cov[0] = linear_var    # var(vx)
        self.twist_cov[35] = angular_var  # var(vyaw) -- hàng/cột 6 (index 5) của ma trận 6x6

        self.cmd_vx = 0.0
        self.cmd_vyaw = 0.0
        self.last_cmd_time = 0.0

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        odom_topic = self.get_parameter('odom_topic').value
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 10)

        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / rate, self.on_tick)

    def on_cmd_vel(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vyaw = msg.angular.z
        self.last_cmd_time = time.time()

    def on_tick(self):
        vx = self.cmd_vx
        vyaw = self.cmd_vyaw
        if time.time() - self.last_cmd_time > self.cmd_vel_timeout:
            vx = 0.0
            vyaw = 0.0

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.orientation.w = 1.0  # không dùng field pose (EKF bỏ qua), giữ quaternion hợp lệ
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vyaw
        odom.twist.covariance = self.twist_cov
        self.pub_odom.publish(odom)


def main():
    rclpy.init()
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
