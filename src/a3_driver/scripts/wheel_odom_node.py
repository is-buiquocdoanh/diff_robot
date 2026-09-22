#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 node: sinh odometry bằng cách tích phân /cmd_vel theo thời gian
(dead-reckoning), làm nguồn "wheel odometry" giả cho robot_localization EKF
(xem odom.launch.py).

Driver ESP32 hiện chưa có encoder feedback + PID đáng tin cậy, nên chưa có
wheel odometry thật. Node này giả định robot bám sát lệnh cmd_vel (vx, vyaw)
để tích phân ra pose (x, y, yaw) -- pose này KHÔNG được EKF fuse (xem
odom0_config trong config/ekf.yaml, mọi field pose đều false), chỉ publish
kèm cho đầy đủ/nhất quán message (giống bản node cmd_vel_odom_node.py đã dùng
tốt với rf2o ở dự án trước). EKF chỉ thật sự tin phần twist (vx, vyaw), pose
đánh dấu độ tin cậy thấp qua covariance lớn (1e3) thay vì bỏ trống bằng 0.0 --
0.0 bị robot_localization hiểu là "đo chính xác tuyệt đối", nguy hiểm hơn hẳn
so với việc không publish field đó.

Node KHÔNG broadcast TF -- việc đó do EKF đảm nhận sau khi fuse với
/odom_rf2o (xem config/ekf.yaml), tránch ghi đè yaw thô mỗi tick như node
odom_publisher_node.py cũ (gây rung/giật khi IMU nhiễu).

vyaw ở đây CHỦ ĐỘNG đặt tin cậy hơn vyaw của rf2o_laser_odometry (xem
angular_stddev bên dưới) -- rf2o so khớp scan nên khi robot QUAY TẠI CHỖ, toàn
bộ điểm quét dịch chuyển giống nhau (thiếu parallax để so khớp chính xác),
dễ trôi hơn hẳn so với lúc tịnh tiến. cmd_vel ngược lại luôn đúng ý định quay
(dù không đo được có bị trượt bánh hay không), nên khi quay ưu tiên tin theo
lệnh hơn.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Pose tích lũy từ dead-reckoning trôi theo thời gian, không có cách nào tự
# biết đã lệch bao nhiêu -- đánh dấu độ tin cậy RẤT thấp (1e3) cho mọi chiều,
# thay vì để mặc định 0.0 (EKF hiểu 0.0 là "đo chính xác tuyệt đối").
_LOW_CONFIDENCE_VAR = 1e3


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

        # Pose: cả 6 chiều đều KHÔNG tin (dead-reckoning tự do, không có sensor
        # nào sửa lại) -- đủ để robot_localization biết bỏ qua thay vì tưởng
        # nhầm là số liệu chính xác.
        self.pose_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):  # x, y, z, roll, pitch, yaw
            self.pose_cov[i] = _LOW_CONFIDENCE_VAR

        # Twist: chỉ tin vx, vyaw (2 chiều robot vi sai thực sự có); vy/vz/vroll/
        # vpitch robot không thể có (không phải omni/bay) nên đánh dấu không tin.
        self.twist_cov = [0.0] * 36
        self.twist_cov[0] = linear_var           # var(vx)
        self.twist_cov[7] = _LOW_CONFIDENCE_VAR   # vy -- robot vi sai không đi ngang được
        self.twist_cov[14] = _LOW_CONFIDENCE_VAR  # vz
        self.twist_cov[21] = _LOW_CONFIDENCE_VAR  # vroll
        self.twist_cov[28] = _LOW_CONFIDENCE_VAR  # vpitch
        self.twist_cov[35] = angular_var          # var(vyaw)

        self.cmd_vx = 0.0
        self.cmd_vyaw = 0.0
        self.last_cmd_time = 0.0

        # Pose tích lũy nội bộ (dead-reckoning), chỉ để publish cho đầy đủ
        # message -- EKF không fuse (xem odom0_config trong config/ekf.yaml).
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_tick_time = self.get_clock().now()

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
        now = self.get_clock().now()
        dt = (now - self.last_tick_time).nanoseconds * 1e-9
        self.last_tick_time = now

        vx = self.cmd_vx
        vyaw = self.cmd_vyaw
        if time.time() - self.last_cmd_time > self.cmd_vel_timeout:
            vx = 0.0
            vyaw = 0.0

        # Tích phân dead-reckoning kiểu mid-point (chính xác hơn Euler thường
        # khi vyaw != 0 trong 1 tick): dùng góc quay giữa đầu-cuối tick để chiếu
        # vx sang x,y thay vì dùng góc đầu tick.
        if dt > 0.0:
            theta_mid = self.theta + 0.5 * vyaw * dt
            self.x += vx * math.cos(theta_mid) * dt
            self.y += vx * math.sin(theta_mid) * dt
            self.theta = math.atan2(
                math.sin(self.theta + vyaw * dt),
                math.cos(self.theta + vyaw * dt),
            )

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.pose.covariance = self.pose_cov
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
