#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import các thư viện cần thiết
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import Twist           # Thư viện ROS để nhận lệnh vận tốc
from std_msgs.msg import Int16  # Để gửi dữ liệu dạng mảng số nguyên
from math import pi as PI                           # Sử dụng số pi cho tính toán góc và vận tốc
from a3_driver.msg import *
from a3_driver.msg import Velquery
import numpy as np
import time
import sys

class ControlMotorByKinematic(LifecycleNode):
    def __init__(self):
        # Khởi tạo node ROS tên 'control_motor'
        super().__init__('kinematic')
        self.get_logger().info("ROS 2 Node Initialized!")
        self.killnode = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rate', 100),
                ('enb_debug', 0),
                ('r_banh', 0.0475),
                ('wheel_separation', 0.365),
                ('pwm_max', 255),
                ('pwm_min', 0),
                ('rpm_max', 320),
                # hệ số nhân cho vận tốc góc (dùng để tăng tốc quay tại chỗ)
                ('omega_scale', 1.0),
                # vùng chết: bỏ qua các lệnh cmd_vel quá nhỏ
                ('deadband_linear', 0.01),
                ('deadband_angular', 0.01),
                # PWM tối thiểu để bánh thực sự quay; dưới ngưỡng này thì coi như dừng
                ('min_pwm_threshold', 0),
                # nếu không nhận cmd_vel mới trong khoảng thời gian này (s) thì dừng xe
                ('stop_timeout', 0.5),
            ]
        )

        self.rate = self.get_parameter('rate').value
        self.enb_debug = self.get_parameter('enb_debug').value

        # Get parameters
        self.r_banh = self.get_parameter('r_banh').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.pwm_max = self.get_parameter('pwm_max').get_parameter_value().integer_value
        self.pwm_min = self.get_parameter('pwm_min').get_parameter_value().integer_value
        self.rpm_max = self.get_parameter('rpm_max').get_parameter_value().integer_value
        # hệ số nhân cho omega (angular.z) trước khi tính kinematics
        self.omega_scale = self.get_parameter('omega_scale').get_parameter_value().double_value
        self.deadband_linear = self.get_parameter('deadband_linear').get_parameter_value().double_value
        self.deadband_angular = self.get_parameter('deadband_angular').get_parameter_value().double_value
        self.min_pwm_threshold = self.get_parameter('min_pwm_threshold').get_parameter_value().integer_value
        self.stop_timeout = self.get_parameter('stop_timeout').get_parameter_value().double_value

        # Đăng ký callback để nhận lệnh vận tốc từ topic /cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmdVel_callback,10)
        self.data_cmdVel = Twist()
        self.is_cmdVel = 0
        self.time_cmdVel_recv = time.time()

        self.pub_vel_query = self.create_publisher(Velquery, '/vel_query', 10)
        self.data_velquery = Velquery()

        self.timer = self.create_timer(1/self.rate, self.run)

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    # Hàm callback khi có dữ liệu mới từ topic /cmd_vel
    def cmdVel_callback(self, data):
        self.data_cmdVel = data
        self.is_cmdVel = 1
        self.time_cmdVel_recv = time.time()

    def diff_drive_kinematics_rpm(self, vel):
        """
        Vx: m/s (vận tốc thẳng của robot)
        omega: rad/s (vận tốc góc của robot)
        wheel_separation: khoảng cách giữa 2 bánh (m)
        r: bán kính bánh xe (m)
        Trả về: [rpm_left, rpm_right]
        """
        vx = vel.linear.x
        omega = self.omega_scale * vel.angular.z

        # vùng chết: lệnh quá nhỏ thì coi như đứng yên
        if abs(vx) < self.deadband_linear and abs(vel.angular.z) < self.deadband_angular:
            vx = 0.0
            omega = 0.0

        v_left = vx - (self.wheel_separation / 2.0) * omega
        v_right = vx + (self.wheel_separation / 2.0) * omega

        w_left_rpm = v_left * 60.0 / (2 * np.pi * self.r_banh)
        w_right_rpm = v_right * 60.0 / (2 * np.pi * self.r_banh)

        w_rpm = np.array([w_left_rpm, w_right_rpm])
        w_rpm_clipped = np.clip(w_rpm, -self.rpm_max, self.rpm_max)

        return w_rpm_clipped

    def rpm_to_pwm(self, rpm):
        """Map |rpm| trong [0, rpm_max] sang pwm trong [pwm_min, pwm_max]."""
        if self.rpm_max <= 0:
            return 0
        pwm = int(round((abs(rpm) / self.rpm_max) * (self.pwm_max - self.pwm_min) + self.pwm_min))
        pwm = int(np.clip(pwm, self.pwm_min, self.pwm_max))
        if pwm < self.min_pwm_threshold:
            pwm = 0
        return pwm

    # Hàm điều khiển chính
    def run(self):
        if self.is_cmdVel == 1:
            if time.time() - self.time_cmdVel_recv < self.stop_timeout:
                wheel_rpm = self.diff_drive_kinematics_rpm(self.data_cmdVel)
                rpm_left, rpm_right = wheel_rpm[0], wheel_rpm[1]

                if self.enb_debug:
                    print(f"RPM  L={rpm_left:.1f}  R={rpm_right:.1f}")

                pwm_left = self.rpm_to_pwm(rpm_left)
                pwm_right = self.rpm_to_pwm(rpm_right)

                dir_left = 1 if rpm_left > 0 else (2 if rpm_left < 0 else 0)
                dir_right = 1 if rpm_right > 0 else (2 if rpm_right < 0 else 0)
                if pwm_left == 0:
                    dir_left = 0
                if pwm_right == 0:
                    dir_right = 0

                self.data_velquery.byte0 = dir_left
                self.data_velquery.byte1 = pwm_left
                self.data_velquery.byte2 = dir_right
                self.data_velquery.byte3 = pwm_right
                self.data_velquery.byte4 = 0
                self.data_velquery.byte5 = 0
                self.data_velquery.byte6 = 0
                self.data_velquery.byte7 = 0
            else:
                if self.enb_debug:
                    print("cmd_vel timeout - stopping")
                self.data_velquery = Velquery()

            self.pub_vel_query.publish(self.data_velquery)
        # -- KILL NODE --
        if self.killnode:
            sys.exit(0)


def main():
    rclpy.init()
    node = ControlMotorByKinematic()
    print("launch kinematic")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
