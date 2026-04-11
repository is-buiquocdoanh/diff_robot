#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import các thư viện cần thiết
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import Twist           # Thư viện ROS để nhận lệnh vận tốc
from message_pkg.msg import Velquery
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
                ('r_banh', 0.048),
                ('lx', 0.35),
                ('rpm_max', 130),
            ]
        )
        
        self.rate = self.get_parameter('rate').value
        
        # Get parameters
        self.r_banh = self.get_parameter('r_banh').get_parameter_value().double_value
        self.lx = self.get_parameter('lx').get_parameter_value().double_value
        self.rpm_max= self.get_parameter('rpm_max').get_parameter_value().integer_value
        # use angular.z directly (no scaling)
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

    def diff_kinematics_rpm(self, vel):
        """
        Vx, Vy: m/s (robot velocity)
        omega: rad/s (robot angular velocity)
        L, W: chiều dài / rộng từ tâm robot đến bánh
        r: bán kính bánh xe (m)
        Trả về: [rpm_FL, rpm_FR, rpm_RL, rpm_RR]
        """
        """
        Differential drive kinematics: convert robot linear.x (m/s) and angular.z (rad/s)
        into wheel RPM for left and right wheels.

        Returns array of 4 values [FL, FR, RL, RR] where RL=FL and RR=FR for compatibility
        with the existing 4-wheel command packing expected by the ESP32 firmware.
        """
        # Assume self.lx is half the wheel base (distance from center to each wheel)
        half_base = self.lx

        v = vel.linear.x
        omega = vel.angular.z  # Use angular.z directly without scaling

        # Linear velocities of left and right wheels (m/s)
        v_l = v - omega * half_base
        v_r = v + omega * half_base

        # Convert linear velocity to wheel angular velocity (rad/s): w = v / r
        w_l = v_l / self.r_banh
        w_r = v_r / self.r_banh

        # Convert to RPM
        rpm_l = w_l * 60.0 / (2.0 * np.pi)
        rpm_r = w_r * 60.0 / (2.0 * np.pi)

        # Clip and round
        rpm_l_c = int(round(np.clip(rpm_l, -self.rpm_max, self.rpm_max)))
        rpm_r_c = int(round(np.clip(rpm_r, -self.rpm_max, self.rpm_max)))

        # Return as 4-wheel array: FL, FR, RL, RR (duplicate left/right for back wheels)
        return np.array([rpm_l_c, rpm_r_c, rpm_l_c, rpm_r_c])

    # Hàm điều khiển chính
    def run(self):
        if self.is_cmdVel == 1:
            # Tính toán giá trị PWM cho bánh trái và bánh phải
            wheel_rpm = self.diff_kinematics_rpm(self.data_cmdVel)

            if time.time() - self.time_cmdVel_recv < 1:
                print("Tốc độ từng bánh (RPM):")
                print(f"FL: {wheel_rpm[0]}")
                print(f"FR: {wheel_rpm[1]}")
                print("--------------------------")

                if wheel_rpm[0] > 0:
                    self.data_velquery.byte0 = 1
                    self.data_velquery.byte1 = int(wheel_rpm[0])
                elif wheel_rpm[0] < 0:
                    self.data_velquery.byte0 = 2
                    self.data_velquery.byte1 = int(-wheel_rpm[0])
                else:
                    self.data_velquery.byte0 = 0
                    self.data_velquery.byte1 = 0                    

                if wheel_rpm[1] > 0:
                    self.data_velquery.byte2 = 1
                    self.data_velquery.byte3 = int(wheel_rpm[1])
                elif wheel_rpm[1] < 0:
                    self.data_velquery.byte2 = 2
                    self.data_velquery.byte3 = int(-wheel_rpm[1])
                else:
                    self.data_velquery.byte2 = 0
                    self.data_velquery.byte3 = 0 

                if wheel_rpm[2] > 0:
                    self.data_velquery.byte4 = 1
                    self.data_velquery.byte5 = int(wheel_rpm[2])
                elif wheel_rpm[2] < 0:
                    self.data_velquery.byte4 = 2
                    self.data_velquery.byte5 = int(-wheel_rpm[2])
                else:
                    self.data_velquery.byte4 = 0
                    self.data_velquery.byte5 = 0 

                if wheel_rpm[3] > 0:
                    self.data_velquery.byte6 = 1
                    self.data_velquery.byte7 = int(wheel_rpm[3])
                elif wheel_rpm[3] < 0:
                    self.data_velquery.byte6 = 2
                    self.data_velquery.byte7 = int(-wheel_rpm[3])
                else:
                    self.data_velquery.byte6 = 0
                    self.data_velquery.byte7 = 0

            else:
                print("Tốc độ từng bánh (RPM):")
                print(f"FL: 0.0")
                print(f"FR: 0.0")
                print("--------------------------")

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
