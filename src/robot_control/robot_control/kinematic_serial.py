#!/usr/bin/env python3
"""ROS2 node: subscribe to /cmd_vel, compute wheel kinematics and send PWM frames to ESP32.

Frame format (CAN-serial): header 0x2A + id (4 bytes little-endian) + 8 data bytes + tail 0x23
data layout used: [LF_dir, LF_pwm, RF_dir, RF_pwm, 0,0,0,0]
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import struct
import time
import serial


class KinematicSerial(Node):
    def __init__(self):
        super().__init__('kinematic_serial')

        # parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('rate', 20)
        self.declare_parameter('log_rate', 2)
        self.declare_parameter('wheel_radius', 0.048)
        self.declare_parameter('wheel_base', 0.35)  # distance between wheels
        self.declare_parameter('rpm_max', 255) # max RPM corresponding to PWM 255 (for mapping cmd_vel to PWM)
        self.declare_parameter('frame_id', 1)
        # deadband and safety
        self.declare_parameter('deadband_linear', 0.02)
        self.declare_parameter('deadband_angular', 0.05)
        self.declare_parameter('stop_timeout', 1.0)
        self.declare_parameter('min_pwm_threshold', 6)

        self.port = self.get_parameter('serial_port').value
        self.baud = int(self.get_parameter('baudrate').value)
        self.rate = float(self.get_parameter('rate').value)
        self.log_rate = float(self.get_parameter('log_rate').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.rpm_max = float(self.get_parameter('rpm_max').value)
        self.frame_id = int(self.get_parameter('frame_id').value)
        # deadband and safety
        self.deadband_linear = float(self.get_parameter('deadband_linear').value)
        self.deadband_angular = float(self.get_parameter('deadband_angular').value)
        self.stop_timeout = float(self.get_parameter('stop_timeout').value)
        self.min_pwm_threshold = int(self.get_parameter('min_pwm_threshold').value)

        # serial
        self.ser = None
        self.connect_serial()

        # cmd_vel state
        self.last_twist = None
        self.last_recv = 0.0

        # subscribe to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmdvel_cb, 10)

        # timer to send frames
        self.timer = self.create_timer(1.0 / self.rate, self.timer_cb)

        self.get_logger().info(f'kinematic_serial started -> port: {self.port} @ {self.baud}')

    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                return
            self.ser = serial.Serial(self.port, int(self.baud), timeout=0.1)
            self.get_logger().info(f'Connected to {self.port}')
        except Exception as e:
            self.get_logger().warn(f'Failed to open serial {self.port}: {e}')
            self.ser = None

    def cmdvel_cb(self, msg: Twist):
        self.last_twist = msg
        self.last_recv = time.time()

    def diff_kinematics_rpm(self, twist: Twist):
        # use half base
        half_base = self.wheel_base / 2.0
        v = twist.linear.x
        omega = twist.angular.z
        v_l = v - omega * half_base
        v_r = v + omega * half_base
        # angular velocity rad/s
        w_l = 0.0
        w_r = 0.0
        if self.wheel_radius != 0:
            w_l = v_l / self.wheel_radius
            w_r = v_r / self.wheel_radius
        rpm_l = w_l * 60.0 / (2.0 * 3.141592653589793)
        rpm_r = w_r * 60.0 / (2.0 * 3.141592653589793)
        # clip
        rpm_l = max(-self.rpm_max, min(self.rpm_max, rpm_l))
        rpm_r = max(-self.rpm_max, min(self.rpm_max, rpm_r))
        return rpm_l, rpm_r

    def rpm_to_pwm(self, rpm_value):
        # map absolute rpm to 0..255
        pwm = int(min(255, max(0, round(abs(rpm_value) * 255.0 / max(1.0, self.rpm_max)))))
        return pwm

    def build_packet(self, lf_dir, lf_pwm, rf_dir, rf_pwm):
        header = b'\x2a'
        tail = b'\x23'
        pid = struct.pack('<I', int(self.frame_id))
        data = bytearray(8)
        data[0] = lf_dir & 0xFF
        data[1] = lf_pwm & 0xFF
        data[2] = rf_dir & 0xFF
        data[3] = rf_pwm & 0xFF
        packet = header + pid + bytes(data) + tail
        return packet

    def timer_cb(self):
        # ensure serial connected
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
        now = time.time()
        # if no cmd_vel recently, send stop
        if self.last_twist is None or (now - self.last_recv) > self.stop_timeout:
            lf_dir = 0; lf_pwm = 0; rf_dir = 0; rf_pwm = 0
        else:
            # check deadband: if both linear and angular are near zero, stop immediately
            v = float(self.last_twist.linear.x)
            omega = float(self.last_twist.angular.z)
            if abs(v) < self.deadband_linear and abs(omega) < self.deadband_angular:
                lf_dir = 0; lf_pwm = 0; rf_dir = 0; rf_pwm = 0
            else:
                rpm_l, rpm_r = self.diff_kinematics_rpm(self.last_twist)
                lf_pwm = self.rpm_to_pwm(rpm_l)
                rf_pwm = self.rpm_to_pwm(rpm_r)
                # prevent tiny pwm values from causing slow drift: enforce minimum threshold
                if lf_pwm <= self.min_pwm_threshold:
                    lf_pwm = 0
                if rf_pwm <= self.min_pwm_threshold:
                    rf_pwm = 0
                lf_dir = 1 if rpm_l > 0 and lf_pwm > 0 else (2 if rpm_l < 0 and lf_pwm > 0 else 0)
                rf_dir = 1 if rpm_r > 0 and rf_pwm > 0 else (2 if rpm_r < 0 and rf_pwm > 0 else 0)

        pkt = self.build_packet(lf_dir, lf_pwm, rf_dir, rf_pwm)
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(pkt)
            except Exception as e:
                self.get_logger().warn(f'Write fail: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

        # debug/info log, throttled by log_rate
        # log_rate is number of info logs per second (default 2)
        try:
            now_t = time.time()
            if not hasattr(self, '_last_log_time'):
                self._last_log_time = 0.0
            if (now_t - self._last_log_time) >= (1.0 / max(0.01, self.log_rate)):
                self.get_logger().info(f'sent L dir={lf_dir} pwm={lf_pwm} R dir={rf_dir} pwm={rf_pwm}')
                self._last_log_time = now_t
        except Exception:
            # keep timer resilient
            pass


def main(args=None):
    rclpy.init(args=args)
    node = KinematicSerial()
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
