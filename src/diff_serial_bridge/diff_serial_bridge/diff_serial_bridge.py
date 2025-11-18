# diff_serial_driver.py
# Tác giả: Bui Quoc Doanh
# Email: doanh762003@gmail.com   
# Node này:
# - Giao tiếp với Arduino qua serial.
# - Đọc encoder (e) và xuất ra topic /encoder_vals.
# - Gửi lệnh tốc độ động cơ (PWM hoặc tick/loop) khi có topic /motor_command.
# ⚠️ Nó KHÔNG subscribe topic /cmd_vel — vì vậy robot của bạn sẽ không di chuyển khi bạn gửi ros2 topic pub /cmd_vel.

# Code hoạt động như sau:
# /motor_command  →  MotorDriver  →  Arduino  →  Động cơ
# Arduino → encoder → MotorDriver → /encoder_vals

# Còn lệnh cmd_vel nằm ngoài vòng này — nó chưa được chuyển đổi thành motor_command. Nên cần node cmdvel_to_motorcommand.py để làm việc đó.

import rclpy
from rclpy.node import Node
from diff_serial_mgs.msg import MotorCommand
from diff_serial_mgs.msg import MotorVels
from diff_serial_mgs.msg import EncoderVals
import time
import math
import serial
from threading import Lock

class MotorDriver(Node):

    def __init__(self):
        super().__init__('diff_serial_bridge')

        # Setup parameters
        self.declare_parameter('encoder_cpr', value=1975)
        if (self.get_parameter('encoder_cpr').value == 0):
            print("WARNING! ENCODER CPR SET TO 0!!")


        self.declare_parameter('loop_rate', value=30)
        if (self.get_parameter('loop_rate').value == 0):
            print("WARNING! LOOP RATE SET TO 0!!")


        self.declare_parameter('serial_port', value="/dev/arduino")
        self.serial_port = self.get_parameter('serial_port').value


        self.declare_parameter('baud_rate', value=57600)
        self.baud_rate = self.get_parameter('baud_rate').value


        self.declare_parameter('serial_debug', value=True)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")

        # Setup topics & services
        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

        self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)

        self.encoder_pub = self.create_publisher(EncoderVals, 'encoder_vals', 10)
        
        # Member Variables
        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.m1_spd = 0.0
        self.m2_spd = 0.0

        self.mutex = Lock()

        # Open serial comms
        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")
        
    # Raw serial commands
    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        self.send_command(f"o {int(mot_1_pwm)} {int(mot_2_pwm)}")

    def send_feedback_motor_command(self, mot_1_ct_per_loop, mot_2_ct_per_loop):
        self.send_command(f"m {int(mot_1_ct_per_loop)} {int(mot_2_ct_per_loop)}")

    def send_encoder_read_command(self):
        resp = self.send_command(f"e")
        if resp:
            return [int(raw_enc) for raw_enc in resp.split()]
        return []

    # More user-friendly functions
    def motor_command_callback(self, motor_command):
        if (motor_command.is_pwm):
            self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        else:
            # counts per loop = req rads/sec X revs/rad X counts/rev X secs/loop 
            scaler = (1 / (2*math.pi)) * self.get_parameter('encoder_cpr').value * (1 / self.get_parameter('loop_rate').value)
            mot1_ct_per_loop = motor_command.mot_1_req_rad_sec * scaler
            mot2_ct_per_loop = motor_command.mot_2_req_rad_sec * scaler
            self.send_feedback_motor_command(mot1_ct_per_loop, mot2_ct_per_loop)

    def check_encoders(self):
        resp = self.send_encoder_read_command()
        if (resp):

            new_time = time.time()
            time_diff = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            m1_diff = resp[0] - self.last_m1_enc
            self.last_m1_enc = resp[0]
            m2_diff = resp[1] - self.last_m2_enc
            self.last_m2_enc = resp[1]

            rads_per_ct = 2*math.pi/self.get_parameter('encoder_cpr').value
            self.m1_spd = m1_diff*rads_per_ct/time_diff
            self.m2_spd = m2_diff*rads_per_ct/time_diff

            spd_msg = MotorVels()
            spd_msg.mot_1_rad_sec = self.m1_spd
            spd_msg.mot_2_rad_sec = self.m2_spd
            self.speed_pub.publish(spd_msg)

            enc_msg = EncoderVals()
            enc_msg.mot_1_enc_val = self.last_m1_enc
            enc_msg.mot_2_enc_val = self.last_m2_enc
            self.encoder_pub.publish(enc_msg)

    # Utility functions
    def send_command(self, cmd_string):
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))
            if self.debug_serial_cmds:
                print("Sent:", repr(cmd_string))

            value = self.conn.readline().decode("utf-8").strip()
            if self.debug_serial_cmds:
                print("Received:", repr(value))
            return value
        except Exception as e:
            print("Error reading serial:", e)
            return ''
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()


def main(args=None):
    
    rclpy.init(args=args)

    motor_driver = MotorDriver()

    rate = motor_driver.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(motor_driver)
        motor_driver.check_encoders()

    motor_driver.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()

