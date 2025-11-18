import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from diff_serial_mgs.msg import MotorCommand

class CmdVelToMotorCommand(Node):
    def __init__(self):
        super().__init__('cmdvel_to_motorcommand')

        # Tham số robot
        self.declare_parameter('wheel_base', 0.29)      # khoảng cách giữa 2 bánh xe (m)
        self.declare_parameter('wheel_radius', 0.035)   # bán kính bánh xe (m)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Đăng ký subscriber và publisher
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)

        self.get_logger().info("✅ Node cmdvel_to_motorcommand started!")

    def cmd_vel_callback(self, msg: Twist):
        # Lấy linear và angular velocity
        linear_x = msg.linear.x    # m/s
        angular_z = msg.angular.z  # rad/s

        # Tính vận tốc góc từng bánh (rad/s)
        v_left = (linear_x - (self.wheel_base / 2.0) * angular_z) / self.wheel_radius
        v_right = (linear_x + (self.wheel_base / 2.0) * angular_z) / self.wheel_radius

        # Tạo message MotorCommand
        motor_cmd = MotorCommand()
        motor_cmd.is_pwm = False
        motor_cmd.mot_1_req_rad_sec = v_left
        motor_cmd.mot_2_req_rad_sec = v_right

        # Gửi lệnh
        self.publisher.publish(motor_cmd)

        # In ra terminal để debug
        self.get_logger().info(
            f"📡 cmd_vel: linear_x={linear_x:.2f} m/s, angular_z={angular_z:.2f} rad/s | "
            f"➡️ MotorCommand: L={v_left:.2f} rad/s, R={v_right:.2f} rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotorCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
