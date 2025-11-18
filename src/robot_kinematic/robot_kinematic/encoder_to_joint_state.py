# Subscribe topic encoder_vals (với encoder count)
# Chuyển đổi giá trị encoder count sang góc quay joint_states.position
# Publish topic joint_states


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from diff_serial_mgs.msg import EncoderVals
import math

class EncoderToJointState(Node):
    def __init__(self):
        super().__init__('encoder_to_joint_state')
        self.encoder_sub = self.create_subscription(
            EncoderVals,
            'encoder_vals',
            self.encoder_callback,
            10
        )
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.encoder_cpr = 1975  # Phải giống node diff_driver

        self.joint_state = JointState()
        self.joint_state.name = ['left_wheel_joint', 'right_wheel_joint']

    def encoder_callback(self, msg: EncoderVals):
        left_counts = msg.mot_1_enc_val
        right_counts = msg.mot_2_enc_val

        # Tính góc quay rad = counts * (2pi / CPR)
        left_angle = left_counts * 2.0 * math.pi / self.encoder_cpr
        right_angle = right_counts * 2.0 * math.pi / self.encoder_cpr

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [left_angle, right_angle]

        self.joint_pub.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderToJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
