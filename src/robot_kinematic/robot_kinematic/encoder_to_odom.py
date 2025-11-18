import rclpy
from rclpy.node import Node
from diff_serial_mgs.msg import EncoderVals
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import math
import tf_transformations
import tf2_ros

class EncoderToOdom(Node):
    def __init__(self):
        super().__init__('encoder_to_odom')

        # Params robot (giống URDF)
        self.encoder_cpr = 1975
        self.wheel_radius = 0.035  # m
        self.wheel_base = 0.25     # m (khoảng cách 2 bánh)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()
        self.last_left_counts = None
        self.last_right_counts = None

        self.sub = self.create_subscription(
            EncoderVals,
            'encoder_vals',
            self.encoder_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def encoder_callback(self, msg: EncoderVals):
        current_time = self.get_clock().now()

        if self.last_left_counts is None:
            self.last_left_counts = msg.mot_1_enc_val
            self.last_right_counts = msg.mot_2_enc_val
            self.last_time = current_time
            return

        # Tính delta counts
        delta_left = msg.mot_1_enc_val - self.last_left_counts
        delta_right = msg.mot_2_enc_val - self.last_right_counts
        dt = (current_time - self.last_time).nanoseconds / 1e9  # seconds

        if dt == 0:
            return

        self.last_left_counts = msg.mot_1_enc_val
        self.last_right_counts = msg.mot_2_enc_val
        self.last_time = current_time

        # Tính góc quay bánh
        delta_left_rad = (2.0 * math.pi * delta_left) / self.encoder_cpr
        delta_right_rad = (2.0 * math.pi * delta_right) / self.encoder_cpr

        # Tính quãng đường bánh đi được
        dist_left = delta_left_rad * self.wheel_radius
        dist_right = delta_right_rad * self.wheel_radius

        # Tính vận tốc và góc quay robot
        dist_center = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base

        # Tính vị trí mới
        if delta_theta == 0:
            dx = dist_center * math.cos(self.theta)
            dy = dist_center * math.sin(self.theta)
        else:
            # Đi theo cung tròn
            radius = dist_center / delta_theta
            dx = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            dy = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

        self.x += dx
        self.y += dy
        self.theta = (self.theta + delta_theta) % (2 * math.pi)

        # Tạo Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Velocity (optional, có thể tính dựa trên dt và dist_center)
        vx = dist_center / dt
        vth = delta_theta / dt

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        self.odom_pub.publish(odom_msg)

        # Broadcast TF transform odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
