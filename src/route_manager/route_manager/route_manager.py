import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import math
import time

class RouteManager(Node):

    def __init__(self):
        super().__init__('route_manager')

        self.declare_parameter('route_file', '')
        self.declare_parameter('marker_frame', 'map')
        route_file = self.get_parameter('route_file').get_parameter_value().string_value
        self.marker_frame = (
            self.get_parameter('marker_frame').get_parameter_value().string_value
        )

        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'route_markers',
            marker_qos
        )

        with open(route_file, 'r') as f:
            self.route = yaml.safe_load(f)['route']

        self.get_logger().info(f"Loaded {len(self.route)} waypoints")
        self.publish_route_markers()
        self.create_timer(1.0, self.publish_route_markers)

    def create_pose(self, wp):
        pose = PoseStamped()
        pose.header.frame_id = self.marker_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']

        yaw = wp.get('yaw', 0.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def publish_route_markers(self):
        markers = MarkerArray()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        now = self.get_clock().now().to_msg()

        line_marker = Marker()
        line_marker.header.frame_id = self.marker_frame
        line_marker.header.stamp = now
        line_marker.ns = 'route'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.08
        line_marker.color.r = 0.0
        line_marker.color.g = 0.7
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0

        for wp in self.route:
            point = Point()
            point.x = float(wp['x'])
            point.y = float(wp['y'])
            point.z = 0.05
            line_marker.points.append(point)

        markers.markers.append(line_marker)

        for i, wp in enumerate(self.route):
            name = wp.get('name', f'P{i + 1}')
            x = float(wp['x'])
            y = float(wp['y'])
            yaw = float(wp.get('yaw', 0.0))

            point_marker = Marker()
            point_marker.header.frame_id = self.marker_frame
            point_marker.header.stamp = now
            point_marker.ns = 'route_points'
            point_marker.id = i
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = x
            point_marker.pose.position.y = y
            point_marker.pose.position.z = 0.08
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.35
            point_marker.scale.y = 0.35
            point_marker.scale.z = 0.12
            point_marker.color.r = 1.0
            point_marker.color.g = 0.1
            point_marker.color.b = 0.1
            point_marker.color.a = 1.0
            markers.markers.append(point_marker)

            direction_marker = Marker()
            direction_marker.header.frame_id = self.marker_frame
            direction_marker.header.stamp = now
            direction_marker.ns = 'route_directions'
            direction_marker.id = i
            direction_marker.type = Marker.ARROW
            direction_marker.action = Marker.ADD
            direction_marker.pose.position.x = x
            direction_marker.pose.position.y = y
            direction_marker.pose.position.z = 0.15
            direction_marker.pose.orientation.z = math.sin(yaw / 2.0)
            direction_marker.pose.orientation.w = math.cos(yaw / 2.0)
            direction_marker.scale.x = 0.6
            direction_marker.scale.y = 0.12
            direction_marker.scale.z = 0.12
            direction_marker.color.r = 1.0
            direction_marker.color.g = 0.8
            direction_marker.color.b = 0.0
            direction_marker.color.a = 1.0
            markers.markers.append(direction_marker)

            text_marker = Marker()
            text_marker.header.frame_id = self.marker_frame
            text_marker.header.stamp = now
            text_marker.ns = 'route_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.6
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.45
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = name
            markers.markers.append(text_marker)

        self.marker_pub.publish(markers)

    def send_route(self):
        goal = FollowWaypoints.Goal()
        goal.poses = [self.create_pose(wp) for wp in self.route]

        self.client.wait_for_server()
        self.get_logger().info("Sending route...")

        send_goal_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Route rejected!")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Route finished!")
        return True

    def handle_waypoint_logic(self):
        for wp in self.route:
            if 'wait' in wp:
                self.get_logger().info(f"Waiting at {wp['name']} for {wp['wait']}s")
                time.sleep(wp['wait'])

    def run(self):
        while rclpy.ok():
            success = self.send_route()

            if success:
                self.handle_waypoint_logic()
                self.get_logger().info("Looping route...")
            else:
                self.get_logger().warn("Retrying route...")

def main():
    rclpy.init()
    node = RouteManager()
    node.run()

if __name__ == '__main__':
    main()
