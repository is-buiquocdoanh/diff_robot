"""Robot giả để thử giao diện web mà không cần phần cứng.

Giả lập: TF odom->base_footprint (tích phân /cmd_vel), /odom, /scan (raycast trên bản
đồ "thế giới thật"), SLAM vẽ dần bản đồ trên /map, AMCL (nhận /initialpose -> TF map->odom)
và action navigate_to_pose. Chế độ được suy ra từ các node trong graph (xem fake_stack):

    slam_toolbox  -> chế độ vẽ map: phát TF map->odom + /map tăng dần theo scan
    amcl          -> chế độ định vị: TF map->odom chỉ có sau khi nhận /initialpose
    bt_navigator  -> có action navigate_to_pose

    ros2 run a3_web fake_robot --ros-args -p world_map:=/đường/dẫn/map.yaml
"""
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Transform, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from a3_web.maps import read_pgm
from a3_web.ros_bridge import quat_from_yaw, yaw_from_quat

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

SCAN_BEAMS = 360
SCAN_MAX = 8.0
STEP = 0.03


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """Ghép phép biến đổi 2D a∘b, mỗi cái là (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


class World:
    """Bản đồ 'sự thật' để raycast: đọc từ yaml/pgm hoặc sinh 1 căn phòng mẫu."""

    def __init__(self, yaml_path=''):
        self.res, self.origin = 0.05, (-4.0, -3.0)
        if yaml_path and os.path.isfile(yaml_path):
            meta = yaml.safe_load(open(yaml_path))
            self.res = float(meta['resolution'])
            self.origin = (float(meta['origin'][0]), float(meta['origin'][1]))
            _, _, pix = read_pgm(os.path.join(os.path.dirname(yaml_path), meta['image']))
            p = (255 - pix.astype(np.float32)) / 255.0 if not meta.get('negate', 0) else pix / 255.0
            self.occ = (p > float(meta.get('occupied_thresh', 0.65)))[::-1]   # hàng 0 = y thấp nhất
            self.free = (p < float(meta.get('free_thresh', 0.25)))[::-1]
        else:
            h, w = 120, 160
            occ = np.zeros((h, w), bool)
            occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
            occ[40:80, 70] = True
            occ[30:34, 20:50] = True
            self.occ, self.free = occ, ~occ
        self.h, self.w = self.occ.shape

    def cell(self, x, y):
        return ((np.floor((y - self.origin[1]) / self.res)).astype(int),
                (np.floor((x - self.origin[0]) / self.res)).astype(int))

    def is_occupied(self, x, y):
        iy, ix = self.cell(np.asarray(x), np.asarray(y))
        inside = (iy >= 0) & (iy < self.h) & (ix >= 0) & (ix < self.w)
        out = np.zeros(np.shape(inside), bool)
        out[inside] = self.occ[iy[inside], ix[inside]]
        return out

    def default_start(self):
        ys, xs = np.nonzero(self.free)
        if len(xs) == 0:
            return (0.0, 0.0, 0.0)
        cy, cx = ys.mean(), xs.mean()
        for i in np.argsort((ys - cy) ** 2 + (xs - cx) ** 2)[:4000]:
            y, x = ys[i], xs[i]
            win = self.occ[max(0, y - 6):y + 7, max(0, x - 6):x + 7]
            if not win.any():
                return (self.origin[0] + (x + 0.5) * self.res, self.origin[1] + (y + 0.5) * self.res, 0.0)
        return (0.0, 0.0, 0.0)

    def raycast(self, pose, angles):
        """Trả về (ranges, r_hit_samples): khoảng cách trúng vật cản mỗi tia (inf nếu không trúng)."""
        r = np.arange(0.1, SCAN_MAX, STEP)
        th = pose[2] + angles
        px = pose[0] + np.outer(np.cos(th), r)
        py = pose[1] + np.outer(np.sin(th), r)
        hit = self.is_occupied(px, py)
        any_hit = hit.any(axis=1)
        first = np.argmax(hit, axis=1)
        ranges = np.where(any_hit, r[first], np.inf)
        return ranges, r, px, py, any_hit, first


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.declare_parameter('world_map', '')
        self.declare_parameter('start_x', float('nan'))
        self.declare_parameter('start_y', float('nan'))
        self.declare_parameter('start_yaw', 0.0)
        # true: tự giả lập SLAM/AMCL/Nav2 (dùng với fake_stack). false: CHỈ giả lập phần cứng (odom, scan,
        # TF odom->base, nhận /cmd_vel) để chạy stack THẬT (slam_toolbox, Nav2...) lên trên - xem README.
        self.declare_parameter('emulate_stacks', True)
        wm = self.get_parameter('world_map').value
        if not wm:
            from a3_web.server import default_maps_dir
            cand = os.path.join(default_maps_dir(), 'map1', 'map1.yaml')
            wm = cand if os.path.isfile(cand) else ''
        self.world = World(wm)
        sx, sy = self.get_parameter('start_x').value, self.get_parameter('start_y').value
        start = self.world.default_start()
        self.start = (start[0] if math.isnan(sx) else sx, start[1] if math.isnan(sy) else sy,
                      float(self.get_parameter('start_yaw').value))
        self.emulate = bool(self.get_parameter('emulate_stacks').value)
        self.get_logger().info(f'Thế giới giả: {wm or "phòng mẫu"}; vị trí xuất phát {self.start}')

        self.lock = threading.RLock()
        self.true = self.start                    # pose thật trong khung "thế giới"
        self.cmd = (0.0, 0.0)
        self.cmd_time = 0.0
        self.nav_cmd = (0.0, 0.0)
        self.map_odom = None                      # T_map_odom (None -> chưa có khung map)
        self.slam_grid = np.full((self.world.h, self.world.w), -1, np.int8)
        self.graph = set()
        self.prev_mode = 'idle'
        self.plan_pts = []

        cbg = ReentrantCallbackGroup()
        self.tf_pub = TransformBroadcaster(self)
        self.static_pub = StaticTransformBroadcaster(self)
        self._publish_static()
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_plan = self.create_publisher(Path, '/plan', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._on_init, 10)
        self.create_timer(0.05, self._physics, callback_group=cbg)
        self.create_timer(0.1, self._scan_step, callback_group=cbg)
        self.create_timer(1.0, self._slow_step, callback_group=cbg)
        self.action = None
        self._cbg = cbg

    # ------------------------------------------------------------------ helpers
    def _publish_static(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_footprint', 'laser_link'
        t.transform.translation.z = 0.15
        t.transform.rotation.w = 1.0
        self.static_pub.sendTransform(t)

    def _odom_pose(self):
        return compose(inverse(self.start), self.true)

    def _est_pose(self):
        """Pose robot trong khung map theo 'ước lượng' (SLAM/AMCL), None nếu chưa có khung map."""
        return compose(self.map_odom, self._odom_pose()) if self.map_odom else None

    def _mode(self):
        if 'slam_toolbox' in self.graph:
            return 'slam'
        if 'amcl' in self.graph:
            return 'loc'
        return 'idle'

    def _tf(self, parent, child, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = pose[0], pose[1]
        t.transform.rotation.z, t.transform.rotation.w = quat_from_yaw(pose[2])
        return t

    # ---------------------------------------------------------------- callbacks
    def _on_cmd(self, msg):
        with self.lock:
            self.cmd = (msg.linear.x, msg.angular.z)
            self.cmd_time = time.time()

    def _on_init(self, msg):
        p = msg.pose.pose
        est = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        with self.lock:
            self.map_odom = compose(est, inverse(self._odom_pose()))
        self.get_logger().info(f'Nhận /initialpose {est}')

    # ------------------------------------------------------------------ mô phỏng
    def _physics(self):
        dt = 0.05
        with self.lock:
            teleop = time.time() - self.cmd_time < 0.5
            v, w = self.cmd if teleop else self.nav_cmd
            x, y, yaw = self.true
            nx, ny = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
            if not self.world.is_occupied(np.array([nx]), np.array([ny]))[0]:
                x, y = nx, ny
            self.true = (x, y, wrap(yaw + w * dt))
            odom_pose = self._odom_pose()
            mode = self._mode() if self.emulate else 'idle'
            if mode != self.prev_mode:
                if mode == 'loc':
                    self.map_odom = None            # AMCL thật chưa có TF map cho tới khi nhận /initialpose
                self.prev_mode = mode
            if mode == 'slam':
                self.map_odom = self.start          # khung map SLAM = khung "thế giới"
            elif mode == 'idle':
                self.map_odom = None
            map_odom = self.map_odom
        stamp = self.get_clock().now().to_msg()
        tfs = [self._tf('odom', 'base_footprint', odom_pose)]
        if map_odom is not None:
            tfs.append(self._tf('map', 'odom', map_odom))
        self.tf_pub.sendTransform(tfs)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'odom', 'base_footprint'
        od.pose.pose.position.x, od.pose.pose.position.y = odom_pose[0], odom_pose[1]
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = quat_from_yaw(odom_pose[2])
        od.twist.twist.linear.x, od.twist.twist.angular.z = v, w
        self.pub_odom.publish(od)

    def _scan_step(self):
        angles = np.linspace(-math.pi, math.pi, SCAN_BEAMS, endpoint=False)
        with self.lock:
            true = self.true
            mode = self._mode()
        ranges, r, px, py, any_hit, first = self.world.raycast(true, angles)
        noisy = ranges + np.random.normal(0, 0.01, ranges.shape)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'
        msg.angle_min, msg.angle_max = -math.pi, math.pi
        msg.angle_increment = 2 * math.pi / SCAN_BEAMS
        msg.range_min, msg.range_max = 0.15, SCAN_MAX
        msg.ranges = np.where(np.isfinite(noisy), noisy, float('inf')).astype(np.float32).tolist()
        self.pub_scan.publish(msg)

        if mode == 'slam' and self.emulate:   # vẽ dần: ô dọc tia = trống, ô trúng = vật cản
            n_free = np.where(any_hit, first, len(r))
            mask = np.arange(len(r))[None, :] < n_free[:, None]
            iy, ix = self.world.cell(px[mask], py[mask])
            ok = (iy >= 0) & (iy < self.world.h) & (ix >= 0) & (ix < self.world.w)
            self.slam_grid[iy[ok], ix[ok]] = 0
            hb = np.nonzero(any_hit)[0]
            hy, hx = self.world.cell(px[hb, first[hb]], py[hb, first[hb]])
            ok = (hy >= 0) & (hy < self.world.h) & (hx >= 0) & (hx < self.world.w)
            self.slam_grid[hy[ok], hx[ok]] = 100

    def _slow_step(self):
        try:
            self.graph = {n for n, _ in self.get_node_names_and_namespaces()}
        except Exception:
            self.graph = set()
        mode = self._mode() if self.emulate else 'off'
        if mode == 'slam':
            m = OccupancyGrid()
            m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), 'map'
            m.info.resolution, m.info.width, m.info.height = self.world.res, self.world.w, self.world.h
            m.info.origin.position.x, m.info.origin.position.y = self.world.origin
            m.info.origin.orientation.w = 1.0
            m.data = self.slam_grid.ravel().tolist()
            self.pub_map.publish(m)
        elif mode == 'idle':
            self.slam_grid[:] = -1   # phiên quét mới bắt đầu từ bản đồ trống

        want_nav = self.emulate and 'bt_navigator' in self.graph and NavigateToPose is not None
        if want_nav and self.action is None:
            self.action = ActionServer(self, NavigateToPose, 'navigate_to_pose', self._execute,
                                       goal_callback=lambda g: GoalResponse.ACCEPT,
                                       cancel_callback=lambda h: CancelResponse.ACCEPT,
                                       callback_group=self._cbg)
        elif not want_nav and self.action is not None:
            self.action.destroy()
            self.action = None
            with self.lock:
                self.nav_cmd = (0.0, 0.0)

    # --------------------------------------------------------------------- Nav2
    def _execute(self, goal_handle):
        p = goal_handle.request.pose.pose
        goal = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        result = NavigateToPose.Result()
        started = time.time()
        while rclpy.ok():
            with self.lock:
                est = self._est_pose()
            if est is None:                          # chưa có initialpose -> Nav2 thật cũng không chạy nổi
                with self.lock:
                    self.nav_cmd = (0.0, 0.0)
                goal_handle.abort()
                return result
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.nav_cmd = (0.0, 0.0)
                    self.plan_pts = []
                goal_handle.canceled()
                return result
            dx, dy = goal[0] - est[0], goal[1] - est[1]
            dist = math.hypot(dx, dy)
            if dist < 0.12:
                # xoay về hướng đích rồi kết thúc
                dyaw = wrap(goal[2] - est[2])
                if abs(dyaw) < 0.1:
                    break
                cmd = (0.0, max(-0.8, min(0.8, 1.5 * dyaw)))
            else:
                dyaw = wrap(math.atan2(dy, dx) - est[2])
                cmd = (0.0, max(-0.9, min(0.9, 2.0 * dyaw))) if abs(dyaw) > 0.5 else \
                    (min(0.3, 0.5 * dist + 0.05), max(-0.9, min(0.9, 2.0 * dyaw)))
            with self.lock:
                self.nav_cmd = cmd
                self.plan_pts = [(est[0], est[1]), (goal[0], goal[1])]
            fb = NavigateToPose.Feedback()
            fb.distance_remaining = float(dist)
            fb.estimated_time_remaining.sec = int(dist / 0.25)
            goal_handle.publish_feedback(fb)
            self._publish_plan()
            if time.time() - started > 300:
                goal_handle.abort()
                return result
            time.sleep(0.1)
        with self.lock:
            self.nav_cmd = (0.0, 0.0)
            self.plan_pts = []
        goal_handle.succeed()
        return result

    def _publish_plan(self):
        with self.lock:
            pts = list(self.plan_pts)
        msg = Path()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'map'
        for (x, y) in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_plan.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
