"""Node ROS2 làm cầu nối giữa web và robot.

Chạy trong 1 executor riêng (thread nền); server aiohttp gọi các phương thức
công khai bên dưới từ thread khác nên mọi trạng thái dùng chung đều qua self.lock.
Quy ước khóa (tránh deadlock): TaskManager.lock -> RosBridge.lock, không bao giờ ngược lại.
"""
import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import BatteryState, LaserScan
from tf2_ros import Buffer, TransformListener

from a3_web.maps import encode_indexed_png, grid_to_indices
from a3_web.tasks import TaskManager

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # thiếu Nav2 -> web vẫn chạy (teleop, SLAM), chỉ không điều hướng được
    NavigateToPose = None

MAX_SCAN_POINTS = 360
MAX_PLAN_POINTS = 150
TELEOP_TIMEOUT_S = 0.4
TF_MAX_AGE_S = 3.0


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def clamp(v, lim):
    return max(-lim, min(lim, v))


DEFAULT_PARAMS = {
    # web server
    'host': '0.0.0.0',
    'port': 8080,
    'maps_dir': '',
    'config_dir': '~/.a3_web',
    'autostart_bringup': True,
    # lệnh các stack (placeholder: {map_yaml}, {controller})
    'cmd_bringup': 'ros2 launch a3_bringup bringup.launch.py',
    'cmd_slam': 'ros2 launch atlas_slam slam.launch.py rviz:=false',
    'cmd_nav': 'ros2 launch atlas_slam navigation.launch.py map:={map_yaml} controller:={controller} rviz:=false',
    # node đặc trưng để biết stack đã "ready" / đang chạy ngoài web
    'ready_nodes_bringup': ['serial_bridge_node'],
    'ready_nodes_slam': ['slam_toolbox'],
    'ready_nodes_nav': ['bt_navigator', 'amcl'],
    'controller': 'dwb',
    # frame + topic
    'map_frame': 'map',
    'odom_frame': 'odom',
    'base_frame': 'base_footprint',
    'scan_topic': '/scan',
    'odom_topic': '/odom',
    'map_topic': '/map',
    'plan_topic': '/plan',
    'cmd_vel_topic': '/cmd_vel',
    'initialpose_topic': '/initialpose',
    'battery_topic': '/battery_state',
    'nav_action': 'navigate_to_pose',
    # an toàn teleop qua web (tối đa)
    'max_linear': 0.5,
    'max_angular': 1.5,
    # kích thước thân robot để vẽ đúng tỉ lệ trên bản đồ (m) - khớp footprint trong nav2 params
    'robot_length': 0.50,
    'robot_width': 0.44,
}


class RosBridge(Node):
    def __init__(self, **param_overrides):
        super().__init__('a3_web_server')
        for key, default in DEFAULT_PARAMS.items():
            self.declare_parameter(key, default)
        self.cfg = {k: self.get_parameter(k).value for k in DEFAULT_PARAMS}
        self.cfg.update(param_overrides)

        self.lock = threading.RLock()
        self.estop = False
        self._cmds = deque()

        # --- trạng thái nhận từ ROS
        self.pose = {'valid': False, 'frame': '', 'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.speed = {'linear': 0.0, 'angular': 0.0}
        self.battery = None
        self._scan = None            # LaserScan mới nhất chưa xử lý
        self._scan_times = deque(maxlen=20)
        self.scan_out = {'frame': '', 'points': [], 'min_dist': None}
        self.plan = []
        self._plan_time = 0.0
        self.live_map = None         # {'version', 'width', ..., 'png': bytes}
        self._map_version = 0
        self._node_names = set()
        self._node_names_time = 0.0

        # --- teleop
        self._teleop_active = False
        self._teleop_last = 0.0

        # --- điều hướng
        self._goal_seq = 0
        self.nav = self._new_nav_state()

        # --- ROS I/O
        c = self.cfg
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.pub_cmd = self.create_publisher(Twist, c['cmd_vel_topic'], 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, c['initialpose_topic'], 10)
        self.create_subscription(LaserScan, c['scan_topic'], self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, c['odom_topic'], self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Path, c['plan_topic'], self._on_plan, qos_profile_sensor_data)
        self.create_subscription(BatteryState, c['battery_topic'], self._on_battery, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, c['map_topic'], self._on_map, latched)
        self.nav_client = ActionClient(self, NavigateToPose, c['nav_action']) if NavigateToPose else None

        self.tasks = TaskManager(self)
        self.create_timer(0.1, self._tick)

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _new_nav_state():
        return {'state': 'idle', 'goal_id': 0, 'goal': None, 'source': None, 'task_id': None,
                'distance_remaining': None, 'eta': None, 'handle': None,
                'cancel_requested': False, 'updated': time.time()}

    def nav_ready(self):
        return bool(self.nav_client and self.nav_client.server_is_ready())

    def nav_info(self):
        with self.lock:
            n = self.nav
            return {'state': n['state'], 'goal_id': n['goal_id'], 'source': n['source'], 'task_id': n['task_id']}

    def has_service(self, name):
        """Service có trong graph ROS không (truy vấn tức thời, không chờ timeout như `ros2 service call`)."""
        try:
            return any(n == name for n, _ in self.get_service_names_and_types())
        except Exception:
            return False

    def node_names(self):
        with self.lock:
            return set(self._node_names)

    # ---------------------------------------------------------------- callbacks
    def _on_scan(self, msg):
        with self.lock:
            self._scan = msg
            self._scan_times.append(time.time())

    def _on_odom(self, msg):
        with self.lock:
            self.speed = {'linear': msg.twist.twist.linear.x, 'angular': msg.twist.twist.angular.z}

    def _on_battery(self, msg):
        pct = msg.percentage
        if pct is not None and math.isfinite(pct):
            pct = pct * 100.0 if pct <= 1.0 else pct
        else:
            pct = None
        with self.lock:
            self.battery = {'percentage': pct, 'voltage': msg.voltage if math.isfinite(msg.voltage) else None}

    def _on_plan(self, msg):
        poses = msg.poses
        if not poses:
            pts = []
        else:
            step = max(1, len(poses) // MAX_PLAN_POINTS)
            pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in poses[::step]]
            last = poses[-1].pose.position
            pts.append([round(last.x, 3), round(last.y, 3)])
        with self.lock:
            self.plan = pts
            self._plan_time = time.time()

    def _on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0 or len(msg.data) != w * h:
            return
        png = encode_indexed_png(grid_to_indices(msg.data, w, h))
        with self.lock:
            self._map_version += 1
            self.live_map = {
                'version': self._map_version, 'width': w, 'height': h,
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x, 'origin_y': msg.info.origin.position.y,
                'frame': msg.header.frame_id or 'map', 'png': png,
            }

    def clear_live_map(self):
        with self.lock:
            self.live_map = None

    # -------------------------------------------------------------------- tick
    def _lookup(self, target, source, max_age=None):
        """TF mới nhất giữa 2 frame. max_age: bỏ TF cũ hơn ngần này giây - khi SLAM/AMCL tắt,
        map->odom cuối cùng vẫn nằm trong buffer mãi mãi và sẽ cho pose "ma" nếu không chặn."""
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None
        if max_age is not None:
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
            if age > max_age:
                return None
        return t.transform

    def _tick(self):
        self._process_cmds()
        now = time.time()

        # E-STOP: liên tục ép 0 lên /cmd_vel; teleop mất tín hiệu -> dừng
        if self.estop:
            self.pub_cmd.publish(Twist())
        elif self._teleop_active and now - self._teleop_last > TELEOP_TIMEOUT_S:
            self.pub_cmd.publish(Twist())
            self._teleop_active = False

        # pose: ưu tiên frame map (có AMCL/SLAM), không có thì odom
        tf, frame = None, ''
        for fr in (self.cfg['map_frame'], self.cfg['odom_frame']):
            tf = self._lookup(fr, self.cfg['base_frame'], max_age=TF_MAX_AGE_S if fr == self.cfg['map_frame'] else None)
            if tf is not None:
                frame = fr
                break
        with self.lock:
            if tf is None:
                self.pose = {'valid': False, 'frame': '', 'x': 0.0, 'y': 0.0, 'theta': 0.0}
            else:
                self.pose = {'valid': True, 'frame': frame, 'x': tf.translation.x,
                             'y': tf.translation.y, 'theta': yaw_from_quat(tf.rotation)}
            scan, self._scan = self._scan, None
            frame_for_scan = self.pose['frame']
        if scan is not None:
            self._process_scan(scan, frame_for_scan)

        self.tasks.step()

        if now - self._node_names_time > 1.0:
            try:
                names = {n for n, _ in self.get_node_names_and_namespaces()}
            except Exception:
                names = set()
            with self.lock:
                self._node_names = names
                self._node_names_time = now

    def _process_scan(self, scan, frame):
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
        ok = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        min_dist = float(ranges[ok].min()) if ok.any() else None
        points = []
        tf = self._lookup(frame, scan.header.frame_id, max_age=TF_MAX_AGE_S) if frame else None
        if tf is not None and ok.any():
            r, a = ranges[ok], angles[ok]
            step = max(1, len(r) // MAX_SCAN_POINTS)
            r, a = r[::step], a[::step]
            yaw = yaw_from_quat(tf.rotation)
            lx, ly = r * np.cos(a), r * np.sin(a)
            gx = tf.translation.x + lx * math.cos(yaw) - ly * math.sin(yaw)
            gy = tf.translation.y + lx * math.sin(yaw) + ly * math.cos(yaw)
            points = np.round(np.stack([gx, gy], axis=1), 2).tolist()
        with self.lock:
            self.scan_out = {'frame': frame, 'points': points, 'min_dist': min_dist}

    # ------------------------------------------------------------------ lệnh
    def _process_cmds(self):
        while True:
            with self.lock:
                if not self._cmds:
                    return
                cmd = self._cmds.popleft()
            try:
                if cmd[0] == 'goal':
                    _, gid, x, y, theta = cmd
                    goal = NavigateToPose.Goal()
                    goal.pose = PoseStamped()
                    goal.pose.header.frame_id = self.cfg['map_frame']
                    goal.pose.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
                    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quat_from_yaw(float(theta))
                    fut = self.nav_client.send_goal_async(goal, feedback_callback=lambda fb, g=gid: self._on_feedback(g, fb))
                    fut.add_done_callback(lambda f, g=gid: self._on_goal_response(g, f))
                elif cmd[0] == 'cancel':
                    cmd[1].cancel_goal_async()
            except Exception as e:  # không để 1 lệnh lỗi làm chết timer
                self.get_logger().error(f'Lỗi xử lý lệnh {cmd[0]}: {e}')
                with self.lock:
                    if cmd[0] == 'goal' and self.nav['goal_id'] == cmd[1]:
                        self.nav['state'] = 'rejected'

    def _on_feedback(self, gid, fb_msg):
        fb = fb_msg.feedback
        with self.lock:
            if self.nav['goal_id'] != gid:
                return
            self.nav['distance_remaining'] = float(fb.distance_remaining)
            eta = fb.estimated_time_remaining
            self.nav['eta'] = eta.sec + eta.nanosec * 1e-9

    def _on_goal_response(self, gid, fut):
        try:
            handle = fut.result()
        except Exception as e:
            self.get_logger().error(f'send_goal lỗi: {e}')
            handle = None
        with self.lock:
            if handle is None or not handle.accepted:
                if self.nav['goal_id'] == gid:
                    self.nav['state'] = 'rejected'
                    self.nav['updated'] = time.time()
                return
            if self.nav['goal_id'] != gid:
                return  # goal cũ đã bị thay thế (Nav2 tự preempt)
            self.nav['handle'] = handle
            self.nav['state'] = 'canceling' if self.nav['cancel_requested'] else 'active'
            self.nav['updated'] = time.time()
            if self.nav['cancel_requested']:
                self._cmds.append(('cancel', handle))
        handle.get_result_async().add_done_callback(lambda f, g=gid: self._on_result(g, f))

    def _on_result(self, gid, fut):
        try:
            status = fut.result().status
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        state = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                 GoalStatus.STATUS_CANCELED: 'canceled'}.get(status, 'aborted')
        with self.lock:
            if self.nav['goal_id'] != gid:
                return
            self.nav['state'] = state
            self.nav['handle'] = None
            self.nav['updated'] = time.time()
            self.plan = []

    # ------------------------------------------------------------ API cho server
    def send_goal(self, x, y, theta, name=None, source='manual', task_id=None):
        if self.estop:
            return False, 'E-STOP đang bật - nhả E-STOP trước'
        if self.nav_client is None:
            return False, 'Thiếu gói nav2_msgs nên không điều hướng được'
        if not self.nav_ready():
            return False, 'Nav2 chưa sẵn sàng - hãy vào chế độ Điều hướng và đợi Nav2 khởi động xong'
        with self.lock:
            self._goal_seq += 1
            gid = self._goal_seq
            self.nav = self._new_nav_state()
            self.nav.update(state='sending', goal_id=gid, source=source, task_id=task_id,
                            goal={'x': x, 'y': y, 'theta': theta, 'name': name})
            self._cmds.append(('goal', gid, x, y, theta))
        return True, gid

    def cancel_goal(self):
        with self.lock:
            n = self.nav
            if n['state'] not in ('sending', 'active'):
                return False, 'Không có mục tiêu nào đang chạy'
            n['state'] = 'canceling'
            n['cancel_requested'] = True
            if n['handle'] is not None:
                self._cmds.append(('cancel', n['handle']))
        return True, 'Đã gửi yêu cầu hủy'

    def set_teleop(self, linear, angular):
        if self.estop:
            return False
        lin = clamp(float(linear), self.cfg['max_linear'])
        ang = clamp(float(angular), self.cfg['max_angular'])
        if lin != 0.0 or ang != 0.0:
            with self.lock:
                navigating = self.nav['state'] in ('sending', 'active')
            if navigating:
                self.cancel_goal()   # người lái tay thì hủy điều hướng tự động
        msg = Twist()
        msg.linear.x, msg.angular.z = lin, ang
        self.pub_cmd.publish(msg)
        self._teleop_active = bool(lin or ang)
        self._teleop_last = time.time()
        return True

    def set_estop(self, on):
        with self.lock:
            self.estop = bool(on)
        if on:
            self.cancel_goal()
            self._teleop_active = False
            self.pub_cmd.publish(Twist())

    def publish_initialpose(self, x, y, theta):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.cfg['map_frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = quat_from_yaw(float(theta))
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        self.pub_init.publish(msg)

    def live_map_png(self):
        with self.lock:
            return self.live_map['png'] if self.live_map else None

    def snapshot(self):
        now = time.time()
        with self.lock:
            nav = self.nav
            navigating = nav['state'] in ('sending', 'active', 'canceling')
            hz = 0.0
            if len(self._scan_times) >= 2 and now - self._scan_times[-1] < 2.0:
                span = self._scan_times[-1] - self._scan_times[0]
                hz = (len(self._scan_times) - 1) / span if span > 0 else 0.0
            live = None
            if self.live_map:
                live = {k: v for k, v in self.live_map.items() if k != 'png'}
            out = {
                'estop': self.estop,
                'pose': dict(self.pose),
                'speed': dict(self.speed),
                'battery': dict(self.battery) if self.battery else None,
                'scan': {**self.scan_out, 'hz': round(hz, 1),
                         'age': round(now - self._scan_times[-1], 1) if self._scan_times else None},
                'plan': list(self.plan) if navigating else [],
                'live_map': live,
                'nav': {'state': nav['state'], 'goal': nav['goal'], 'source': nav['source'],
                        'task_id': nav['task_id'], 'goal_id': nav['goal_id'],
                        'distance_remaining': nav['distance_remaining'], 'eta': nav['eta'],
                        'server_ready': self.nav_ready()},
                'robot': {'length': self.cfg['robot_length'], 'width': self.cfg['robot_width']},
                'limits': {'max_linear': self.cfg['max_linear'], 'max_angular': self.cfg['max_angular']},
            }
        out['tasks'] = self.tasks.snapshot()   # ngoài lock: giữ đúng thứ tự khóa
        out['run'] = self.tasks.run_snapshot()
        return out
