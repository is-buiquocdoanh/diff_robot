"""Stack giả để thử web mà không cần robot thật: chỉ tạo các node ROS có ĐÚNG TÊN mà
web dùng để nhận biết stack đã "ready" (xem ready_nodes_* trong config/web_server.yaml).

    ros2 run a3_web fake_stack slam        # node: slam_toolbox
    ros2 run a3_web fake_stack nav         # node: bt_navigator, amcl, map_server
    ros2 run a3_web fake_stack bringup     # node: serial_bridge_node

fake_robot quan sát các node này để biết đang ở chế độ SLAM hay Nav mà phát dữ liệu tương ứng.
"""
import sys

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

NODES = {
    'bringup': ['serial_bridge_node'],
    'slam': ['slam_toolbox'],
    'nav': ['bt_navigator', 'amcl', 'map_server'],
}


def main(args=None):
    argv = list(sys.argv[1:] if args is None else args)
    name = next((a for a in argv if a in NODES), None)
    if name is None:
        print(f'Cách dùng: fake_stack <{"|".join(NODES)}> [--map <yaml>]', file=sys.stderr)
        return 2
    rclpy.init()
    nodes = [rclpy.create_node(n) for n in NODES[name]]
    print(f'[fake_stack] {name}: {", ".join(NODES[name])}', flush=True)
    executor = SingleThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
