"""Web server điều khiển robot A3: REST + WebSocket + file tĩnh, chạy cùng node ROS2.

    ros2 launch a3_web web.launch.py        # khuyến nghị
    ros2 run a3_web web_server              # chạy trực tiếp

AppState gom toàn bộ logic nghiệp vụ (chế độ SLAM/Nav, lưu bản đồ, cài đặt) không phụ
thuộc aiohttp để dễ test; các handler bên dưới chỉ là lớp mỏng chuyển HTTP <-> AppState.
"""
import asyncio
import json
import math
import os
import socket
import threading
import time
from pathlib import Path

import rclpy
from aiohttp import web
from rclpy.executors import SingleThreadedExecutor

from a3_web.maps import NAME_RE, MapError, MapStore
from a3_web.process_manager import ProcessManager
from a3_web.ros_bridge import RosBridge

CONTROLLERS = ('dwb', 'rpp', 'mppi')
ACTIVE_STATES = ('starting', 'running', 'external')


class ApiError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def default_maps_dir():
    """Ưu tiên src/a3_maps cạnh package (chạy từ source), sau đó ~/diff_robot_v3/src/a3_maps."""
    candidates = [Path(__file__).resolve().parents[2] / 'a3_maps',
                  Path('~/diff_robot_v3/src/a3_maps').expanduser(),
                  Path('~/a3_maps').expanduser()]
    for c in candidates:
        if c.is_dir():
            return str(c)
    return str(candidates[-1])


def static_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory('a3_web')) / 'static'
        if (p / 'index.html').is_file():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / 'static'


def lan_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))
            return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'


def scrub(obj):
    """Đổi NaN/Inf -> None đệ quy (JSON.parse của trình duyệt không nhận NaN)."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: scrub(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [scrub(v) for v in obj]
    return obj


def dumps(obj):
    try:
        return json.dumps(obj, separators=(',', ':'), allow_nan=False)
    except ValueError:
        return json.dumps(scrub(obj), separators=(',', ':'))


# =============================================================== nghiệp vụ
class AppState:
    def __init__(self, bridge):
        self.bridge = bridge
        cfg = bridge.cfg
        self.store = MapStore(cfg['maps_dir'] or default_maps_dir())
        self.config_dir = Path(os.path.expanduser(cfg['config_dir']))
        self.config_dir.mkdir(parents=True, exist_ok=True)
        self.settings = {'controller': cfg['controller'] if cfg['controller'] in CONTROLLERS else 'dwb',
                         'return_home': False, 'require_confirm': True, 'selected_map': None}
        self._load_settings()
        self.pm = ProcessManager({
            'bringup': {'cmd': cfg['cmd_bringup'], 'ready_nodes': cfg['ready_nodes_bringup']},
            'slam': {'cmd': cfg['cmd_slam'], 'ready_nodes': cfg['ready_nodes_slam']},
            'nav': {'cmd': cfg['cmd_nav'], 'ready_nodes': cfg['ready_nodes_nav']},
        }, bridge.node_names, log_dir=self.config_dir / 'logs')
        self.active_map = None
        self.mode_op = {'busy': False, 'message': '', 'error': None}
        self._mode_lock = threading.Lock()
        bridge.tasks.require_confirm = bool(self.settings['require_confirm'])
        bridge.tasks.return_home = bool(self.settings['return_home'])
        bridge.tasks.home_provider = self._home_waypoint

    # ------------------------------------------------------------ cài đặt
    @property
    def _settings_path(self):
        return self.config_dir / 'settings.json'

    def _load_settings(self):
        try:
            data = json.loads(self._settings_path.read_text())
        except (OSError, ValueError):
            return
        if data.get('controller') in CONTROLLERS:
            self.settings['controller'] = data['controller']
        for key in ('return_home', 'require_confirm'):
            if isinstance(data.get(key), bool):
                self.settings[key] = data[key]
        if isinstance(data.get('selected_map'), str):
            self.settings['selected_map'] = data['selected_map']

    def _save_settings(self):
        try:
            self._settings_path.write_text(json.dumps(self.settings, indent=2))
        except OSError:
            pass

    def update_settings(self, data):
        if 'controller' in data:
            if data['controller'] not in CONTROLLERS:
                raise ApiError(f'controller phải thuộc {CONTROLLERS}')
            self.settings['controller'] = data['controller']
        for key in ('return_home', 'require_confirm'):
            if key in data:
                self.settings[key] = bool(data[key])
        self.bridge.tasks.require_confirm = self.settings['require_confirm']
        self.bridge.tasks.return_home = self.settings['return_home']
        self._save_settings()

    # --------------------------------------------------------------- bản đồ
    @property
    def selected_map(self):
        name = self.settings.get('selected_map')
        return name if name and self.store.exists(name) else None

    def select_map(self, name):
        if self.derive_mode() != 'idle':
            raise ApiError('Hãy dừng quét/điều hướng trước khi đổi bản đồ đang xem')
        if not self.store.exists(name):
            raise ApiError(f'Không tìm thấy bản đồ "{name}"', 404)
        self.settings['selected_map'] = name
        self._save_settings()

    def _home_waypoint(self):
        name = self.selected_map
        if not name:
            return None
        wps = self.store.get_waypoints(name)
        for wtype in ('KITCHEN', 'DOCK'):
            for w in wps:
                if w.get('type') == wtype:
                    return w
        return None

    async def save_map(self, name, overwrite=False):
        if not NAME_RE.match(name or ''):
            raise ApiError('Tên bản đồ chỉ gồm chữ/số/_/- (tối đa 40 ký tự)')
        slam = self.pm.status()['slam']
        if slam['state'] not in ('running', 'external') or not slam['ready']:
            raise ApiError('SLAM chưa chạy/sẵn sàng - vào chế độ Quét bản đồ trước')
        if self.bridge.live_map_png() is None:
            raise ApiError('Chưa nhận được bản đồ từ SLAM - hãy lái robot đi một đoạn rồi thử lại')
        d = self.store.map_dir(name)
        existed = d.exists()
        if existed and not overwrite:
            raise ApiError(f'Bản đồ "{name}" đã tồn tại (chọn ghi đè hoặc đổi tên)', 409)
        d.mkdir(parents=True, exist_ok=True)
        prefix = d / name

        async def run(*argv, timeout):
            proc = await asyncio.create_subprocess_exec(
                *argv, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            try:
                out, _ = await asyncio.wait_for(proc.communicate(), timeout)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()
                return -1, 'quá thời gian chờ'
            return proc.returncode, out.decode(errors='replace')

        code, out = await run('ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix), timeout=60)
        if code != 0 or not (d / f'{name}.yaml').is_file():
            if not existed:
                import shutil
                shutil.rmtree(d, ignore_errors=True)
            tail = ' | '.join(out.strip().splitlines()[-3:])
            raise ApiError(f'Lưu bản đồ thất bại (map_saver_cli): {tail}', 500)

        posegraph = False
        # Serialize posegraph để sau này quét tiếp được (không bắt buộc). Chỉ gọi khi service thật sự
        # có trong graph - `ros2 service call` tới service không tồn tại sẽ đứng chờ tới hết timeout.
        if "'" not in str(prefix) and self.bridge.has_service('/slam_toolbox/serialize_map'):
            code, _ = await run('ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                                'slam_toolbox/srv/SerializePoseGraph', f"{{filename: '{prefix}'}}", timeout=25)
            posegraph = code == 0 and (d / f'{name}.posegraph').is_file()
        self.settings['selected_map'] = name
        self._save_settings()
        return {'name': name, 'posegraph': posegraph}

    def delete_map(self, name):
        if self.derive_mode() == 'navigation' and self.active_map == name:
            raise ApiError('Bản đồ đang được dùng để điều hướng - dừng điều hướng trước')
        self.store.delete(name)
        if self.settings.get('selected_map') == name:
            self.settings['selected_map'] = None
            self._save_settings()

    # ------------------------------------------------------------- chế độ
    def derive_mode(self, stacks=None):
        stacks = stacks or self.pm.status()
        if stacks['slam']['state'] in ACTIVE_STATES:
            return 'mapping'
        if stacks['nav']['state'] in ACTIVE_STATES:
            return 'navigation'
        return 'idle'

    def request_mode(self, mode, map_name=None):
        """Chuyển chế độ ở thread nền (tắt/bật stack mất vài giây)."""
        if mode not in ('idle', 'mapping', 'navigation'):
            raise ApiError('mode phải là idle | mapping | navigation')
        if mode == 'navigation':
            map_name = map_name or self.selected_map
            if not map_name or not self.store.exists(map_name):
                raise ApiError('Hãy chọn một bản đồ đã lưu để điều hướng')
        if not self._mode_lock.acquire(blocking=False):
            raise ApiError('Đang chuyển chế độ, vui lòng đợi', 409)
        self.mode_op = {'busy': True, 'message': 'Đang chuyển chế độ...', 'error': None}
        threading.Thread(target=self._do_mode, args=(mode, map_name), daemon=True).start()

    def _fail(self, msg):
        self.mode_op = {'busy': False, 'message': '', 'error': msg}

    def _do_mode(self, mode, map_name):
        try:
            self.bridge.cancel_goal()
            stacks = self.pm.status()
            if mode == 'idle':
                for name in ('nav', 'slam'):
                    self.mode_op['message'] = f'Đang tắt {name}...'
                    ok, msg = self.pm.stop(name)
                    if not ok:
                        return self._fail(msg)
                self.active_map = None
                self.bridge.clear_live_map()
            elif mode == 'mapping':
                self.mode_op['message'] = 'Đang tắt Nav2...'
                ok, msg = self.pm.stop('nav')
                if not ok:
                    return self._fail(msg)
                self.active_map = None
                if stacks['slam']['state'] not in ACTIVE_STATES:
                    self.bridge.clear_live_map()
                    self.mode_op['message'] = 'Đang khởi động SLAM...'
                    ok, msg = self.pm.start('slam')
                    if not ok:
                        return self._fail(msg)
            else:  # navigation
                self.mode_op['message'] = 'Đang tắt SLAM...'
                ok, msg = self.pm.stop('slam')
                if not ok:
                    return self._fail(msg)
                self.bridge.clear_live_map()
                if stacks['nav']['state'] in ACTIVE_STATES and self.active_map != map_name:
                    self.mode_op['message'] = 'Đang tắt Nav2 để đổi bản đồ...'
                    ok, msg = self.pm.stop('nav')
                    if not ok:
                        return self._fail(msg)
                if not self.pm.is_active('nav'):
                    self.mode_op['message'] = 'Đang khởi động Nav2...'
                    ok, msg = self.pm.start('nav', map_yaml=str(self.store.yaml_path(map_name)),
                                            controller=self.settings['controller'])
                    if not ok:
                        return self._fail(msg)
                self.active_map = map_name
                self.settings['selected_map'] = map_name
                self._save_settings()
            self.mode_op = {'busy': False, 'message': '', 'error': None}
        except Exception as e:   # không để thread chết mà UI kẹt "busy"
            self._fail(f'Lỗi nội bộ: {e}')
        finally:
            self._mode_lock.release()

    def bringup(self, start):
        if start:
            ok, msg = self.pm.start('bringup')
        else:
            ok, msg = self.pm.stop('bringup')
        if not ok:
            raise ApiError(msg)
        return msg

    # ------------------------------------------------------------ trạng thái
    def state(self):
        s = self.bridge.snapshot()
        stacks = self.pm.status()
        mode = self.derive_mode(stacks)
        if mode != 'mapping' and s['live_map'] is not None:
            self.bridge.clear_live_map()
            s['live_map'] = None
        if mode != 'navigation':
            self.active_map = None
        s.update(type='state', t=time.time(), stacks=stacks, mode=mode,
                 selected_map=self.active_map if mode == 'navigation' and self.active_map else self.selected_map,
                 active_map=self.active_map if mode == 'navigation' else None,
                 mode_op=dict(self.mode_op), settings=dict(self.settings),
                 web={'maps_dir': str(self.store.dir)})
        return s

    def shutdown(self):
        self.pm.shutdown_all()


# ================================================================ HTTP layer
def create_app(state):
    routes = web.RouteTableDef()
    static = static_dir()

    async def body(request):
        try:
            data = await request.json()
        except (ValueError, json.JSONDecodeError):
            return {}
        return data if isinstance(data, dict) else {}

    def ok(**kw):
        return web.json_response({'ok': True, **kw}, dumps=dumps)

    @web.middleware
    async def errors(request, handler):
        try:
            resp = await handler(request)
        except ApiError as e:
            return web.json_response({'ok': False, 'error': str(e)}, status=e.status)
        except MapError as e:
            return web.json_response({'ok': False, 'error': str(e)}, status=400)
        except web.HTTPException:
            raise
        except Exception as e:
            return web.json_response({'ok': False, 'error': f'Lỗi máy chủ: {e}'}, status=500)
        if request.path.startswith('/static') or request.path == '/':
            resp.headers['Cache-Control'] = 'no-cache'
        return resp

    # ---------------------------------------------------------- trang + trạng thái
    @routes.get('/')
    async def index(request):
        return web.FileResponse(static / 'index.html')

    @routes.get('/api/status')
    async def status(request):
        return web.json_response(state.state(), dumps=dumps)

    # ------------------------------------------------------------------ WebSocket
    @routes.get('/ws')
    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=15)
        await ws.prepare(request)
        request.app['clients'].add(ws)
        try:
            await ws.send_str(dumps(state.state()))
            async for msg in ws:
                if msg.type != web.WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except ValueError:
                    continue
                if data.get('type') == 'teleop':
                    try:
                        state.bridge.set_teleop(float(data.get('linear', 0)), float(data.get('angular', 0)))
                    except (TypeError, ValueError):
                        pass
        finally:
            request.app['clients'].discard(ws)
            # mất kết nối trình duyệt lúc đang bấm giữ nút -> dừng ngay
            state.bridge.set_teleop(0.0, 0.0)
        return ws

    # ------------------------------------------------------------------ bản đồ
    @routes.get('/api/maps')
    async def maps_list(request):
        return ok(maps=state.store.list_maps(), selected=state.selected_map,
                  active=state.active_map, maps_dir=str(state.store.dir))

    @routes.post('/api/maps/select')
    async def maps_select(request):
        state.select_map((await body(request)).get('name'))
        return ok(selected=state.selected_map)

    @routes.post('/api/maps/save')
    async def maps_save(request):
        data = await body(request)
        result = await state.save_map(str(data.get('name', '')).strip(), bool(data.get('overwrite')))
        return ok(**result)

    @routes.delete('/api/maps/{name}')
    async def maps_delete(request):
        state.delete_map(request.match_info['name'])
        return ok()

    @routes.get('/api/maps/{name}/image.png')
    async def map_image(request):
        return web.Response(body=state.store.image_png(request.match_info['name']),
                            content_type='image/png', headers={'Cache-Control': 'max-age=60'})

    @routes.get('/api/maps/{name}/download')
    async def map_download(request):
        name = request.match_info['name']
        return web.Response(body=state.store.zip_bytes(name), content_type='application/zip',
                            headers={'Content-Disposition': f'attachment; filename="{name}.zip"'})

    @routes.get('/api/live_map.png')
    async def live_map(request):
        png = state.bridge.live_map_png()
        if png is None:
            raise web.HTTPNotFound()
        return web.Response(body=png, content_type='image/png', headers={'Cache-Control': 'no-store'})

    # ---------------------------------------------------------------- waypoint
    @routes.get('/api/maps/{name}/waypoints')
    async def wp_list(request):
        name = request.match_info['name']
        if not state.store.exists(name):
            raise ApiError(f'Không tìm thấy bản đồ "{name}"', 404)
        return ok(waypoints=state.store.get_waypoints(name))

    @routes.post('/api/maps/{name}/waypoints')
    async def wp_add(request):
        return ok(waypoint=state.store.add_waypoint(request.match_info['name'], await body(request)))

    @routes.put('/api/maps/{name}/waypoints/{wid}')
    async def wp_update(request):
        return ok(waypoint=state.store.update_waypoint(
            request.match_info['name'], request.match_info['wid'], await body(request)))

    @routes.delete('/api/maps/{name}/waypoints/{wid}')
    async def wp_delete(request):
        state.store.delete_waypoint(request.match_info['name'], request.match_info['wid'])
        return ok()

    # ------------------------------------------------------------- chế độ + stack
    @routes.post('/api/mode')
    async def set_mode(request):
        data = await body(request)
        state.request_mode(data.get('mode'), data.get('map'))
        return ok()

    @routes.post('/api/stacks/{name}/{action}')
    async def stack_action(request):
        name, action = request.match_info['name'], request.match_info['action']
        if name not in state.pm.stacks or action not in ('start', 'stop'):
            raise ApiError('Stack/hành động không hợp lệ', 404)
        if name == 'bringup':
            return ok(message=state.bringup(action == 'start'))
        if action == 'stop':
            state.request_mode('idle')
        elif name == 'slam':
            state.request_mode('mapping')
        else:
            state.request_mode('navigation', (await body(request)).get('map'))
        return ok()

    @routes.get('/api/stacks/{name}/log')
    async def stack_log(request):
        name = request.match_info['name']
        if name not in state.pm.stacks:
            raise ApiError('Stack không tồn tại', 404)
        return ok(lines=state.pm.tail(name, int(request.query.get('n', 150))))

    # ------------------------------------------------------------ điều hướng
    def resolve_target(data):
        if 'waypoint_id' in data:
            name = state.active_map or state.selected_map
            wp = state.store.find_waypoint(name, data['waypoint_id']) if name else None
            if wp is None:
                raise ApiError('Không tìm thấy điểm', 404)
            return {'name': wp['name'], 'x': wp['x'], 'y': wp['y'], 'theta': wp.get('theta', 0.0)}
        try:
            x, y, theta = float(data['x']), float(data['y']), float(data.get('theta', 0.0))
        except (KeyError, TypeError, ValueError):
            raise ApiError('Thiếu/sai toạ độ x, y, theta')
        if not all(math.isfinite(v) for v in (x, y, theta)):
            raise ApiError('Toạ độ không hợp lệ')
        return {'name': data.get('name'), 'x': x, 'y': y, 'theta': theta}

    @routes.post('/api/nav/goal')
    async def nav_goal(request):
        t = resolve_target(await body(request))
        good, res = state.bridge.send_goal(t['x'], t['y'], t['theta'], name=t['name'])
        if not good:
            raise ApiError(res, 409)
        return ok(goal_id=res)

    @routes.post('/api/nav/cancel')
    async def nav_cancel(request):
        good, msg = state.bridge.cancel_goal()
        if not good:
            raise ApiError(msg, 409)
        return ok(message=msg)

    @routes.post('/api/initialpose')
    async def initial_pose(request):
        t = resolve_target(await body(request))
        state.bridge.publish_initialpose(t['x'], t['y'], t['theta'])
        return ok()

    @routes.post('/api/estop')
    async def estop(request):
        state.bridge.set_estop(bool((await body(request)).get('on', True)))
        return ok(estop=state.bridge.estop)

    # ------------------------------------------------------------ nhiệm vụ
    @routes.post('/api/tasks')
    async def task_add(request):
        data = await body(request)
        target = resolve_target(data)
        return ok(task=state.bridge.tasks.add(target, data.get('note', '')))

    @routes.delete('/api/tasks/{tid}')
    async def task_delete(request):
        if not state.bridge.tasks.remove(request.match_info['tid']):
            raise ApiError('Không tìm thấy nhiệm vụ', 404)
        return ok()

    @routes.post('/api/tasks/{tid}/confirm')
    async def task_confirm(request):
        if not state.bridge.tasks.confirm(request.match_info['tid']):
            raise ApiError('Nhiệm vụ không ở trạng thái chờ xác nhận', 409)
        return ok()

    @routes.post('/api/tasks/clear')
    async def task_clear(request):
        state.bridge.tasks.clear(finished_only=bool((await body(request)).get('finished_only')))
        return ok()

    @routes.post('/api/tasks/pause')
    async def task_pause(request):
        state.bridge.tasks.set_paused(bool((await body(request)).get('paused', True)))
        return ok(paused=state.bridge.tasks.paused)

    @routes.post('/api/settings')
    async def settings(request):
        state.update_settings(await body(request))
        return ok(settings=state.settings)

    # -------------------------------------------------------------- vòng đời
    async def broadcast_loop(app):
        while True:
            try:
                if app['clients']:
                    text = dumps(state.state())
                    await asyncio.gather(*(ws.send_str(text) for ws in list(app['clients'])),
                                         return_exceptions=True)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f'[a3_web] broadcast lỗi: {e}', flush=True)
            await asyncio.sleep(0.1)

    async def autostart(app):
        await asyncio.sleep(4.0)   # đợi graph ROS thấy node chạy ngoài web (tránh chạy trùng bringup)
        if state.bridge.cfg['autostart_bringup'] and state.pm.status()['bringup']['state'] == 'stopped':
            ok_, msg = state.pm.start('bringup')
            print(f'[a3_web] autostart bringup: {msg}', flush=True)

    async def on_startup(app):
        app['tasks'] = [asyncio.create_task(broadcast_loop(app)), asyncio.create_task(autostart(app))]

    async def on_cleanup(app):
        for t in app['tasks']:
            t.cancel()
        await asyncio.get_running_loop().run_in_executor(None, state.shutdown)

    app = web.Application(middlewares=[errors])
    app['clients'] = set()
    app.add_routes(routes)
    app.router.add_static('/static', static, show_index=False, follow_symlinks=True)  # colcon --symlink-install cài file dạng symlink
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)  # để aiohttp tự xử lý SIGINT/SIGTERM
    bridge = RosBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()

    state = AppState(bridge)
    port = int(bridge.cfg['port'])
    print(f'[a3_web] Bản đồ: {state.store.dir}', flush=True)
    print(f'[a3_web] Mở trình duyệt: http://{lan_ip()}:{port}  (hoặc http://localhost:{port})', flush=True)
    try:
        web.run_app(create_app(state), host=bridge.cfg['host'], port=port, print=None)
    finally:
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
