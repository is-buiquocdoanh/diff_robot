"""Chạy lộ trình (chuỗi điểm theo thứ tự) bằng NavigateToPose.

Một lộ trình được nạp thành các bước (task) chạy tuần tự:

    queued -> going -> arrived (chờ xác nhận) -> done
                   \\-> failed | cancelled

- `require_confirm=False` bỏ bước chờ xác nhận (going -> done).
- `run['loop']=True`: hết 1 vòng thì tự chạy lại vòng mới; bước nào failed/cancelled thì DỪNG cả lộ trình
  (không nhảy cóc sang bước sau - robot có thể đang ở chỗ bất ngờ).
- `return_home=True`: khi lộ trình chạy xong (không lặp) và có bước vừa hoàn thành, tự gửi robot về điểm home
  (do `home_provider()` cung cấp - thường là điểm KITCHEN/DOCK đầu tiên).
- `add()` vẫn nhận nhiệm vụ đơn lẻ (API cũ), giao diện web hiện chỉ dùng lộ trình.
"""
import threading
import time

ACTIVE = ('queued', 'going', 'arrived')
KEEP_FINISHED = 60   # giữ tối đa bấy nhiêu bước đã kết thúc trong danh sách


class TaskManager:
    def __init__(self, bridge):
        self.bridge = bridge
        self.lock = threading.RLock()
        self.tasks = []
        self.run = None            # {'route_id','route_name','loop','targets','cycle'} khi đang chạy lộ trình
        self.paused = False
        self.require_confirm = True
        self.return_home = False
        self.home_provider = lambda: None
        self._seq = 0
        self._pending_home = False

    # ------------------------------------------------------------- tạo bước
    def _new_task(self, target, note='', **extra):
        self._seq += 1
        task = {
            'id': f'T-{self._seq:03d}',
            'name': target['name'],
            'x': target['x'], 'y': target['y'], 'theta': target.get('theta', 0.0),
            'note': str(note)[:80],
            'status': 'queued',
            'created': time.strftime('%H:%M'),
            'goal_id': None,
        }
        task.update(extra)
        self.tasks.append(task)
        return task

    def add(self, target, note=''):
        with self.lock:
            return dict(self._new_task(target, note))

    def _enqueue_cycle(self):
        run = self.run
        n = len(run['targets'])
        for i, t in enumerate(run['targets']):
            self._new_task(t, run['route_name'], route_id=run['route_id'], route_step=i + 1, route_total=n,
                           cycle=run['cycle'])

    # ------------------------------------------------------------- lộ trình
    def start_run(self, route, targets):
        """Bắt đầu chạy lộ trình. False nếu đang có bước chưa xong (chỉ chạy 1 lộ trình mỗi lần)."""
        with self.lock:
            if any(t['status'] in ACTIVE for t in self.tasks):
                return False
            self.tasks = []
            self.paused = False
            self._pending_home = False
            self.run = {'route_id': route['id'], 'route_name': route['name'], 'loop': bool(route.get('loop')),
                        'targets': list(targets), 'cycle': 1}
            self._enqueue_cycle()
            return True

    def stop_run(self):
        self.clear()

    def run_snapshot(self):
        with self.lock:
            if not self.run:
                return None
            cur = next((t for t in self.tasks if t['status'] in ('going', 'arrived')), None)
            return {
                'route_id': self.run['route_id'], 'route_name': self.run['route_name'],
                'loop': self.run['loop'], 'cycle': self.run['cycle'], 'paused': self.paused,
                'total': len(self.run['targets']),
                'done': sum(1 for t in self.tasks if t['status'] == 'done'),
                'current': ({'id': cur['id'], 'step': cur.get('route_step'), 'name': cur['name'],
                             'status': cur['status']} if cur else None),
            }

    # ------------------------------------------------------------- thao tác
    def _find(self, task_id):
        return next((t for t in self.tasks if t['id'] == task_id), None)

    def remove(self, task_id):
        with self.lock:
            task = self._find(task_id)
            if task is None:
                return False
            if task['status'] == 'going':
                self.bridge.cancel_goal()
            if task['status'] in ACTIVE:
                task['status'] = 'cancelled'
            else:
                self.tasks.remove(task)
            return True

    def clear(self, finished_only=False):
        with self.lock:
            if finished_only:
                self.tasks = [t for t in self.tasks if t['status'] in ACTIVE]
                return
            if any(t['status'] == 'going' for t in self.tasks):
                self.bridge.cancel_goal()
            for t in self.tasks:
                if t['status'] in ACTIVE:
                    t['status'] = 'cancelled'
            self.run = None
            self._pending_home = False

    def confirm(self, task_id):
        with self.lock:
            task = self._find(task_id)
            if task is None or task['status'] != 'arrived':
                return False
            task['status'] = 'done'
            self._pending_home = True
            return True

    def set_paused(self, paused):
        with self.lock:
            self.paused = bool(paused)

    # ---------------------------------------------------------------- vòng lặp
    def _abort_run(self):
        """Một bước thất bại/bị hủy -> hủy các bước còn chờ của lộ trình và kết thúc lượt chạy."""
        if self.run:
            for t in self.tasks:
                if t['status'] == 'queued':
                    t['status'] = 'cancelled'
            self.run = None

    def step(self):
        """Gọi định kỳ (10Hz) từ node ROS."""
        nav = self.bridge.nav_info()
        with self.lock:
            cur = next((t for t in self.tasks if t['status'] == 'going'), None)
            if cur is not None:
                if nav['goal_id'] != cur['goal_id']:
                    cur['status'] = 'cancelled'      # bị goal khác (thủ công) chiếm mất
                    self._abort_run()
                elif nav['state'] == 'succeeded':
                    if self.require_confirm:
                        cur['status'] = 'arrived'
                    else:
                        cur['status'] = 'done'
                        self._pending_home = True
                elif nav['state'] in ('aborted', 'rejected'):
                    cur['status'] = 'failed'
                    self._abort_run()
                elif nav['state'] == 'canceled':
                    cur['status'] = 'cancelled'
                    self._abort_run()
                self._trim()
                return

            if any(t['status'] == 'arrived' for t in self.tasks):
                return                                # chờ người xác nhận đã giao
            if self.paused or self.bridge.estop:
                return

            nxt = next((t for t in self.tasks if t['status'] == 'queued'), None)
            if nxt is not None:
                ok, gid = self.bridge.send_goal(nxt['x'], nxt['y'], nxt['theta'],
                                                name=nxt['name'], source='task', task_id=nxt['id'])
                if ok:
                    nxt['status'] = 'going'
                    nxt['goal_id'] = gid
                    self._pending_home = False
                return                                # thất bại (Nav2 chưa sẵn sàng) -> thử lại tick sau

            if self.run:                              # hết 1 vòng
                if self.run['loop']:
                    self.tasks = [t for t in self.tasks if t['status'] in ACTIVE]
                    self.run['cycle'] += 1
                    self._enqueue_cycle()
                    return
                self.run = None

            if self.return_home and self._pending_home:
                home = self.home_provider()
                self._pending_home = False
                if home:
                    self.bridge.send_goal(home['x'], home['y'], home.get('theta', 0.0),
                                          name=f'{home["name"]} (về)', source='home')

    def _trim(self):
        finished = [t for t in self.tasks if t['status'] not in ACTIVE]
        if len(finished) > KEEP_FINISHED:
            drop = set(id(t) for t in finished[:len(finished) - KEEP_FINISHED])
            self.tasks = [t for t in self.tasks if id(t) not in drop]

    def snapshot(self):
        with self.lock:
            return [{k: v for k, v in t.items() if k != 'goal_id'} for t in self.tasks]
