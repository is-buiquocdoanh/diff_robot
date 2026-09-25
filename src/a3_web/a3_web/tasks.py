"""Hàng đợi nhiệm vụ giao đồ: chạy tuần tự từng điểm bằng NavigateToPose.

Vòng đời 1 nhiệm vụ:  queued -> going -> arrived (chờ xác nhận) -> done
                                    \\-> failed | cancelled
`require_confirm=False` thì bỏ bước chờ xác nhận (going -> done).
`return_home=True`: khi hàng đợi hết và có nhiệm vụ vừa xong, tự gửi robot về
điểm home (do `home_provider()` cung cấp - thường là điểm KITCHEN/DOCK đầu tiên).
"""
import threading
import time

ACTIVE = ('queued', 'going', 'arrived')
KEEP_FINISHED = 30   # giữ tối đa bấy nhiêu nhiệm vụ đã kết thúc trong danh sách


class TaskManager:
    def __init__(self, bridge):
        self.bridge = bridge
        self.lock = threading.RLock()
        self.tasks = []
        self.paused = False
        self.require_confirm = True
        self.return_home = False
        self.home_provider = lambda: None
        self._seq = 0
        self._pending_home = False

    # ------------------------------------------------------------- thao tác
    def add(self, target, note=''):
        with self.lock:
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
            self.tasks.append(task)
            return dict(task)

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
    def step(self):
        """Gọi định kỳ (10Hz) từ node ROS."""
        nav = self.bridge.nav_info()
        with self.lock:
            cur = next((t for t in self.tasks if t['status'] == 'going'), None)
            if cur is not None:
                if nav['goal_id'] != cur['goal_id']:
                    cur['status'] = 'cancelled'      # bị goal khác (thủ công) chiếm mất
                elif nav['state'] == 'succeeded':
                    if self.require_confirm:
                        cur['status'] = 'arrived'
                    else:
                        cur['status'] = 'done'
                        self._pending_home = True
                elif nav['state'] in ('aborted', 'rejected'):
                    cur['status'] = 'failed'
                elif nav['state'] == 'canceled':
                    cur['status'] = 'cancelled'
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
