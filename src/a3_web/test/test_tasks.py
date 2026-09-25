from a3_web.tasks import TaskManager


class FakeBridge:
    """Thay node ROS: ghi lại goal, để test tự điều khiển kết quả điều hướng."""

    def __init__(self):
        self.estop = False
        self.goals = []
        self.cancels = 0
        self.gid = 0
        self.nav = {'state': 'idle', 'goal_id': 0, 'source': None, 'task_id': None}
        self.ready = True

    def nav_info(self):
        return dict(self.nav)

    def send_goal(self, x, y, theta, name=None, source='manual', task_id=None):
        if not self.ready:
            return False, 'chưa sẵn sàng'
        self.gid += 1
        self.goals.append((name, source))
        self.nav = {'state': 'active', 'goal_id': self.gid, 'source': source, 'task_id': task_id}
        return True, self.gid

    def cancel_goal(self):
        self.cancels += 1
        self.nav['state'] = 'canceled'
        return True, ''

    def finish(self, state):
        self.nav['state'] = state


def targets(*names):
    return [{'name': n, 'x': i, 'y': 0.0, 'theta': 0.0} for i, n in enumerate(names)]


def route(name='R', loop=False):
    return {'id': 'r1', 'name': name, 'loop': loop}


def test_run_sequential_with_confirm():
    b = FakeBridge()
    tm = TaskManager(b)
    assert tm.start_run(route(), targets('A', 'B', 'C'))
    assert tm.start_run(route(), targets('X')) is False        # đang chạy: không chạy chồng
    tm.step()
    assert [g[0] for g in b.goals] == ['A']
    b.finish('succeeded'); tm.step()
    assert tm.snapshot()[0]['status'] == 'arrived'
    tm.step()
    assert len(b.goals) == 1                                     # chờ xác nhận, không đi tiếp
    snap = tm.run_snapshot()
    assert snap['total'] == 3 and snap['done'] == 0 and snap['current']['step'] == 1
    assert tm.confirm(tm.snapshot()[0]['id'])
    tm.step()
    assert [g[0] for g in b.goals] == ['A', 'B']
    assert tm.run_snapshot()['done'] == 1


def test_run_without_confirm_finishes_and_clears():
    b = FakeBridge()
    tm = TaskManager(b)
    tm.require_confirm = False
    tm.start_run(route(), targets('A', 'B'))
    for _ in range(2):
        tm.step(); b.finish('succeeded'); tm.step()
    tm.step()
    assert [t['status'] for t in tm.snapshot()] == ['done', 'done']
    assert tm.run_snapshot() is None                             # lộ trình đã xong


def test_loop_restarts_cycle():
    b = FakeBridge()
    tm = TaskManager(b)
    tm.require_confirm = False
    tm.start_run(route(loop=True), targets('A', 'B'))
    for _ in range(2):
        tm.step(); b.finish('succeeded'); tm.step()
    tm.step()                                                    # hết vòng 1 -> nạp vòng 2
    snap = tm.run_snapshot()
    assert snap['cycle'] == 2 and snap['done'] == 0
    tm.step()
    assert [g[0] for g in b.goals] == ['A', 'B', 'A']
    tm.stop_run()
    assert tm.run_snapshot() is None and b.cancels == 1


def test_failure_aborts_remaining_steps():
    b = FakeBridge()
    tm = TaskManager(b)
    tm.start_run(route(), targets('A', 'B', 'C'))
    tm.step(); b.finish('aborted'); tm.step()
    assert [t['status'] for t in tm.snapshot()] == ['failed', 'cancelled', 'cancelled']
    assert tm.run_snapshot() is None
    tm.step()
    assert len(b.goals) == 1


def test_waits_for_nav_ready_and_estop():
    b = FakeBridge()
    b.ready = False
    tm = TaskManager(b)
    tm.start_run(route(), targets('A'))
    tm.step()
    assert tm.snapshot()[0]['status'] == 'queued'                # Nav2 chưa sẵn sàng -> chờ, không mất bước
    b.ready = True
    b.estop = True
    tm.step()
    assert tm.snapshot()[0]['status'] == 'queued'                # E-STOP -> không gửi goal
    b.estop = False
    tm.step()
    assert tm.snapshot()[0]['status'] == 'going'


def test_return_home_after_route():
    b = FakeBridge()
    tm = TaskManager(b)
    tm.require_confirm = False
    tm.return_home = True
    tm.home_provider = lambda: {'name': 'Bếp', 'x': 9.0, 'y': 9.0, 'theta': 0.0}
    tm.start_run(route(), targets('A'))
    tm.step(); b.finish('succeeded'); tm.step(); tm.step()
    assert b.goals[-1] == ('Bếp (về)', 'home')
