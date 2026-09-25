"""Quản lý các "stack" ROS2 (bringup / slam / nav) như subprocess `ros2 launch ...`.

Mỗi stack chạy trong process group riêng để tắt được cả cây node con. Trạng thái:

    stopped   chưa chạy
    starting  process do web bật đã chạy nhưng các node đặc trưng chưa xuất hiện
    running   process do web bật + node đặc trưng đã có trong graph
    external  KHÔNG do web bật nhưng thấy node đặc trưng (vd bạn đã chạy launch ở
              terminal khác) -> web không bật trùng và không tắt được
    stopping  đang tắt
    failed    process tự thoát bất thường
"""
import collections
import os
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time

# Tiến trình con kế thừa trạng thái "bỏ qua SIGINT" nếu web được chạy nền bằng `&` trong shell không
# tương tác - khi đó stop() bằng SIGINT vô tác dụng và phải chờ hết timeout. Bọc lệnh để luôn đặt lại
# SIGINT/SIGTERM về mặc định rồi exec lệnh thật (không dùng preexec_fn vì không an toàn khi có nhiều thread).
_EXEC_WRAPPER = ('import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); '
                 'signal.signal(signal.SIGTERM, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])')

EXTERNAL_COOLDOWN_S = 8.0   # sau khi tắt, node cũ còn treo trong graph 1 lúc -> đừng nhầm là "external"


class Stack:
    def __init__(self, name, cmd_template, ready_nodes):
        self.name = name
        self.cmd_template = cmd_template
        self.ready_nodes = list(ready_nodes)
        self.proc = None
        self.state = 'stopped'
        self.started_at = None
        self.exit_code = None
        self.cmd_line = ''
        self.cooldown_until = 0.0
        self.stop_requested = False
        self.log = collections.deque(maxlen=400)
        self.lock = threading.RLock()


class ProcessManager:
    def __init__(self, stacks, node_names_fn, log_dir=None):
        """stacks: {name: {'cmd': template, 'ready_nodes': [...]}}"""
        self.stacks = {n: Stack(n, c['cmd'], c.get('ready_nodes', [])) for n, c in stacks.items()}
        self._node_names = node_names_fn
        self.log_dir = os.path.expanduser(log_dir) if log_dir else None
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)

    # ------------------------------------------------------------ nội bộ
    def _get(self, name):
        if name not in self.stacks:
            raise KeyError(f'Stack không tồn tại: {name}')
        return self.stacks[name]

    def _nodes_present(self, stack):
        names = self._node_names()
        return any(n in names for n in stack.ready_nodes) if stack.ready_nodes else False

    def _refresh(self, stack):
        """Cập nhật trạng thái theo process + graph. Gọi khi đã giữ stack.lock."""
        proc = stack.proc
        if proc is not None and proc.poll() is not None:
            stack.exit_code = proc.returncode
            stack.proc = None
            if stack.stop_requested or proc.returncode in (0, -signal.SIGINT, -signal.SIGTERM):
                stack.state = 'stopped'
            else:
                stack.state = 'failed'
            stack.cooldown_until = time.time() + EXTERNAL_COOLDOWN_S
        if stack.proc is not None:
            if stack.state != 'stopping':
                stack.state = 'running' if self._nodes_present(stack) else 'starting'
        else:
            if stack.state in ('stopped', 'external'):
                external = time.time() >= stack.cooldown_until and self._nodes_present(stack)
                stack.state = 'external' if external else 'stopped'

    def _pump_logs(self, stack, proc, log_file):
        try:
            for line in proc.stdout:
                line = line.rstrip('\n')
                stack.log.append(line)
                if log_file:
                    log_file.write(line + '\n')
                    log_file.flush()
        finally:
            if log_file:
                log_file.close()

    # -------------------------------------------------------------- API
    def start(self, name, **fmt):
        stack = self._get(name)
        with stack.lock:
            self._refresh(stack)
            if stack.state == 'external':
                return False, f'Stack "{name}" đang chạy ngoài web (terminal khác) - tắt nó trước'
            if stack.state in ('starting', 'running', 'stopping'):
                return False, f'Stack "{name}" đã chạy (trạng thái: {stack.state})'
            try:
                quoted = {k: shlex.quote(str(v)) for k, v in fmt.items()}
                argv = shlex.split(stack.cmd_template.format(**quoted))
            except (KeyError, ValueError, IndexError) as e:
                return False, f'Lệnh của stack "{name}" sai cú pháp: {e}'
            if not argv or shutil.which(argv[0]) is None:
                stack.state = 'failed'
                return False, f'Không chạy được lệnh: không tìm thấy "{argv[0] if argv else ""}"'
            env = dict(os.environ, PYTHONUNBUFFERED='1')
            try:
                proc = subprocess.Popen(
                    [sys.executable, '-c', _EXEC_WRAPPER, *argv], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, errors='replace', env=env, start_new_session=True)
            except (OSError, ValueError) as e:
                stack.state = 'failed'
                return False, f'Không chạy được lệnh: {e}'
            stack.proc = proc
            stack.state = 'starting'
            stack.started_at = time.time()
            stack.exit_code = None
            stack.stop_requested = False
            stack.cmd_line = ' '.join(argv)
            stack.log.clear()
            stack.log.append(f'$ {stack.cmd_line}')
            log_file = None
            if self.log_dir:
                try:
                    log_file = open(os.path.join(self.log_dir, f'{name}.log'), 'w')
                except OSError:
                    log_file = None
            threading.Thread(target=self._pump_logs, args=(stack, proc, log_file), daemon=True).start()
        return True, f'Đã khởi động "{name}"'

    def stop(self, name, sigint_timeout=12.0, term_timeout=5.0):
        """Tắt stack (chặn cho tới khi thoát xong - gọi từ thread nền)."""
        stack = self._get(name)
        with stack.lock:
            self._refresh(stack)
            proc = stack.proc
            if proc is None:
                if stack.state == 'external':
                    return False, f'Stack "{name}" chạy ngoài web nên không tắt được từ đây'
                return True, f'Stack "{name}" đã dừng'
            stack.stop_requested = True
            stack.state = 'stopping'
        for sig, timeout in ((signal.SIGINT, sigint_timeout), (signal.SIGTERM, term_timeout), (signal.SIGKILL, 3.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            except OSError:
                pass
            try:
                proc.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                continue
        self._reap_group(proc.pid)
        with stack.lock:
            self._refresh(stack)
            stack.state = 'stopped'
            stack.cooldown_until = time.time() + EXTERNAL_COOLDOWN_S
        return True, f'Đã dừng "{name}"'

    @staticmethod
    def _reap_group(pgid, grace=6.0):
        """Tiến trình đầu (ros2 launch) thoát trước các node con của nó vài giây - đợi cả nhóm biến mất,
        node nào còn sót thì SIGKILL để không để lại node mồ côi chạy nền (gây trùng lặp ở lần bật sau)."""
        deadline = time.time() + grace
        while time.time() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                return
            except OSError:
                return
            time.sleep(0.1)
        try:
            os.killpg(pgid, signal.SIGKILL)
        except OSError:
            pass

    def is_active(self, name):
        """True nếu stack đang chạy (do web bật hoặc chạy ngoài)."""
        stack = self._get(name)
        with stack.lock:
            self._refresh(stack)
            return stack.state in ('starting', 'running', 'external', 'stopping')

    def status(self):
        out = {}
        for name, stack in self.stacks.items():
            with stack.lock:
                self._refresh(stack)
                out[name] = {
                    'state': stack.state,
                    'ready': stack.state in ('running', 'external') and self._nodes_present(stack),
                    'managed': stack.proc is not None,
                    'pid': stack.proc.pid if stack.proc else None,
                    'since': stack.started_at if stack.proc else None,
                    'exit_code': stack.exit_code,
                    'cmd': stack.cmd_line or stack.cmd_template,
                }
        return out

    def tail(self, name, n=120):
        stack = self._get(name)
        return list(stack.log)[-n:]

    def shutdown_all(self):
        """Tắt mọi stack do web bật, song song và nhanh (ros2 launch ngoài cho web ~10s trước khi SIGKILL)."""
        threads = []
        for name, stack in self.stacks.items():
            if stack.proc is not None:
                t = threading.Thread(target=self.stop, args=(name,), kwargs={'sigint_timeout': 3.5, 'term_timeout': 2.0},
                                     daemon=True)
                t.start()
                threads.append(t)
        for t in threads:
            t.join(timeout=12)
