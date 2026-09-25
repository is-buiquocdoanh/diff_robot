import sys
import time

from a3_web.process_manager import ProcessManager

SLEEP = f'{sys.executable} -c "import time; print(\'hello\', flush=True); time.sleep(60)"'


def make(names_holder, cmd=SLEEP):
    return ProcessManager({'demo': {'cmd': cmd, 'ready_nodes': ['demo_node']}},
                          lambda: names_holder['names'])


def wait_for(pred, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_start_ready_stop():
    holder = {'names': set()}
    pm = make(holder)
    assert pm.status()['demo']['state'] == 'stopped'
    ok, _ = pm.start('demo')
    assert ok
    assert pm.status()['demo']['state'] == 'starting'
    assert wait_for(lambda: 'hello' in ' '.join(pm.tail('demo')))
    holder['names'] = {'demo_node'}
    st = pm.status()['demo']
    assert st['state'] == 'running' and st['ready'] and st['managed']
    assert pm.start('demo')[0] is False           # không chạy trùng
    ok, _ = pm.stop('demo', sigint_timeout=5)
    assert ok
    assert pm.status()['demo']['state'] == 'stopped'
    assert pm.status()['demo']['pid'] is None


def test_external_detection_and_cooldown():
    holder = {'names': {'demo_node'}}
    pm = make(holder)
    assert pm.status()['demo']['state'] == 'external'
    ok, msg = pm.start('demo')
    assert not ok and 'ngoài web' in msg
    assert pm.stop('demo')[0] is False
    holder['names'] = set()
    assert pm.status()['demo']['state'] == 'stopped'


def test_failed_process_and_bad_command():
    holder = {'names': set()}
    pm = make(holder, cmd=f'{sys.executable} -c "import sys; sys.exit(3)"')
    assert pm.start('demo')[0]
    assert wait_for(lambda: pm.status()['demo']['state'] == 'failed')
    assert pm.status()['demo']['exit_code'] == 3
    pm2 = make(holder, cmd='/không/tồn/tại/binary')
    ok, msg = pm2.start('demo')
    assert not ok and 'Không chạy được' in msg


def test_format_quotes_values():
    holder = {'names': set()}
    pm = make(holder, cmd=f'{sys.executable} -c "import sys; print(sys.argv[1], flush=True)" {{map_yaml}}')
    assert pm.start('demo', map_yaml='/tmp/a b; echo hacked/map.yaml')[0]
    assert wait_for(lambda: any('a b; echo hacked' in l for l in pm.tail('demo')))
    pm.shutdown_all()


def test_missing_placeholder():
    pm = make({'names': set()}, cmd='echo {map_yaml}')
    ok, msg = pm.start('demo')
    assert not ok and 'sai cú pháp' in msg


def test_reap_group_kills_lingering_children():
    import os
    import subprocess
    # tiến trình đầu thoát ngay, nhưng để lại 1 con bỏ qua SIGINT/SIGTERM trong cùng nhóm
    leader = subprocess.Popen(['sh', '-c', '(trap "" INT TERM; sleep 30) & exit 0'], start_new_session=True)
    leader.wait()
    os.killpg(leader.pid, 0)          # nhóm vẫn còn (con còn sống)
    ProcessManager._reap_group(leader.pid, grace=0.5)

    def gone():
        try:
            os.killpg(leader.pid, 0)
            return False
        except ProcessLookupError:
            return True
    assert wait_for(gone, 5.0)
