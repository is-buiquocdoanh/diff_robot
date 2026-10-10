#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phát âm thanh cảnh báo khi Collision Monitor đang CHẶN /cmd_vel vì có vật cản, tắt
cảnh báo khi vật cản hết và robot đi tiếp. Xem docs/canh_bao_am_thanh.md để biết cách dùng.

Bản Nav2 Humble (nav2_collision_monitor 1.1.20) KHÔNG publish topic trạng thái
(tham số "state_topic" không được node nhận -- đã kiểm tra bằng
`ros2 param list /collision_monitor`, không có field này), nên node KHÔNG thể subscribe
thẳng "đang STOP hay không" như các bản Nav2 mới hơn (Iron+) có nav2_msgs/CollisionMonitorState.

Thay vào đó suy luận gián tiếp bằng cách so 2 topic đã có sẵn (xem navigation.launch.py):
  cmd_vel_in_topic  (mặc định /cmd_vel_raw) -- lệnh velocity_smoother MUỐN gửi
  cmd_vel_out_topic (mặc định /cmd_vel)     -- lệnh THẬT SỰ xuống robot sau khi
                                               collision_monitor lọc theo va chạm
Nếu robot đang MUỐN đi (cmd_vel_raw != 0) nhưng lệnh thật lại bằng 0 (cmd_vel == 0) kéo
dài hơn enter_delay giây -> chắc chắn collision_monitor đang STOP vì vật cản (không phải
robot tự nhiên đứng yên, vì khi đó cmd_vel_raw cũng đã bằng 0 rồi) -> bật cảnh báo. Hết
chênh lệch (cmd_vel lại bằng cmd_vel_raw, hoặc cmd_vel_raw về 0 vì huỷ goal) -> tắt.
"""
import os
import shutil
import struct
import subprocess
import threading
import wave

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

_SAMPLE_RATE = 44100


def _generate_beep_wav(path, freq_hz, duration_s):
    """Sinh 1 tiếng beep hình sin ngắn bằng stdlib (wave+struct), không cần numpy/asset
    ngoài -- giống cách maps.py tự vẽ PNG bằng zlib thay vì phụ thuộc Pillow."""
    import math

    n_samples = int(_SAMPLE_RATE * duration_s)
    amplitude = 24000  # < 32767, chừa biên độ tránh clip loa rẻ tiền
    fade = max(1, n_samples // 20)  # fade-in/out ngắn để không "tạch" ở đầu/cuối
    frames = bytearray()
    for i in range(n_samples):
        env = 1.0
        if i < fade:
            env = i / fade
        elif i > n_samples - fade:
            env = (n_samples - i) / fade
        sample = int(amplitude * env * math.sin(2 * math.pi * freq_hz * i / _SAMPLE_RATE))
        frames += struct.pack('<h', sample)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with wave.open(path, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(_SAMPLE_RATE)
        wf.writeframes(bytes(frames))


class CollisionAlarmNode(Node):
    def __init__(self):
        super().__init__('collision_alarm')

        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel_raw')
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel')
        self.declare_parameter('speed_eps', 0.02)          # m/s hoặc rad/s -- dưới ngưỡng này coi là "đứng yên"
        self.declare_parameter('msg_timeout', 0.5)         # giây -- lệnh quá cũ coi như đã về 0 (mất kết nối/node đứng)
        self.declare_parameter('enter_delay', 0.15)        # giây lệch liên tục mới bật cảnh báo (chống báo nhầm thoáng qua)
        self.declare_parameter('exit_delay', 0.15)         # giây hết lệch liên tục mới tắt cảnh báo
        self.declare_parameter('beep_interval', 0.6)       # giây giữa 2 tiếng beep khi đang cảnh báo
        self.declare_parameter('beep_freq_hz', 880.0)
        self.declare_parameter('beep_duration_s', 0.15)
        self.declare_parameter('player', 'aplay')          # 'aplay' (ALSA) hoặc 'paplay' (PulseAudio)
        self.declare_parameter('wav_path', '')              # rỗng = tự sinh beep vào ~/.cache/atlas_slam/

        self.speed_eps = float(self.get_parameter('speed_eps').value)
        self.msg_timeout = float(self.get_parameter('msg_timeout').value)
        self.enter_delay = float(self.get_parameter('enter_delay').value)
        self.exit_delay = float(self.get_parameter('exit_delay').value)
        self.beep_interval = float(self.get_parameter('beep_interval').value)
        self.player = str(self.get_parameter('player').value)

        wav_path = str(self.get_parameter('wav_path').value)
        if not wav_path:
            wav_path = os.path.expanduser('~/.cache/atlas_slam/collision_beep.wav')
        if not os.path.isfile(wav_path):
            _generate_beep_wav(wav_path, float(self.get_parameter('beep_freq_hz').value),
                                float(self.get_parameter('beep_duration_s').value))
        self.wav_path = wav_path

        self.player_ok = shutil.which(self.player) is not None
        if not self.player_ok:
            self.get_logger().error(
                f'Không tìm thấy lệnh "{self.player}" để phát âm thanh -- cài alsa-utils '
                f'("sudo apt install alsa-utils") hoặc đổi tham số player. Node vẫn chạy '
                f'(vẫn log cảnh báo) nhưng sẽ không phát được tiếng.')

        self._raw = None          # (Twist, stamp) mới nhất từ cmd_vel_in_topic
        self._out = None          # (Twist, stamp) mới nhất từ cmd_vel_out_topic
        self._blocked_since = None
        self._clear_since = None
        self._alarm_active = False
        self._last_beep = 0.0
        self._beep_busy = threading.Lock()

        self.create_subscription(Twist, self.get_parameter('cmd_vel_in_topic').value,
                                  lambda m: self._on('_raw', m), 10)
        self.create_subscription(Twist, self.get_parameter('cmd_vel_out_topic').value,
                                  lambda m: self._on('_out', m), 10)
        self.create_timer(1.0 / 20.0, self._tick)
        self.get_logger().info(
            f'collision_alarm sẵn sàng (so {self.get_parameter("cmd_vel_in_topic").value} '
            f'với {self.get_parameter("cmd_vel_out_topic").value}, beep="{self.wav_path}")')

    def _on(self, attr, msg):
        setattr(self, attr, (msg, self._now()))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _speed(twist):
        return max(abs(twist.linear.x), abs(twist.angular.z))

    def _fresh_speed(self, cached, now):
        if cached is None:
            return 0.0
        msg, stamp = cached
        if now - stamp > self.msg_timeout:
            return 0.0
        return self._speed(msg)

    def _tick(self):
        now = self._now()
        raw_speed = self._fresh_speed(self._raw, now)
        out_speed = self._fresh_speed(self._out, now)
        requesting = raw_speed > self.speed_eps
        blocked_now = requesting and out_speed <= self.speed_eps

        if blocked_now:
            self._blocked_since = self._blocked_since or now
            self._clear_since = None
        else:
            self._clear_since = self._clear_since or now
            self._blocked_since = None

        if not self._alarm_active and self._blocked_since is not None \
                and now - self._blocked_since >= self.enter_delay:
            self._alarm_active = True
            self._last_beep = 0.0
            self.get_logger().warn('Vật cản chặn đường -- phát cảnh báo âm thanh')
        elif self._alarm_active and self._clear_since is not None \
                and now - self._clear_since >= self.exit_delay:
            self._alarm_active = False
            self.get_logger().info('Hết vật cản -- robot tiếp tục di chuyển')

        if self._alarm_active and self.player_ok and now - self._last_beep >= self.beep_interval:
            self._last_beep = now
            threading.Thread(target=self._beep, daemon=True).start()

    def _beep(self):
        if not self._beep_busy.acquire(blocking=False):
            return   # tiếng trước chưa phát xong, bỏ qua lần này thay vì xếp hàng chồng tiếng
        try:
            subprocess.run([self.player, self.wav_path], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, timeout=3.0)
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().error(f'Không phát được âm thanh: {e}')
        finally:
            self._beep_busy.release()


def main():
    rclpy.init()
    node = CollisionAlarmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
