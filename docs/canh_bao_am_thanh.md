# Cảnh báo âm thanh khi gặp vật cản (Collision Monitor)

Khi robot đang di chuyển mà gặp vật cản lọt vào vùng an toàn của **Collision Monitor**
(xem `src/atlas_slam/config/collision_monitor_params.yaml`), node `collision_alarm`
tự phát tiếng beep lặp lại cho tới khi vật cản biến mất, lúc đó robot tự đi tiếp và
tiếng beep tắt. Không cần làm gì thêm — mặc định đã bật sẵn khi chạy
`navigation.launch.py`.

## Vì sao không dùng thẳng trạng thái của Collision Monitor

Bản Nav2 của Humble đang dùng (`nav2_collision_monitor` 1.1.20) **không publish**
topic trạng thái (`nav2_msgs/msg/CollisionMonitorState`) dù file config có khai
`state_topic` — tham số đó bị node bỏ qua (kiểm tra bằng
`ros2 param list /collision_monitor` sẽ không thấy `state_topic` trong danh sách).
Tính năng này chỉ có ở bản Nav2 mới hơn (Iron trở lên).

Vì vậy `collision_alarm_node.py` suy luận gián tiếp bằng cách so sánh 2 topic đã có
sẵn trong `navigation.launch.py`:

| Topic | Ý nghĩa |
|---|---|
| `/cmd_vel_raw` | lệnh `velocity_smoother` **muốn** gửi (output của Nav2 controller) |
| `/cmd_vel` | lệnh **thật sự** xuống robot, sau khi `collision_monitor` lọc theo `/scan` |

Nếu robot đang muốn đi (`cmd_vel_raw` khác 0) nhưng lệnh thật lại bằng 0
(`cmd_vel` = 0) liên tục hơn 0.15 giây → chắc chắn `collision_monitor` đang chặn vì
vật cản (không phải robot tự nhiên đứng yên, vì khi đó `cmd_vel_raw` cũng đã về 0).
Hết chênh lệch → tắt cảnh báo.

```
controller_server ─▶ cmd_vel_nav ─▶ velocity_smoother ─▶ cmd_vel_raw ─┬─▶ collision_monitor ─▶ cmd_vel ─▶ robot
                                                                       │        ▲
                                                           collision_alarm ─────┘
                                                        (so cmd_vel_raw với cmd_vel)
```

## Tham số (`collision_alarm_node.py`)

Chỉnh bằng `ros2 param set` lúc đang chạy, hoặc truyền qua launch:

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `speed_eps` | `0.02` | m/s hoặc rad/s — dưới ngưỡng này coi là "đứng yên" |
| `enter_delay` | `0.15` s | phải lệch liên tục bấy nhiêu giây mới bật cảnh báo (chống báo nhầm thoáng qua) |
| `exit_delay` | `0.15` s | hết lệch liên tục bấy nhiêu giây mới tắt cảnh báo |
| `beep_interval` | `0.6` s | khoảng cách giữa 2 tiếng beep khi đang cảnh báo |
| `beep_freq_hz` / `beep_duration_s` | `880.0` / `0.15` | tần số/độ dài tiếng beep tự sinh |
| `player` | `aplay` | lệnh phát âm thanh — đổi thành `paplay` nếu dùng PulseAudio |
| `wav_path` | *(rỗng)* | rỗng = tự sinh beep vào `~/.cache/atlas_slam/collision_beep.wav`; điền đường dẫn `.wav` khác để dùng âm thanh riêng |

Tiếng beep mặc định tự sinh bằng `wave`/`struct` của Python (không cần file âm thanh
đính kèm), lưu 1 lần ở `~/.cache/atlas_slam/`, dùng lại các lần sau. Xoá file đó nếu
muốn sinh lại (vd. sau khi đổi `beep_freq_hz`).

## Bật/tắt

```bash
# Tắt hẳn cảnh báo âm thanh (Collision Monitor vẫn hoạt động bình thường)
ros2 launch atlas_slam navigation.launch.py map:=... collision_alarm:=false

# Đổi sang PulseAudio + tiếng beep riêng
ros2 launch atlas_slam navigation.launch.py map:=... \
    collision_alarm:=true
ros2 param set /collision_alarm player paplay
ros2 param set /collision_alarm wav_path /home/you/canh_bao.wav
```

Lưu ý: tắt `collision_monitor:=false` thì `cmd_vel_raw` được relay thẳng sang
`cmd_vel` (xem `collision_monitor_bypass` trong `navigation.launch.py`), nên
`collision_alarm` sẽ không bao giờ thấy chênh lệch để báo — không cần tắt tay thêm
`collision_alarm` trong trường hợp đó.

## Kiểm tra nhanh không cần robot thật

```bash
ros2 launch a3_web demo.launch.py          # robot giả (xem src/a3_web/README.md)
ros2 launch atlas_slam navigation.launch.py map:=<map.yaml> rviz:=false
```

Theo dõi log:

```bash
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel
```

Khi robot đang điều hướng mà có vật thật/giả lọt vào vùng `PolygonStop`
(~8cm quanh thân robot, xem `collision_monitor_params.yaml`), `/cmd_vel` sẽ về 0
trong khi `/cmd_vel_raw` vẫn khác 0, terminal chạy `navigation.launch.py` in:

```
[collision_alarm]: Vật cản chặn đường -- phát cảnh báo âm thanh
...
[collision_alarm]: Hết vật cản -- robot tiếp tục di chuyển
```

## Không nghe thấy tiếng

1. Kiểm tra log node có dòng `Không tìm thấy lệnh "aplay"...` không — cài
   `sudo apt install alsa-utils` (hoặc đổi `player` sang `paplay` nếu dùng
   PulseAudio, thường có sẵn trên Ubuntu).
2. Test phát tay: `aplay ~/.cache/atlas_slam/collision_beep.wav` — nếu không ra
   tiếng thì lỗi nằm ở driver âm thanh/loa của máy, không phải node.
3. `aplay -l` liệt kê thiết bị phát — nếu máy có nhiều card âm thanh (vd. HDMI +
   loa trong), có thể cần chọn đúng card mặc định (`alsamixer`, hoặc
   `~/.asoundrc`) trước khi node mới phát đúng ra loa mong muốn.
