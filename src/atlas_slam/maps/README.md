# Maps

Thư mục này chứa bản đồ do SLAM tạo ra (`slam.launch.py`), định dạng chuẩn Nav2
(`.pgm` + `.yaml`) — dùng cho AMCL, vì `navigation.launch.py` luôn định vị bằng
AMCL (xem `launch/localization.launch.py`).

```bash
ros2 run nav2_map_server map_saver_cli -f src/atlas_slam/maps/<tên_map>
```

Tạo ra `<tên_map>.yaml` (metadata) + `<tên_map>.pgm` (ảnh occupancy grid). Chạy
lệnh này trong lúc `slam.launch.py` đang hoạt động, sau khi đã lái robot đi hết
khu vực cần vẽ map. Dùng file `.yaml` tạo ra làm giá trị cho arg `map:=` khi
chạy `navigation.launch.py`.

Các file `maze_map.*` trong thư mục này là map mẫu tạo trong môi trường mô
phỏng (world "maze") của phiên bản trước — không đại diện cho không gian hoạt
động thật của robot A3, cần vẽ map mới bằng robot thật trước khi dùng
`navigation.launch.py` thật sự.
