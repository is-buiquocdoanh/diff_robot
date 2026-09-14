# Diff Drive PID Tuner

Bộ công cụ để tune PID tốc độ bánh xe (diff drive robot, ESP32 + encoder +
BTS7960) một cách trực quan: chỉnh Kp/Ki/Kd và xem ngay đồ thị đáp ứng RPM,
trước khi mang bộ số đó sang code chính thức của robot
([ESP32/diff_drive_ros/src/main.cpp](../ESP32/diff_drive_ros/src/main.cpp)).

Gồm 2 phần:

- **Firmware** ([src/main.cpp](src/main.cpp)) — nạp vào ESP32, chạy vòng PID
  tốc độ 50Hz cho 2 bánh, nhận lệnh và gửi dữ liệu qua USB Serial dạng text.
- **Web app** ([tools/web/pid_tuner.html](tools/web/pid_tuner.html)) — mở
  bằng Chrome/Edge trên máy tính, kết nối trực tiếp ESP32 qua Web Serial API,
  có UI chỉnh gains + vẽ đồ thị realtime.

## Yêu cầu

- PlatformIO (CLI hoặc extension VSCode) để build/nạp firmware.
- ESP32 đã đấu: 2 encoder (bánh trái ở GPIO 4/15, bánh phải ở GPIO 23/22),
  2 driver BTS7960 (trái: GPIO 25/26, phải: GPIO 32/33) — đúng sơ đồ chân của
  robot thật, để bộ số tune ra dùng lại được ngay.
- Trình duyệt **Chrome hoặc Edge** (Web Serial API không chạy trên
  Firefox/Safari).

## Bước 1 — Nạp firmware

```bash
cd diff_tune_PID
pio run -t upload
```

Nếu có nhiều cổng USB, thêm `--upload-port /dev/ttyUSBx`. Sau khi nạp xong,
**đóng mọi cửa sổ Serial Monitor** (PlatformIO monitor, Arduino IDE...) —
web app cần tự mở cổng, nếu cổng đang bị chương trình khác giữ thì trình
duyệt sẽ không kết nối được.

## Bước 2 — Mở công cụ web

Mở trực tiếp file bằng Chrome/Edge:

```
chrome tools/web/pid_tuner.html
```

Nếu trình duyệt chặn Web Serial trên `file://` (một số bản Chrome khó chịu
với origin file), chạy tạm 1 server tĩnh rồi mở qua `http://localhost`:

```bash
cd diff_tune_PID/tools/web
python3 -m http.server 8000
# mở http://localhost:8000/pid_tuner.html
```

## Bước 3 — Kết nối và chạy thử

1. Bấm **Kết nối**, chọn đúng cổng ESP32 trong popup của Chrome.
2. Chấm tròn ở góc trên chuyển sang xanh + chữ "Đã kết nối" là ổn.
3. Bấm thử nút **Tiến** với tốc độ mặc định 150 RPM — nếu đấu đúng, 2 bánh
   quay cùng chiều, đồ thị RPM đo được ở dưới sẽ nhảy lên.
4. Bấm **STOP** để dừng ngay khi cần (cũng tự dừng nếu mất kết nối > 0.5s).

## Bước 4 — Quy trình tune PID

Đi theo thứ tự, đừng chỉnh cả 3 số cùng lúc:

1. **Chỉ Kp** (Ki = Kd = 0): tăng dần Kp từ 0, mỗi lần tăng thì bấm **Tiến**
   với 1 tốc độ cố định (ví dụ 150 RPM) để xem đáp ứng. Tăng tới khi bánh
   bám tốc độ nhanh nhưng chưa dao động/rung. Nếu thấy RPM đo được lượn
   sóng quanh setpoint (overshoot cao, dao động kéo dài) thì Kp đã quá cao —
   giảm lại.
2. **Thêm Ki nhỏ**: nếu ở bước 1 bánh bám gần setpoint nhưng luôn lệch một
   chút và không bao giờ về đúng (steady-state error), tăng Ki từ 0 lên từng
   bước nhỏ (0.05–0.1) tới khi sai số ổn định biến mất. Ki quá cao sẽ gây
   giật/rung trễ (overshoot xuất hiện muộn sau khi đã tưởng ổn).
3. **Kd chỉ khi cần**: thường để 0 vì RPM đo từ encoder có nhiễu rời rạc,
   Kd sẽ khuếch đại nhiễu đó. Chỉ thêm (rất nhỏ, ví dụ 0.001–0.01) nếu vẫn
   còn dao động sau khi đã cân Kp/Ki hợp lý và muốn giảm damping thêm.
4. Lặp lại test với **Xoay CW/CCW** (2 bánh ngược dấu) để chắc bộ số cũng ổn
   khi 2 bánh quay ngược chiều nhau, không chỉ khi chạy thẳng.
5. Dùng **square wave** (mục "Test tùy chỉnh") để tự động lặp lại step test
   liên tục — tiện để quan sát nhiều chu kỳ đáp ứng mà không cần bấm tay,
   đặc biệt hữu ích khi so sánh trước/sau mỗi lần đổi gains.

### Các nút test

| Nút | Ý nghĩa | Bánh trái | Bánh phải |
|---|---|---|---|
| Tiến | Chạy thẳng | +v | +v |
| Lùi | Lùi thẳng | -v | -v |
| Xoay CW | Xoay tại chỗ | +v | -v |
| Xoay CCW | Xoay tại chỗ (ngược) | -v | +v |
| Tùy chỉnh | Test 1 bánh hoặc bất đối xứng | tự nhập | tự nhập |
| STOP | Dừng khẩn cấp | 0 | 0 |

`v` lấy từ ô "Tốc độ (RPM)" ở khối Tiến/Lùi/Xoay. Khối "Tùy chỉnh" có 2 ô
riêng — dùng khi muốn test chỉ 1 bánh (ô còn lại để 0) hoặc test lệch tốc độ.

**Square wave**: khi bật, cứ mỗi "Chu kỳ mỗi mức" (ms) hệ thống tự đảo giữa
setpoint hiện tại (giá trị bạn vừa áp dụng lần cuối, qua bất kỳ nút nào) và
0. Tắt checkbox hoặc bấm STOP để dừng.

### Đọc đồ thị và chỉ số

- Đồ thị RPM: nét liền = RPM đo được, nét đứt = setpoint. Xanh = bánh trái,
  cam = bánh phải. Chọn cửa sổ thời gian hiển thị (5/10/20/30s) ở góc trên.
- Đồ thị PWM: giá trị PWM có dấu (-255..255) đang áp ra mỗi bánh — hữu ích
  để thấy PID có bão hòa (kịch ±255) hay không.
- Bảng chỉ số (tính từ lần đổi setpoint gần nhất mỗi bánh):
  - **Overshoot** — vượt quá setpoint bao nhiêu % so với biên độ bước nhảy.
  - **Rise time (10→90%)** — thời gian đi từ 10% đến 90% quãng đường tới
    setpoint.
  - **Settle time (±5%)** — thời gian từ lúc đổi setpoint tới khi RPM đo
    được vào và ở trong vùng ±5% setpoint liên tục ≥300ms.

  Đây là chỉ số ước lượng để so sánh tương đối giữa các lần chỉnh gains,
  không phải số đo phòng lab chính xác tuyệt đối.

## Bước 5 — Mang bộ số về code thật

Khung "Xuất bộ số về main.cpp thật" luôn hiển thị Kp/Ki/Kd hiện tại dạng:

```cpp
const float PID_KP = 1.2000f;
const float PID_KI = 0.0500f;
const float PID_KD = 0.0000f;
```

Bấm **Copy**, dán vào đúng vị trí khai báo `PID_KP/PID_KI/PID_KD` trong
[ESP32/diff_drive_ros/src/main.cpp](../ESP32/diff_drive_ros/src/main.cpp).

**Lưu ý quan trọng về kiến trúc**: PID trong project tune này là
*full-authority* — PID tính thẳng ra PWM (-255..255), không cộng thêm
feedforward tuyến tính. Code sản phẩm hiện tại (`main.cpp` gốc) lại dùng
PID chỉ để "trim" (bù) một lượng nhỏ quanh feedforward tuyến tính, giới hạn
trong `±PID_TRIM_LIMIT` (mặc định 80). Hai kiến trúc cần Kp/Ki/Kd khác nhau
về độ lớn. Có 2 cách áp dụng bộ số vừa tune:

- **Cách đơn giản (khuyến nghị)**: trong `main.cpp` gốc, bật
  `ENABLE_PID = true`, sửa `PID_TRIM_LIMIT` thành `255.0f`, và sửa
  `feedforwardLeft`/`feedforwardRight` thành `0` (bỏ hẳn feedforward) — để
  PID hoạt động full-authority giống hệt project tune này. Dán thẳng
  Kp/Ki/Kd đã tune, không cần quy đổi.
- **Cách giữ nguyên kiến trúc trim**: nếu muốn giữ feedforward + trim như
  cũ, phải tune lại một bộ Kp/Ki/Kd *nhỏ hơn* riêng cho vai trò trim (không
  dùng thẳng số từ project này), vì trim chỉ cần sửa sai số dư sau khi
  feedforward đã làm phần lớn việc.

## Giao thức Serial (cho ai muốn debug/mở rộng)

Baudrate 115200, mỗi lệnh/dữ liệu 1 dòng kết thúc bằng `\n`.

**Host → ESP32:**

| Lệnh | Ý nghĩa |
|---|---|
| `K,<kp>,<ki>,<kd>` | Set gains PID cho cả 2 bánh |
| `V,<left_rpm>,<right_rpm>` | Set setpoint RPM có dấu (âm = lùi) |
| `S` | Dừng ngay + reset PID |

**ESP32 → Host** (gửi liên tục ~50Hz):

```
D,<t_ms>,<sp_left>,<meas_left>,<pwm_left>,<sp_right>,<meas_right>,<pwm_right>
```

`t_ms` là `millis()` trên ESP32 (dùng làm mốc thời gian cho đồ thị).
`pwm_*` là PWM có dấu đã áp ra (dương = tiến, âm = lùi).

An toàn: nếu ESP32 không nhận được lệnh nào (`K`/`V`/`S`) trong >500ms, tự
động dừng cả 2 bánh và reset PID (đề phòng mất kết nối USB giữa lúc xe đang
chạy).

## Xử lý sự cố

- **Không thấy cổng trong popup Kết nối**: kiểm tra đã cắm USB, cài driver
  CH340/CP210x (tùy board), và không có chương trình khác (Serial Monitor,
  Arduino IDE) đang mở cổng đó.
- **Kết nối được nhưng đồ thị không nhảy**: mở panel "Log dữ liệu thô" ở
  cuối trang, xem có dòng `D,...` chảy vào không. Nếu không có gì, kiểm tra
  lại đã nạp đúng firmware của project này (không phải firmware sản phẩm
  cũ) và baudrate 115200.
- **Web Serial báo lỗi "Failed to open serial port"**: cổng đang bị chiếm —
  đóng PlatformIO Serial Monitor / các tab Chrome khác đã kết nối cổng đó,
  rồi bấm Kết nối lại.
- **Bánh rung/giật mạnh khi tăng Ki**: hạ Ki về 0, tăng lại từ từ hơn — Ki
  cộng dồn theo `dt` nên rất nhạy với period 20ms, chỉ cần tăng bằng bước
  0.05 mỗi lần.
