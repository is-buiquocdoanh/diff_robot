#ifndef WHEEL_PID_H
#define WHEEL_PID_H

// Bộ điều khiển PID tốc độ cho 1 bánh (đơn vị: RPM có dấu -> PWM có dấu).
// Tách riêng khỏi main.cpp để main.cpp chỉ còn phần khai báo hằng số/tham số
// (Kp, Ki, Kd, RPM_MAX, PID_INTERVAL_MS...) - dễ chỉnh mà không phải đọc lại
// thuật toán mỗi lần tune.
class WheelPID {
public:
  WheelPID(float kp, float ki, float kd, float output_min, float output_max);

  void setGains(float kp, float ki, float kd);

  // Xóa trạng thái tích phân/đạo hàm - gọi khi dừng xe hoặc chuyển hướng đột
  // ngột, để integral không "nhớ" sai số cũ gây giật khi chạy lại.
  void reset();

  // setpoint_rpm, measured_rpm: RPM có dấu (âm = lùi). dt: giây kể từ lần
  // compute() trước. Trả về giá trị output đã clamp trong [output_min, output_max].
  float compute(float setpoint_rpm, float measured_rpm, float dt);

private:
  float kp_;
  float ki_;
  float kd_;
  float output_min_;
  float output_max_;

  float integral_;
  float prev_error_;
  bool has_prev_;
};

#endif
