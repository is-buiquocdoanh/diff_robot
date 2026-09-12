#include "wheel_pid.h"

WheelPID::WheelPID(float kp, float ki, float kd, float output_min, float output_max)
  : kp_(kp), ki_(ki), kd_(kd),
    output_min_(output_min), output_max_(output_max),
    integral_(0.0f), prev_error_(0.0f), has_prev_(false) {}

void WheelPID::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void WheelPID::reset() {
  integral_ = 0.0f;
  prev_error_ = 0.0f;
  has_prev_ = false;
}

float WheelPID::compute(float setpoint_rpm, float measured_rpm, float dt) {
  if (dt <= 0.0f) {
    return 0.0f;
  }

  float error = setpoint_rpm - measured_rpm;

  float derivative = 0.0f;
  if (has_prev_) {
    derivative = (error - prev_error_) / dt;
  }
  prev_error_ = error;
  has_prev_ = true;

  float output_unclamped = kp_ * error + ki_ * integral_ + kd_ * derivative;

  float output = output_unclamped;
  if (output > output_max_) output = output_max_;
  if (output < output_min_) output = output_min_;

  // Anti-windup kiểu clamping: chỉ cộng dồn tích phân khi output chưa bão
  // hòa theo hướng làm error lớn thêm - tránh integral "phình" khi PWM đã
  // kịch trần/sàn mà sai số vẫn còn (vd khi bánh bị chặn cứng).
  bool saturated_high = (output_unclamped > output_max_) && (error > 0.0f);
  bool saturated_low  = (output_unclamped < output_min_) && (error < 0.0f);
  if (!saturated_high && !saturated_low) {
    integral_ += error * dt;
  }

  return output;
}
