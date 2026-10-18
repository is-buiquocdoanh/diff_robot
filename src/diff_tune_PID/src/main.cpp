#include <Arduino.h>
#include "driver/pcnt.h"
#include "wheel_pid.h"

// ==================== Pin / phần cứng ====================
// Giữ đúng sơ đồ chân như robot thật (src/diff_drive_ros) để bộ số PID tìm
// được ở đây transfer thẳng sang, không lệch do khác động cơ/encoder/gearbox.
#define ENC_LEFT_FRONT_A 15
#define ENC_LEFT_FRONT_B 4
#define ENC_RIGHT_FRONT_A 23
#define ENC_RIGHT_FRONT_B 22

const int L_PWM_PIN_A = 25; // left forward
const int L_PWM_PIN_B = 26; // left backward
const int R_PWM_PIN_A = 32; // right forward
const int R_PWM_PIN_B = 33; // right backward

const int CH_L_A = 0;
const int CH_L_B = 1;
const int CH_R_A = 2;
const int CH_R_B = 3;

const int PWM_FREQ = 2000;
const int PWM_RES = 8; // 8-bit

// Encoder / gearbox - copy nguyên từ main.cpp sản phẩm (đã xác nhận đúng).
const float ENCODER_PULSES_PER_REV = 13.0f;
const float EDGE_COUNT = 2.0f;
const float GEARBOX_RATIO = 19.2f;
const float TICKS_PER_REV = ENCODER_PULSES_PER_REV * EDGE_COUNT * GEARBOX_RATIO; // 499.2

// Giới hạn an toàn cho setpoint nhận từ web tool (RPM có dấu).
const float RPM_SETPOINT_LIMIT = 350.0f;

// ==================== PID (full-authority) ====================
// Khác với main.cpp sản phẩm (PID chỉ "trim" quanh feedforward tuyến tính),
// ở đây PID điều khiển toàn quyền PWM từ sai số RPM - để đồ thị đáp ứng phản
// ánh đúng hành vi thật của riêng bộ PID, không bị feedforward che khuất.
// Kp/Ki/Kd tìm được ở đây dùng làm PID chính (thay hẳn feedforward) khi đưa
// về robot thật, hoặc làm điểm khởi đầu để tính lại hệ số trim nếu vẫn muốn
// giữ kiến trúc feedforward + trim.
WheelPID leftPid(0.0f, 0.0f, 0.0f, -255.0f, 255.0f);
WheelPID rightPid(0.0f, 0.0f, 0.0f, -255.0f, 255.0f);

volatile int16_t tickLeft = 0;
volatile int16_t tickRight = 0;
int16_t pidPrevTickLeft = 0;
int16_t pidPrevTickRight = 0;

float setpointLeft = 0.0f;  // RPM có dấu, âm = lùi
float setpointRight = 0.0f;

const unsigned long PID_INTERVAL_MS = 20; // 50Hz
unsigned long lastPidMillis = 0;

// An toàn: mất kết nối với web tool (không có lệnh mới) -> dừng xe.
const unsigned long CMD_TIMEOUT_MS = 500;
unsigned long lastCmdMillis = 0;

void setupPCNT(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t pcntConfig;
  pcntConfig.pulse_gpio_num = pinA;
  pcntConfig.ctrl_gpio_num = pinB;
  pcntConfig.channel = PCNT_CHANNEL_0;
  pcntConfig.unit = unit;
  pcntConfig.pos_mode = PCNT_COUNT_INC;
  pcntConfig.neg_mode = PCNT_COUNT_DEC;
  pcntConfig.lctrl_mode = PCNT_MODE_REVERSE;
  pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
  pcntConfig.counter_h_lim = 32767;
  pcntConfig.counter_l_lim = -32768;

  pcnt_unit_config(&pcntConfig);
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void set_bts7960_pwm(int channelA, int channelB, int pwm, int dir) {
  // dir: 0 stop, 1 forward (A on), 2 backward (B on)
  if (dir == 1) {
    ledcWrite(channelA, pwm);
    ledcWrite(channelB, 0);
  } else if (dir == 2) {
    ledcWrite(channelA, 0);
    ledcWrite(channelB, pwm);
  } else {
    ledcWrite(channelA, 0);
    ledcWrite(channelB, 0);
  }
}

void signedToDirPwm(float value, int &dir, int &pwm) {
  int rounded = (int)roundf(value);
  if (rounded > 0) {
    dir = 1;
    pwm = constrain(rounded, 0, 255);
  } else if (rounded < 0) {
    dir = 2;
    pwm = constrain(-rounded, 0, 255);
  } else {
    dir = 0;
    pwm = 0;
  }
}

void applyStop() {
  setpointLeft = 0.0f;
  setpointRight = 0.0f;
  leftPid.reset();
  rightPid.reset();
  set_bts7960_pwm(CH_L_A, CH_L_B, 0, 0);
  set_bts7960_pwm(CH_R_A, CH_R_B, 0, 0);
}

// Giao thức text đơn giản qua Serial (USB), 1 lệnh / dòng, kết thúc bằng \n:
//   K,<kp>,<ki>,<kd>   - set gains cho cả 2 bánh
//   V,<left>,<right>   - set setpoint RPM có dấu cho bánh trái/phải
//   S                  - dừng ngay + reset PID
void handleLine(char *line) {
  lastCmdMillis = millis();
  if (line[0] == 'K') {
    float kp, ki, kd;
    if (sscanf(line, "K,%f,%f,%f", &kp, &ki, &kd) == 3) {
      leftPid.setGains(kp, ki, kd);
      rightPid.setGains(kp, ki, kd);
      Serial.printf("OK,K,%.4f,%.4f,%.4f\n", kp, ki, kd);
    }
  } else if (line[0] == 'V') {
    float l, r;
    if (sscanf(line, "V,%f,%f", &l, &r) == 2) {
      setpointLeft = constrain(l, -RPM_SETPOINT_LIMIT, RPM_SETPOINT_LIMIT);
      setpointRight = constrain(r, -RPM_SETPOINT_LIMIT, RPM_SETPOINT_LIMIT);
    }
  } else if (line[0] == 'S') {
    applyStop();
  }
}

void readSerialCommands() {
  static char buf[64];
  static uint8_t idx = 0;
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    char ch = (char)c;
    if (ch == '\n' || ch == '\r') {
      if (idx > 0) {
        buf[idx] = '\0';
        handleLine(buf);
        idx = 0;
      }
    } else if (idx < (uint8_t)(sizeof(buf) - 1)) {
      buf[idx++] = ch;
    } else {
      idx = 0; // overflow -> reset
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(ENC_LEFT_FRONT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_FRONT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_FRONT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_FRONT_B, INPUT_PULLUP);

  setupPCNT(PCNT_UNIT_0, ENC_LEFT_FRONT_A, ENC_LEFT_FRONT_B);
  setupPCNT(PCNT_UNIT_1, ENC_RIGHT_FRONT_A, ENC_RIGHT_FRONT_B);

  ledcSetup(CH_L_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_PWM_PIN_A, CH_L_A);
  ledcWrite(CH_L_A, 0);

  ledcSetup(CH_L_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_PWM_PIN_B, CH_L_B);
  ledcWrite(CH_L_B, 0);

  ledcSetup(CH_R_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_PWM_PIN_A, CH_R_A);
  ledcWrite(CH_R_A, 0);

  ledcSetup(CH_R_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_PWM_PIN_B, CH_R_B);
  ledcWrite(CH_R_B, 0);

  lastCmdMillis = millis();
  lastPidMillis = millis();
  Serial.println("READY,diff_tune_PID");
}

void loop() {
  int16_t tmpL = 0, tmpR = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &tmpL);
  pcnt_get_counter_value(PCNT_UNIT_1, &tmpR);
  tickLeft = tmpL;
  tickRight = tmpR;

  readSerialCommands();

  if (millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
    applyStop();
  }

  unsigned long now = millis();
  if (now - lastPidMillis >= PID_INTERVAL_MS) {
    float dt = (now - lastPidMillis) / 1000.0f;
    lastPidMillis = now;

    int deltaL = (int)tickLeft - (int)pidPrevTickLeft;
    int deltaR = (int)tickRight - (int)pidPrevTickRight;
    if (deltaL > 10000) deltaL -= 32768;
    if (deltaL < -10000) deltaL += 32768;
    if (deltaR > 10000) deltaR -= 32768;
    if (deltaR < -10000) deltaR += 32768;
    pidPrevTickLeft = tickLeft;
    pidPrevTickRight = tickRight;

    if (dt > 0.0f) {
      float measuredLeft  = ((float)deltaL / TICKS_PER_REV) * (60.0f / dt);
      float measuredRight = ((float)deltaR / TICKS_PER_REV) * (60.0f / dt);

      float outLeft  = leftPid.compute(setpointLeft,  measuredLeft,  dt);
      float outRight = rightPid.compute(setpointRight, measuredRight, dt);

      int dirL, pwmL, dirR, pwmR;
      signedToDirPwm(outLeft, dirL, pwmL);
      signedToDirPwm(outRight, dirR, pwmR);

      set_bts7960_pwm(CH_L_A, CH_L_B, pwmL, dirL);
      set_bts7960_pwm(CH_R_A, CH_R_B, pwmR, dirR);

      long pwmSignedL = (dirL == 2) ? -(long)pwmL : (long)pwmL;
      long pwmSignedR = (dirR == 2) ? -(long)pwmR : (long)pwmR;

      // D,<t_ms>,<sp_left>,<meas_left>,<pwm_left>,<sp_right>,<meas_right>,<pwm_right>
      Serial.printf("D,%lu,%.2f,%.2f,%ld,%.2f,%.2f,%ld\n",
                    now, setpointLeft, measuredLeft, pwmSignedL,
                    setpointRight, measuredRight, pwmSignedR);
    }
  }
}
