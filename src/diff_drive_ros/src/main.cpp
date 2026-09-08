#include <Arduino.h>
#include "driver/pcnt.h"
#include <ctype.h>
#include "can_serial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Encoder pins (same as your original configuration)
#define ENC_LEFT_FRONT_A 4
#define ENC_LEFT_FRONT_B 15

#define ENC_RIGHT_FRONT_A 23
#define ENC_RIGHT_FRONT_B 22

// BTS7960 control pins (IN1, IN2 inputs -> each motor has two PWM inputs)
const int L_PWM_PIN_A = 25; // left forward
const int L_PWM_PIN_B = 26; // left backward
const int R_PWM_PIN_A = 32; // right forward
const int R_PWM_PIN_B = 33; // right backward

// LEDC channels
const int CH_L_A = 0;
const int CH_L_B = 1;
const int CH_R_A = 2;
const int CH_R_B = 3;

// Encoder counters
volatile int16_t tickLeft = 0;
volatile int16_t tickRight = 0;

// Encoder / gearbox parameters (adjust if different)
const int ENCODER_PULSES_PER_REV = 13;
const int QUAD_FACTOR = 4;
const int GEARBOX_RATIO = 38;
const int TICKS_PER_REV = ENCODER_PULSES_PER_REV * QUAD_FACTOR * GEARBOX_RATIO; // e.g. 1976

// Timing
const unsigned long PRINT_INTERVAL = 500; // ms
unsigned long lastPrint = 0;
// Packet timeout: if no binary packet received within this ms, stop motors
const unsigned long PACKET_TIMEOUT_MS = 1000;
unsigned long lastPacketMillis = 0;

// PWM settings
const int PWM_FREQ = 2000;
const int PWM_RES = 8; // 8-bit

// IMU (BNO055, I2C). GPIO22 is already used by ENC_RIGHT_FRONT_B, so the I2C
// bus is remapped away from the ESP32 default pins (21/22) to avoid conflict.
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 19
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A, &Wire);
bool imuReady = false;
const unsigned long IMU_SEND_INTERVAL = 20; // ms (~50 Hz)
unsigned long lastImuSend = 0;

// Reads the BNO055's onboard sensor fusion output and sends it to the Pi as a
// plain-text CSV line over USB Serial. Motor-command frames arriving on
// Serial are binary and line-terminated debug/IMU text never collides with
// them since the Pi only ever reads text lines back from the ESP32.
void sendImuData() {
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     // rad/s
  imu::Vector<3> lacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // m/s^2, gravity removed

  uint8_t sys, gyroCal, accelCal, magCal;
  bno.getCalibration(&sys, &gyroCal, &accelCal, &magCal);

  Serial.printf("IMU,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u,%u\n",
                quat.w(), quat.x(), quat.y(), quat.z(),
                gyro.x(), gyro.y(), gyro.z(),
                lacc.x(), lacc.y(), lacc.z(),
                sys, gyroCal, accelCal, magCal);
}

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
  pcnt_set_filter_value(unit, 1000);
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

// Use Serial for debug and Serial2 for Raspi cmd_vel (CAN-serial protocol)
// Default Serial2 pins can be set below if needed
#define RXD2 16
#define TXD2 17

CanSerial CSerialUSB(Serial);
CanSerial CSerial2(Serial2);

void setup() {
  Serial.begin(115200);
  delay(50);
  // start Serial (USB) and UART2 for Raspberry Pi / external controller
  CSerialUSB.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  CSerial2.begin(115200);
  delay(50);
  // Enable internal pull-ups for encoder inputs (helps open-collector encoders)
  pinMode(ENC_LEFT_FRONT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_FRONT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_FRONT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_FRONT_B, INPUT_PULLUP);

  // Setup encoders and use a smaller filter threshold so short pulses are not dropped
  setupPCNT(PCNT_UNIT_0, ENC_LEFT_FRONT_A, ENC_LEFT_FRONT_B);
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);

  setupPCNT(PCNT_UNIT_1, ENC_RIGHT_FRONT_A, ENC_RIGHT_FRONT_B);
  pcnt_set_filter_value(PCNT_UNIT_1, 100);
  pcnt_filter_enable(PCNT_UNIT_1);

  // Setup PWM channels
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

  Serial.println("BTS7960 motor test ready.");
  Serial.println("Commands:\n  L <pwm 0-255> <dir 0|1|2>   - set left motor\n  R <pwm> <dir> - set right motor\n  S - stop both");

  // IMU init
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  if (!bno.begin()) {
    Serial.println("[IMU] BNO055 not detected - check wiring/address");
    imuReady = false;
  } else {
    delay(50);
    bno.setExtCrystalUse(true);
    imuReady = true;
    Serial.println("[IMU] BNO055 ready");
  }
}

// Note: ASCII parsing below uses a fixed buffer and C-style parsing to avoid Arduino String.

void loop() {
  // read encoder values into temporaries (pcnt API expects non-volatile pointer)
  int16_t tmpL = 0;
  int16_t tmpR = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &tmpL);
  pcnt_get_counter_value(PCNT_UNIT_1, &tmpR);
  tickLeft = tmpL;
  tickRight = tmpR;

  // Read cmd_vel-like packet from either Serial (USB) or Serial2 (UART2) and apply to motors
  DataPacket pkt;
  bool got = false;
  if (CSerialUSB.readPacket(pkt)) {
    got = true;
    lastPacketMillis = millis();
  } else if (CSerial2.readPacket(pkt)) {
    got = true;
    lastPacketMillis = millis();
  }

  if (got) {
    // Expecting layout similar to original: data[0]=LF_dir, data[1]=LF_pwm, data[2]=RF_dir, data[3]=RF_pwm
    int lf_dir = pkt.data[0];
    int lf_pwm = pkt.data[1];
    int rf_dir = pkt.data[2];
    int rf_pwm = pkt.data[3];
    lf_pwm = constrain(lf_pwm, 0, 255);
    rf_pwm = constrain(rf_pwm, 0, 255);
    set_bts7960_pwm(CH_L_A, CH_L_B, lf_pwm, lf_dir);
    set_bts7960_pwm(CH_R_A, CH_R_B, rf_pwm, rf_dir);
  }

  // Stream IMU data to the Pi over USB Serial at a fixed rate, interleaved
  // as plain text lines with any other debug output on the same port.
  if (imuReady) {
    unsigned long nowImu = millis();
    if (nowImu - lastImuSend >= IMU_SEND_INTERVAL) {
      lastImuSend = nowImu;
      sendImuData();
    }
  }

  // Safety: if no binary packet received for PACKET_TIMEOUT_MS, stop motors
  if (!got) {
    if (millis() - lastPacketMillis > PACKET_TIMEOUT_MS) {
      // only call stop once per timeout occurrence (set motors to zero)
      static bool stopped = false;
      if (!stopped) {
        set_bts7960_pwm(CH_L_A, CH_L_B, 0, 0);
        set_bts7960_pwm(CH_R_A, CH_R_B, 0, 0);
        Serial.println("[SAFETY] No packet timeout - motors stopped");
        stopped = true;
      }
    }
  } else {
    // reset stopped flag when we receive a packet
    static bool stopped = false;
    stopped = false;
  }

  // serial command handling (ASCII) - only if no binary packet consumed
  // Use a small fixed buffer to avoid String allocations and blocking
  static char cmdBuf[64];
  static uint8_t cmdIdx = 0;
  if (!got) {
    while (Serial.available()) {
      int c = Serial.read();
      if (c < 0) break;
      char ch = (char)c;
      // accept CR/LF as terminator
      if (ch == '\n' || ch == '\r') {
        if (cmdIdx == 0) {
          // empty line, ignore
        } else {
          cmdBuf[cmdIdx] = '\0';
          // parse command in-place
          char cmd = toupper(cmdBuf[0]);
          if (cmd == 'L' || cmd == 'R') {
            int idx = 1;
            bool found = false;
            // parse integers from C string
            int pwm = 0;
            int sign = 1;
            // simple parse: skip non-digit and '-'
            while (idx < cmdIdx && cmdBuf[idx] && !(isdigit((unsigned char)cmdBuf[idx]) || cmdBuf[idx] == '-')) idx++;
            if (idx < cmdIdx && (isdigit((unsigned char)cmdBuf[idx]) || cmdBuf[idx] == '-')) {
              if (cmdBuf[idx] == '-') { sign = -1; idx++; }
              int v = 0; bool any = false;
              while (idx < cmdIdx && isdigit((unsigned char)cmdBuf[idx])) { any = true; v = v*10 + (cmdBuf[idx]-'0'); idx++; }
              if (any) pwm = v * sign, found = true;
            }
            if (!found) pwm = 0;
            // parse dir
            int dir = 1;
            while (idx < cmdIdx && cmdBuf[idx] && !(isdigit((unsigned char)cmdBuf[idx]) || cmdBuf[idx] == '-')) idx++;
            if (idx < cmdIdx && (isdigit((unsigned char)cmdBuf[idx]) || cmdBuf[idx] == '-')) {
              int s = 1; if (cmdBuf[idx] == '-') { s = -1; idx++; }
              int v = 0; bool any = false;
              while (idx < cmdIdx && isdigit((unsigned char)cmdBuf[idx])) { any = true; v = v*10 + (cmdBuf[idx]-'0'); idx++; }
              if (any) dir = v * s;
            }
            pwm = constrain(pwm, 0, 255);
            if (cmd == 'L') {
              set_bts7960_pwm(CH_L_A, CH_L_B, pwm, dir);
              Serial.printf("Set LEFT pwm=%d dir=%d\n", pwm, dir);
            } else {
              set_bts7960_pwm(CH_R_A, CH_R_B, pwm, dir);
              Serial.printf("Set RIGHT pwm=%d dir=%d\n", pwm, dir);
            }
          } else if (cmd == 'S') {
            set_bts7960_pwm(CH_L_A, CH_L_B, 0, 0);
            set_bts7960_pwm(CH_R_A, CH_R_B, 0, 0);
            Serial.println("Stopped both motors");
          } else if (cmd == 'D') { // DBG
            int la_state = digitalRead(ENC_LEFT_FRONT_A);
            int lb_state = digitalRead(ENC_LEFT_FRONT_B);
            int ra_state = digitalRead(ENC_RIGHT_FRONT_A);
            int rb_state = digitalRead(ENC_RIGHT_FRONT_B);
            int duty_LA = ledcRead(CH_L_A);
            int duty_LB = ledcRead(CH_L_B);
            int duty_RA = ledcRead(CH_R_A);
            int duty_RB = ledcRead(CH_R_B);
            Serial.printf("DBG PIN states: LA=%d LB=%d | RA=%d RB=%d\n", la_state, lb_state, ra_state, rb_state);
            Serial.printf("DBG LEDC duty: LA=%d LB=%d | RA=%d RB=%d\n", duty_LA, duty_LB, duty_RA, duty_RB);
          } else {
            Serial.println("Unknown command");
          }
        }
        cmdIdx = 0; // reset buffer
      } else {
        // store character if space
        if (cmdIdx < (int)sizeof(cmdBuf)-1) {
          cmdBuf[cmdIdx++] = ch;
        } else {
          // overflow -> reset
          cmdIdx = 0;
        }
      }
    }
  }

  // small RX watchdog: if no packet has been received for a while, allow monitor to wake it by printing info
  static unsigned long lastRxTime = 0;
  if (got) lastRxTime = millis();
  else {
    if (millis() - lastRxTime > 5000) {
      // every 5s when idle, print a short heartbeat so monitor users see it's alive
      Serial.print(".");
      lastRxTime = millis();
    }
  }

  // periodic print and RPM calculation
  unsigned long now = millis();
  if (now - lastPrint > PRINT_INTERVAL) {
    unsigned long dt = now - lastPrint; // ms
    static int16_t prevLeft = 0;
    static int16_t prevRight = 0;
    int deltaL = (int)tickLeft - (int)prevLeft;
    int deltaR = (int)tickRight - (int)prevRight;
    // handle possible rollover of signed 16-bit
    if (deltaL > 10000) deltaL -= 32768;
    if (deltaL < -10000) deltaL += 32768;
    if (deltaR > 10000) deltaR -= 32768;
    if (deltaR < -10000) deltaR += 32768;

    // RPM = (delta_counts / ticks_per_rev) * (60000 / dt_ms)
    double rpmL = ((double)deltaL / (double)TICKS_PER_REV) * (60000.0 / (double)dt);
    double rpmR = ((double)deltaR / (double)TICKS_PER_REV) * (60000.0 / (double)dt);

  int la_state = digitalRead(ENC_LEFT_FRONT_A);
  int lb_state = digitalRead(ENC_LEFT_FRONT_B);
  int ra_state = digitalRead(ENC_RIGHT_FRONT_A);
  int rb_state = digitalRead(ENC_RIGHT_FRONT_B);
  int duty_LA = ledcRead(CH_L_A);
  int duty_LB = ledcRead(CH_L_B);
  int duty_RA = ledcRead(CH_R_A);
  int duty_RB = ledcRead(CH_R_B);

  // Print one line per wheel for easier debugging
  Serial.printf("LEFT  | tick=%d | d=%d | RPM=%.2f | PWM_A=%d PWM_B=%d | LA=%d LB=%d\n",
          tickLeft, deltaL, rpmL, duty_LA, duty_LB, la_state, lb_state);
  Serial.printf("RIGHT | tick=%d | d=%d | RPM=%.2f | PWM_A=%d PWM_B=%d | RA=%d RB=%d\n",
          tickRight, deltaR, rpmR, duty_RA, duty_RB, ra_state, rb_state);

    // print small diagnostics: free heap and last packet age
    unsigned long packet_age = (millis() - lastPacketMillis);
    Serial.printf("DBG: freeHeap=%u lastPktAge=%lums RPM_L=%.2f RPM_R=%.2f\n", ESP.getFreeHeap(), packet_age, rpmL, rpmR);

    prevLeft = tickLeft;
    prevRight = tickRight;
    lastPrint = now;
  }

  delay(10);
}
