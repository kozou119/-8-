#ifndef ROBOT_BOARD_IO_H
#define ROBOT_BOARD_IO_H
#include <Arduino.h>
#include <stdint.h>
namespace robotio {
inline bool digitalPinValid(int pin) { return pin >= 0 && pin <= 19; }
inline bool pinLayoutValid(const int *pins, unsigned count) {
  for (unsigned i=0; i<count; ++i) {
    if (pins[i] == -1) continue; // Unconfigured optional connection.
    if (pins[i] < 2 || pins[i] >= 18) return false; // UART and I2C reserved.
    for (unsigned j=0; j<i; ++j) if (pins[i] == pins[j]) return false;
  }
  return true;
}
struct MotorPins {
  int leftPwm, leftDir, rightPwm, rightDir;
  bool flipLeft, flipRight;
  MotorPins(int lp, int ld, int rp, int rd, bool fl = false, bool fr = false)
    : leftPwm(lp), leftDir(ld), rightPwm(rp), rightDir(rd), flipLeft(fl), flipRight(fr) {}
};
class Motors {
 public:
  explicit Motors(const MotorPins &pins) : pins(pins) {}
  bool begin() {
    const int list[] = {pins.leftPwm, pins.leftDir, pins.rightPwm, pins.rightDir};
    for (unsigned i=0; i<4; ++i) {
      // D0/D1 reserved for ZigBee Serial1; A4/A5 reserved for I2C.
      if (!digitalPinValid(list[i]) || list[i] < 2 || list[i] >= 18) return false;
      for (unsigned j=0; j<i; ++j) if (list[i] == list[j]) return false;
    }
    if (!digitalPinHasPWM(pins.leftPwm) || !digitalPinHasPWM(pins.rightPwm)) return false;
    analogWriteResolution(8); // UNO R4: explicit 0..255 duty range.
    // Establish zero duty before configuring direction outputs.
    pinMode(pins.leftPwm, OUTPUT); analogWrite(pins.leftPwm, 0);
    pinMode(pins.rightPwm, OUTPUT); analogWrite(pins.rightPwm, 0);
    pinMode(pins.leftDir, OUTPUT); digitalWrite(pins.leftDir, LOW);
    pinMode(pins.rightDir, OUTPUT); digitalWrite(pins.rightDir, LOW);
    enabled = true;
    return true;
  }
  void setSpeeds(int left, int right) {
    if (!enabled) return;
    channel(left, pins.leftPwm, pins.leftDir, pins.flipLeft);
    channel(right, pins.rightPwm, pins.rightDir, pins.flipRight);
  }
 private:
  MotorPins pins;
  bool enabled = false;
  static void channel(int speed, int pwm, int dir, bool flip) {
    speed = speed < -400 ? -400 : (speed > 400 ? 400 : speed);
    bool reverse = speed < 0;
    int magnitude = reverse ? -speed : speed;
    // Remove PWM before changing the direction input.
    analogWrite(pwm, 0);
    digitalWrite(dir, (reverse != flip) ? HIGH : LOW);
    analogWrite(pwm, magnitude * 255L / 400);
  }
};
class Button {
 public:
  explicit Button(int pin) : pin(pin) {}
  bool begin() {
    if (!digitalPinValid(pin) || pin < 2 || pin >= 18) return false;
    pinMode(pin, INPUT_PULLUP);
    lastRaw = stable = digitalRead(pin) == LOW;
    changedAt = millis(); enabled = true;
    return true;
  }
  bool getSingleDebouncedPress() {
    if (!enabled) return false;
    bool raw = digitalRead(pin) == LOW;
    uint32_t now = millis();
    if (raw != lastRaw) { lastRaw = raw; changedAt = now; }
    if (raw == stable || uint32_t(now - changedAt) < 20) return false;
    stable = raw;
    return stable;
  }
  void waitForButton() {
    if (!enabled) return;
    // Require a fresh press; the previous calibration press must be released first.
    while (digitalRead(pin) == LOW) { getSingleDebouncedPress(); delay(1); }
    while (!getSingleDebouncedPress()) delay(1);
    while (digitalRead(pin) == LOW || stable) { getSingleDebouncedPress(); delay(1); }
  }
 private:
  int pin;
  bool enabled = false, lastRaw = false, stable = false;
  uint32_t changedAt = 0;
};
}
#endif
