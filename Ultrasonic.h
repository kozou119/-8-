#ifndef ROBOT_ULTRASONIC_H
#define ROBOT_ULTRASONIC_H
#include <Arduino.h>
#include <stdint.h>
// Optional driver for CONFIRMED 10us-trigger / positive-echo-width transducers.
// This protocol is not shared by every ultrasonic sensor; no model is assumed.
class TriggerEchoRanger {
 public:
  TriggerEchoRanger(int trigger, int echo) : trigger(trigger), echo(echo) {}
  bool begin() {
    if (trigger < 2 || trigger >= 18 || echo < 2 || echo >= 18 || trigger == echo) return false;
    pinMode(trigger, OUTPUT); digitalWrite(trigger, LOW); pinMode(echo, INPUT);
    enabled = true; return true;
  }
  bool read(int &rangeMm, uint32_t &observedAt) {
    if (!enabled) return false;
    const uint32_t now = millis();
    if (sampled && uint32_t(now - sampleAt) < 60) {
      rangeMm = cachedMm; observedAt = sampleAt; return true;
    }
    if (digitalRead(echo) == HIGH) return false; // Stuck echo or bad wiring.
    digitalWrite(trigger, LOW); delayMicroseconds(2);
    digitalWrite(trigger, HIGH); delayMicroseconds(10); digitalWrite(trigger, LOW);
    // Bounded blocking: 12ms (~2m at room temperature); no echo = no target.
    unsigned long us = pulseIn(echo, HIGH, 12000UL);
    if (us == 0 && digitalRead(echo) == HIGH) return false;
    if (us > 0 && us < 100) return false; // Invalid near-field pulse; never a held cup.
    cachedMm = us == 0 ? -1 : int(us * 343UL / 2000UL);
    sampleAt = millis(); sampled = true;
    rangeMm = cachedMm; observedAt = sampleAt;
    return true;
  }
 private:
  int trigger, echo, cachedMm = -1;
  uint32_t sampleAt = 0;
  bool enabled = false, sampled = false;
};
#endif
