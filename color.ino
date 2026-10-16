#include <Adafruit_TCS34725.h>
#include "ColorSensor.h"
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);
static float references[4][3]; // black, white, red, blue; current field/lighting only.
static bool calibrated = false;

bool colorBegin() { return tcs.begin(); }

static bool rawColor(float *rgb) {
  // Renesas Wire has no getWireTimeoutFlag(); test the TCS34725 ACK explicitly.
  Wire.beginTransmission(0x29);
  if (Wire.endTransmission() != 0) return false;
  uint16_t r, g, b, clear;
  tcs.getRawData(&r, &g, &b, &clear);
  // 2.4 ms integration saturates at 1024 counts, not 65535.
  if (clear == 0 || clear >= 1024 || r >= 1024 || g >= 1024 || b >= 1024) return false;
  rgb[0] = r; rgb[1] = g; rgb[2] = b;
  return true;
}

bool colorCalibrate() {
  calibrated = false;
  for (uint8_t surface = 0; surface < 4; ++surface) {
    Serial.print(F("Place sensor on: "));
    switch (surface) {
      case 0: Serial.println(F("BLACK, then press/release button")); break;
      case 1: Serial.println(F("WHITE, then press/release button")); break;
      case 2: Serial.println(F("RED, then press/release button")); break;
      case 3: Serial.println(F("BLUE, then press/release button")); break;
    }
    button.waitForButton();
    for (uint8_t channel = 0; channel < 3; ++channel) references[surface][channel] = 0;
    for (uint8_t sample = 0; sample < 20; ++sample) {
      float rgb[3];
      if (!rawColor(rgb)) return false;
      for (uint8_t channel = 0; channel < 3; ++channel)
        references[surface][channel] += rgb[channel] / 20.0f;
      delay(5); // Calibration only; no moving motors.
    }
  }
  for (uint8_t channel = 0; channel < 3; ++channel)
    if (references[1][channel] - references[0][channel] < 25) return false;
  // Reject indistinguishable calibration surfaces.
  for (uint8_t a = 0; a < 4; ++a) for (uint8_t b = a + 1; b < 4; ++b) {
    float distance = 0;
    for (uint8_t channel = 0; channel < 3; ++channel) {
      float d = (references[a][channel] - references[b][channel]) /
                (references[1][channel] - references[0][channel]);
      distance += d * d;
    }
    if (distance < 0.12f) return false;
  }
  calibrated = true;
  return true;
}

bool colorRead(contest::Floor &floor) {
  floor = contest::UNKNOWN;
  float rgb[3];
  if (!calibrated || !rawColor(rgb)) return false;
#if defined(WIRE_HAS_TIMEOUT)
  if (Wire.getWireTimeoutFlag()) { Wire.clearWireTimeoutFlag(); return false; }
#endif
  float best = 1000000, second = 1000000;
  uint8_t closest = 0;
  for (uint8_t surface = 0; surface < 4; ++surface) {
    float distance = 0;
    for (uint8_t channel = 0; channel < 3; ++channel) {
      float d = (rgb[channel] - references[surface][channel]) /
                (references[1][channel] - references[0][channel]);
      distance += d * d;
    }
    if (distance < best) { second = best; best = distance; closest = surface; }
    else if (distance < second) second = distance;
  }
  if (best > 0.30f || second - best < 0.04f) return true; // Mixed/unknown: controller stops.
  const contest::Floor labels[] = {contest::BLACK, contest::WHITE, contest::RED, contest::BLUE};
  floor = labels[closest];
  return true;
}
