#ifndef ROBOT_CAPTURE_ESTIMATOR_H
#define ROBOT_CAPTURE_ESTIMATOR_H
#include <stdint.h>
// Passive arm: estimate capture from THREE DISTINCT, stable near-range observations.
// This cannot prove cup identity/retention and is never a scoring sensor.
class CaptureEstimator {
 public:
  CaptureEstimator(int minimumMm, int maximumMm) : low(minimumMm), high(maximumMm) {}
  bool update(int mm, uint32_t observedAt, uint32_t now) {
    if (low < 20 || high <= low || uint32_t(now-observedAt) > 150 || mm < low || mm > high) {
      count=0; held=false; haveSample=false; return false;
    }
    if (haveSample && observedAt==lastSample) return held;
    int difference=mm-lastMm; if (difference<0) difference=-difference;
    if (!haveSample || difference>15 || uint32_t(observedAt-lastSample)>150) {
      since=observedAt; count=0; held=false;
    }
    haveSample=true; lastSample=observedAt; lastMm=mm;
    if (count<3) ++count;
    held=count>=3 && uint32_t(observedAt-since)>=120;
    return held;
  }
 private:
  int low, high, lastMm=0;
  uint32_t since=0, lastSample=0;
  unsigned count=0;
  bool held=false, haveSample=false;
};
#endif
