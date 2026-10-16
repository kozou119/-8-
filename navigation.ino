#include <LSM303.h>
#include "HardwarePort.h"
#include "RobotHardwareConfig.h"
#include "Ultrasonic.h"
#include "CaptureEstimator.h"
#include "CompassMath.h"

LSM303 compass;
TriggerEchoRanger ranger(hardware::ULTRASONIC_TRIGGER, hardware::ULTRASONIC_ECHO);
CaptureEstimator cupEstimate(hardware::CAPTURE_MIN_MM, hardware::CAPTURE_MAX_MM);
compassmath::Calibration magneticCalibration;
static bool navigationCalibrated = false;

static bool readCompassRaw() {
  compass.readAcc();
  if (compass.timeoutOccurred() || compass.last_status != 0) return false;
  compass.readMag();
  if (compass.timeoutOccurred() || compass.last_status != 0) return false;
  if (compass.getDeviceType()!=LSM303::device_D &&
      (compass.m.x==-4096 || compass.m.y==-4096 || compass.m.z==-4096)) return false;
  return true;
}

bool navigationBegin() {
  navigationCalibrated = false;
  if (!hardware::TRIGGER_ECHO_CONFIRMED || !ranger.begin()) return false;
  compass.setTimeout(20); // Pololu library timeout is in milliseconds, Wire is in microseconds.
  if (!compass.init()) return false; // Detect supported LSM303 variant/address.
  compass.enableDefault();
  if (compass.last_status != 0) return false;
  // enableDefault() only gives 6.25/7.5Hz magnetics. Use a variant-specific fast rate.
  const uint8_t rateRegister = compass.getDeviceType()==LSM303::device_D
    ? LSM303::CTRL5 : LSM303::CRA_REG_M;
  const uint8_t rateValue = compass.getDeviceType()==LSM303::device_D ? 0x70 : 0x18;
  compass.writeMagReg(rateRegister,rateValue); // D: 50Hz high resolution; others: 75Hz.
  if (compass.last_status != 0 || compass.readMagReg(rateRegister)!=rateValue ||
      compass.last_status != 0) return false;
  delay(25); // First conversion, setup only.
  compass.timeoutOccurred();
  return readCompassRaw();
}

bool navigationCalibrate() {
  magneticCalibration = compassmath::Calibration();
  Serial.println(F("COMPASS: press/release, then turn robot by hand on a level surface through 2 full turns."));
  Serial.println(F("Press again to finish (max 20 seconds). Motors remain OFF."));
  button.waitForButton();
  uint32_t began = millis();
  while (uint32_t(millis()-began)<20000) {
    if (!readCompassRaw()) return false;
    float slope;
    if (compassmath::slope(compass.a.x,compass.a.y,compass.a.z,slope) && slope<15)
      magneticCalibration.add(compass.m.x,compass.m.y);
    if (button.getSingleDebouncedPress() && uint32_t(millis()-began)>1000) break;
    delay(20); // Setup only, with motors off.
  }
  navigationCalibrated = magneticCalibration.valid();
  Serial.println(navigationCalibrated ? F("COMPASS calibrated") : F("FAULT: insufficient compass range/samples"));
  return navigationCalibrated;
}

bool navigationRead(NavigationFrame &frame) {
  if (!navigationCalibrated) return false;
  uint32_t compassAt = millis()-20; // Include the configured sensor conversion interval.
  if (!readCompassRaw()) return false;
  if (!magneticCalibration.heading(compass.m.x,compass.m.y,hardware::COMPASS_CLOCKWISE_SIGN,
        hardware::COMPASS_OFFSET_DEG,frame.heading)) return false;
  if (!compassmath::slope(compass.a.x,compass.a.y,compass.a.z,frame.slopeDeg)) return false;
  uint32_t rangeAt;
  if (!ranger.read(frame.rangeMm,rangeAt)) return false;
  uint32_t now = millis();
  frame.observedAt = uint32_t(now-compassAt)>uint32_t(now-rangeAt) ? compassAt : rangeAt;
  frame.held = cupEstimate.update(frame.rangeMm,rangeAt,now);
  return true;
}

bool captureGoalHeading(float &heading) {
  Serial.println(F("GOAL: point robot toward your own goal, press/release button."));
  button.waitForButton();
  float x=0,y=0;
  for (unsigned i=0;i<12;++i) {
    NavigationFrame frame;
    if (!navigationRead(frame) || frame.slopeDeg>15) return false;
    float rad=frame.heading*0.0174532925f;
    x+=cosf(rad); y+=sinf(rad);
    delay(20);
  }
  if (x*x+y*y<100) return false; // Reject an unstable direction during reference capture.
  heading=atan2f(y,x)*57.2957795f;
  if (heading<0) heading+=360;
  return true;
}
