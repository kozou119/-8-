#ifndef HARDWARE_PORT_H
#define HARDWARE_PORT_H
#include <stdint.h>
// Course robot: Trigger/Echo ultrasonic + LSM303 + passive cup arm.
// observedAt is the oldest observation, including cached ultrasonic data.
// Heading convention: clockwise degrees [0, 360), same reference as goalHeading.
// rangeMm: -1 means a successful observation with no target; an I/O error returns false.
// held is a conservative ultrasonic estimate, not physical confirmation or points.
struct NavigationFrame {
  uint32_t observedAt = 0;
  float heading = 0;
  float slopeDeg = 0;
  int rangeMm = -1;
  bool held = false;
};
bool navigationBegin();
bool navigationCalibrate();
bool navigationRead(NavigationFrame &frame);
bool captureGoalHeading(float &heading);
#endif
