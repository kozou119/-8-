#include <Wire.h>
#include <stdio.h>
#include "RobotHardwareConfig.h"
#include "BoardIO.h"
#include "CompetitionConfig.h"
#include "HardwarePort.h"
#include "ColorSensor.h"

#if !defined(ARDUINO_UNOR4_MINIMA)
#error "Select Arduino UNO R4 Minima (arduino:renesas_uno:minima)."
#endif
robotio::Motors motors(robotio::MotorPins(hardware::LEFT_PWM, hardware::LEFT_DIR,
  hardware::RIGHT_PWM, hardware::RIGHT_DIR, hardware::FLIP_LEFT, hardware::FLIP_RIGHT));
robotio::Button button(hardware::BUTTON_PIN);
contest::Controller controller(competitionConfig());
bool ready = false;
bool sensorsPrepared = false;
uint32_t lastTick = 0, lastTelemetry = 0;

void setup() {
  Serial.begin(115200);  // USB setup/calibration log; never wait for a PC connection.
  Serial1.begin(9600);  // Existing ZigBee link.
  const int pinLayout[] = {hardware::LEFT_PWM, hardware::LEFT_DIR, hardware::RIGHT_PWM,
    hardware::RIGHT_DIR, hardware::BUTTON_PIN, hardware::ULTRASONIC_TRIGGER,
    hardware::ULTRASONIC_ECHO, hardware::CUP_SWITCH_PIN};
  if (!robotio::pinLayoutValid(pinLayout, sizeof(pinLayout) / sizeof(pinLayout[0]))) {
    Serial.println(F("FAULT: pin conflict or reserved UART/I2C pin"));
    return;
  }
  if (!motors.begin() || !button.begin()) {
    Serial.println(F("FAULT: configure and verify motor driver/button pins in RobotHardwareConfig.h"));
    return;
  }
  motors.setSpeeds(0, 0);
  Wire.begin();
  // Renesas Wire has this API but does not define WIRE_HAS_TIMEOUT.
  Wire.setWireTimeout(25000, true);
  if (!colorBegin()) {
    Serial.println(F("FAULT: TCS34725 not found"));
    return;
  }
  if (!navigationBegin()) {
    Serial.println(F("FAULT: LSM303 or ultrasonic initialization failed"));
    return;
  }
  // No motors are driven during calibration. Move the robot by hand BEFORE the match.
  bool calibrated = navigationCalibrate() && colorCalibrate();
  float goalHeading;
  if (calibrated) calibrated = captureGoalHeading(goalHeading) && controller.setGoalHeading(goalHeading);
  sensorsPrepared = calibrated;
  ready = calibrated && competitionConfig().measured;
  if (calibrated && !competitionConfig().measured)
    Serial.println(F("SETUP ONLY: measured=false; verify arm range, PWM, push distance before enabling"));
  Serial.println(ready ? F("READY: place at assigned start, then press button")
                       : F("FAULT: color calibration failed"));
}

void loop() {
  const uint32_t now = millis();
  if (now - lastTick < 20) return;
  lastTick = now;
  contest::Input in;
  NavigationFrame frame;
  contest::Floor floor = contest::UNKNOWN;
  bool colorOK = sensorsPrepared && colorRead(floor);
  bool navigationOK = sensorsPrepared && navigationRead(frame);
  in.now = millis();  // Timestamp AFTER sensor I/O; delayed reads must not look fresh.
  in.observedAt = frame.observedAt;
  in.floor = floor;
  in.heading = frame.heading;
  in.slopeDeg = frame.slopeDeg;
  in.rangeMm = frame.rangeMm;
  in.held = frame.held;
  in.sensorsOK = colorOK && navigationOK;
  bool pressed = button.getSingleDebouncedPress();
  if (ready && pressed && controller.state() == contest::IDLE) controller.start(in);
  else in.stop = pressed;
  contest::Output output = controller.tick(in);
  motors.setSpeeds(output.left, output.right); // Exactly one motor update, after decisions.
  // Short CSV records, nonblocking, maximum 5 Hz at 9600 baud.
  // V1,id,elapsed_ms,state,reason,floor,range_mm,heading_deg,held,left,right,attempts,healthy
  if (in.now - lastTelemetry >= 200) {
    lastTelemetry = in.now;
    char line[100];
    int length = snprintf(line, sizeof(line), "V1,%u,%lu,%u,%u,%u,%d,%d,%u,%d,%d,%u,%u\n",
      (unsigned)ROBOT_ID, (unsigned long)controller.elapsed(in.now), (unsigned)controller.state(),
      (unsigned)(ready ? controller.reason() : contest::NOT_CONFIGURED), (unsigned)floor,
      frame.rangeMm, isfinite(frame.heading) && frame.heading >= 0 && frame.heading < 360
        ? (int)frame.heading : -1, frame.held ? 1u : 0u,
      output.left, output.right, (unsigned)controller.deliveryAttempts(), in.sensorsOK ? 1u : 0u);
    if (length > 0 && length < (int)sizeof(line) && Serial1.availableForWrite() >= length)
      Serial1.write((const uint8_t *)line, length);
  }
}
