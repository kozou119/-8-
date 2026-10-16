#ifndef COMPETITION_CONTROLLER_H
#define COMPETITION_CONTROLLER_H
#include <stdint.h>
#include <math.h>

namespace contest {
enum Floor : uint8_t { UNKNOWN, WHITE, BLACK, RED, BLUE };
enum State : uint8_t { IDLE, ENTER, SEARCH, APPROACH, ALIGN, DELIVER, PUSH,
                       RELEASE, RETREAT, TURN, FINISHED, FAULT };
enum Reason : uint8_t { OK, NOT_CONFIGURED, SENSOR, TIME_UP, MANUAL_STOP,
                        BOUNDARY, LOST_CUP, TIMEOUT, SLOPE };
struct Config {
  Floor goal = RED;
  float goalHeading = 0;             // Measured in each arena; clockwise degrees.
  uint32_t matchMs = 180000;
  bool measured = false;             // Enable ONLY after hardware/geometry validation.
  int cruise = 140, approach = 100, carry = 120, rotate = 120;
  int maxMotor = 180;
  uint16_t targetMm = 650, captureMm = 65;
  uint32_t enterMs = 450, pushMs = 350, reverseMs = 350, turnMs = 600;
};
struct Input {
  uint32_t now = 0, observedAt = 0;
  Floor floor = UNKNOWN;
  bool sensorsOK = false;            // Fresh floor/range/heading/grip readings.
  bool held = false;                 // Passive arm: stable near-range estimate, not proof.
  int rangeMm = -1;                  // -1 = valid reading with no object in range.
  float heading = 0;
  float slopeDeg = 0;
  bool stop = false;
};
struct Output { int left = 0, right = 0; };
inline float angleError(float target, float actual) {
  float error = fmodf(target - actual + 540.0f, 360.0f) - 180.0f;
  return error;
}
class Controller {
 public:
  explicit Controller(const Config &config) : cfg(config) {}
  State state() const { return phase; }
  Reason reason() const { return why; }
  uint16_t deliveryAttempts() const { return attempts; } // Never an asserted score.
  bool setGoalHeading(float heading) {
    if (phase!=IDLE || !isfinite(heading) || heading<0 || heading>=360) return false;
    cfg.goalHeading=heading; return true;
  }
  uint32_t elapsed(uint32_t now) const { return begun ? now - started : 0; }
  bool start(const Input &in) {
    if (phase != IDLE) return false;
    if (!configurationOK()) { transition(FAULT, in.now, NOT_CONFIGURED); return false; }
    if (!healthy(in)) { transition(FAULT, in.now, SENSOR); return false; }
    if (in.slopeDeg>18) { transition(FAULT,in.now,SLOPE); return false; }
    begun = true;
    started = in.now;
    transition(ENTER, in.now);
    return true;
  }
  Output tick(const Input &in) {
    Output out;
    if (phase == IDLE || phase == FINISHED || phase == FAULT) return out;
    if (in.stop) { transition(FINISHED, in.now, MANUAL_STOP); return out; }
    if (elapsed(in.now) >= cfg.matchMs) {
      transition(FINISHED, in.now, TIME_UP); return out;
    }
    if (!healthy(in)) { transition(FAULT, in.now, SENSOR); return out; }
    const uint32_t age = in.now - entered;
    // Unknown floor: stop immediately; tolerate a brief mixed-color transition.
    if (in.floor == UNKNOWN) {
      goalSeen = false;
      // Never let a paused, timed deposit complete as if the motors had kept moving.
      if (phase == PUSH || phase == RELEASE) {
        transition(FAULT, in.now, SENSOR); return out;
      }
      if (!unknown) { unknown = true; unknownSince = in.now; }
      if (in.now - unknownSince >= 300) transition(FAULT, in.now, SENSOR);
      return out;
    }
    unknown = false;
    // Flat-ground collector: back off a hill instead of using a tilted 2-D heading.
    if (in.slopeDeg>18 && phase!=RETREAT) {
      if (phase==TURN) transition(FAULT,in.now,SLOPE);
      else transition(RETREAT,in.now,SLOPE);
      return out;
    }
    // Entry permits only the line the robot starts on, for a bounded interval.
    if (phase == ENTER) {
      if ((in.floor == RED || in.floor == BLUE) && in.floor != cfg.goal) {
        transition(FAULT, in.now, BOUNDARY); return out;
      }
      if (age >= cfg.enterMs) {
        if (in.floor != WHITE) transition(FAULT, in.now, BOUNDARY);
        else transition(SEARCH, in.now);
        return out;
      }
      return drive(cfg.approach, cfg.approach);
    }
    // Reverse only a short measured distance; never drive forward across black.
    if (phase == RETREAT) {
      if (age < cfg.reverseMs) return drive(-cfg.approach, -cfg.approach);
      if (in.slopeDeg>18) { transition(FAULT,in.now,SLOPE); return out; }
      transition(TURN, in.now); return out;
    }
    if (phase == TURN) {
      if (age < cfg.turnMs) return drive(-cfg.rotate, cfg.rotate);
      if (in.floor != WHITE) transition(FAULT, in.now, BOUNDARY);
      else transition(in.held ? ALIGN : SEARCH, in.now);
      return out;
    }
    if (in.floor == BLACK) { transition(RETREAT, in.now, BOUNDARY); return out; }
    // Goal crossing is legal only with a held cup, the selected color and alignment.
    if (in.floor == RED || in.floor == BLUE) {
      const bool goalPath = phase == DELIVER || phase == PUSH || phase == RELEASE;
      if (in.floor != cfg.goal || !goalPath ||
          fabsf(angleError(cfg.goalHeading, in.heading)) > 20) {
        transition(RETREAT, in.now, BOUNDARY); return out;
      }
    }
    // Loss before a legal goal crossing abandons delivery, rather than blind pushing.
    if ((phase == ALIGN || phase == DELIVER) && !in.held) {
      transition(SEARCH, in.now, LOST_CUP); return out;
    }
    switch (phase) {
      case SEARCH:
        if (in.held) { transition(ALIGN, in.now); break; }
        if (in.rangeMm > 0 && in.rangeMm <= cfg.targetMm) {
          if (!targetSeen) { targetSeen = true; targetSince = in.now; }
          if (in.now - targetSince >= 80) transition(APPROACH, in.now);
          break; // Stop while confirming, so the target remains in view.
        }
        targetSeen = false;
        // Alternate slow scan and bounded advance; different robots can vary timing.
        out = (age % 4000 < 2200) ? drive(-cfg.rotate, cfg.rotate)
                                      : drive(cfg.cruise, cfg.cruise);
        break;
      case APPROACH:
        if (in.held) { transition(ALIGN, in.now); break; }
        if (age > 4500 || in.rangeMm <= 0 || in.rangeMm > cfg.targetMm + 150) {
          transition(SEARCH, in.now, TIMEOUT); break;
        }
        if (in.rangeMm <= cfg.captureMm) {
          if (!near) { near = true; nearSince = in.now; }
          // An object that cannot be captured may be another robot or the mountain.
          if (in.now - nearSince > 900) { transition(RETREAT, in.now, TIMEOUT); break; }
        } else near = false;
        out = drive(cfg.approach, cfg.approach);
        break;
      case ALIGN: {
        float error = angleError(cfg.goalHeading, in.heading);
        if (age > 5000) { transition(FAULT, in.now, TIMEOUT); break; }
        if (fabsf(error) < 8) { transition(DELIVER, in.now); break; }
        out = error > 0 ? drive(cfg.rotate, -cfg.rotate) : drive(-cfg.rotate, cfg.rotate);
        break;
      }
      case DELIVER: {
        if (age > 18000) { transition(RETREAT, in.now, TIMEOUT); break; }
        if (in.floor == cfg.goal) {
          if (!goalSeen) { goalSeen = true; goalSince = in.now; }
          if (in.now - goalSince >= 60) transition(PUSH, in.now);
          break; // Confirm the line while stationary before committing to the push.
        }
        goalSeen = false;
        float error = angleError(cfg.goalHeading, in.heading);
        if (fabsf(error) > 30) { transition(ALIGN, in.now); break; }
        int correction = static_cast<int>(error * 2.0f);
        out = drive(cfg.carry + correction, cfg.carry - correction);
        break;
      }
      case PUSH:
        // A retained cup must stay confirmed until the measured push is finished.
        if (!in.held) { transition(RETREAT, in.now, LOST_CUP); break; }
        if (age >= cfg.pushMs) { transition(RELEASE, in.now); break; }
        out = drive(cfg.approach, cfg.approach);
        break;
      case RELEASE:
        // Passive scoop: back away without pulling the cup back over the line.
        if (age >= cfg.reverseMs) {
          if (!in.held) ++attempts;
          transition(TURN, in.now, in.held ? LOST_CUP : OK);
          break;
        }
        out = drive(-cfg.approach, -cfg.approach);
        break;
      default: break;
    }
    return out;
  }
 private:
  Config cfg;
  State phase = IDLE;
  Reason why = OK;
  uint32_t entered = 0, started = 0, targetSince = 0, nearSince = 0, unknownSince = 0, goalSince = 0;
  uint16_t attempts = 0;
  bool begun = false, targetSeen = false, near = false, unknown = false, goalSeen = false;
  bool configurationOK() const {
    return cfg.measured && (cfg.goal == RED || cfg.goal == BLUE) &&
      isfinite(cfg.goalHeading) && cfg.goalHeading >= 0 && cfg.goalHeading < 360 &&
      cfg.matchMs > 0 && cfg.matchMs <= 180000 && cfg.enterMs > 0 && cfg.enterMs <= 1000 &&
      cfg.pushMs > 0 && cfg.pushMs <= 1000 && cfg.reverseMs > 0 && cfg.reverseMs <= 1000 &&
      cfg.turnMs > 0 && cfg.turnMs <= 2000 && cfg.captureMm > 0 &&
      cfg.captureMm < cfg.targetMm && cfg.maxMotor > 0 && cfg.maxMotor <= 400 &&
      cfg.cruise > 0 && cfg.cruise <= cfg.maxMotor && cfg.approach > 0 &&
      cfg.approach <= cfg.maxMotor && cfg.carry > 0 && cfg.carry <= cfg.maxMotor &&
      cfg.rotate > 0 && cfg.rotate <= cfg.maxMotor;
  }
  bool healthy(const Input &in) const {
    return in.sensorsOK && in.now - in.observedAt <= 150 &&
      isfinite(in.heading) && in.heading >= 0 && in.heading < 360 &&
      isfinite(in.slopeDeg) && in.slopeDeg>=0 && in.slopeDeg<=90 &&
      (in.rangeMm == -1 || (in.rangeMm > 0 && in.rangeMm <= 5000));
  }
  Output drive(int left, int right) const {
    Output out;
    out.left = left < -cfg.maxMotor ? -cfg.maxMotor : (left > cfg.maxMotor ? cfg.maxMotor : left);
    out.right = right < -cfg.maxMotor ? -cfg.maxMotor : (right > cfg.maxMotor ? cfg.maxMotor : right);
    return out;
  }
  void transition(State next, uint32_t now, Reason reason = OK) {
    phase = next; entered = now; why = reason; targetSeen = false; near = false; goalSeen = false;
  }
};
}
#endif
