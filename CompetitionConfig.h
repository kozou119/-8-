#ifndef COMPETITION_CONFIG_H
#define COMPETITION_CONFIG_H
#include "CompetitionController.h"
// All motor speeds/times below are provisional. Do not enable before bench/field tests.
constexpr uint8_t ROBOT_ID = 1;
inline contest::Config competitionConfig() {
  contest::Config c;
  c.goal = contest::RED;           // Change RED/BLUE before each match.
  c.goalHeading = 0.0f;           // Replaced by own-goal reference captured during setup.
  c.measured = false;             // Remains false until hardware and geometry verified.
  return c;
}
#endif
