#include "CompetitionController.h"
#include <assert.h>
#include <stdio.h>
#include <limits>
#include <stdlib.h>
#include <initializer_list>
using namespace contest;
static unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); abort(); } } while (0)
static Config config(Floor goal = RED) {
  Config c; c.measured = true; c.goal = goal; c.goalHeading = 0;
  return c;
}
static Input sample(uint32_t now, Floor floor = WHITE, bool held = false, int mm = -1,
                    float heading = 0) {
  Input in; in.now = in.observedAt = now; in.sensorsOK = true;
  in.floor = floor; in.held = held; in.rangeMm = mm; in.heading = heading;
  return in;
}
static void stopped(Output o) { CHECK(o.left == 0 && o.right == 0); }
static void entered(Controller &c, uint32_t origin = 0) {
  CHECK(c.start(sample(origin, BLACK)));
  c.tick(sample(origin + 450)); CHECK(c.state() == SEARCH);
}
static void delivering(Controller &c) {
  entered(c);
  c.tick(sample(500, WHITE, true)); CHECK(c.state() == ALIGN);
  c.tick(sample(520, WHITE, true)); CHECK(c.state() == DELIVER);
}
static void testLegalDelivery(Floor goal) {
  Controller c(config(goal)); delivering(c);
  Output o = c.tick(sample(600, WHITE, true)); CHECK(o.left > 0 && o.right > 0);
  stopped(c.tick(sample(700, goal, true))); CHECK(c.state() == DELIVER);
  stopped(c.tick(sample(760, goal, true))); CHECK(c.state() == PUSH);
  o = c.tick(sample(780, goal, true)); CHECK(o.left > 0 && o.right > 0);
  c.tick(sample(1110, WHITE, true)); CHECK(c.state() == RELEASE);
  o = c.tick(sample(1130, WHITE, false)); CHECK(o.left < 0 && o.right < 0);
  c.tick(sample(1460, WHITE, false)); CHECK(c.deliveryAttempts() == 1);
  c.tick(sample(2060)); CHECK(c.state() == SEARCH);
}
int main() {
  {
    Controller c(config()); CHECK(c.setGoalHeading(359));
    CHECK(!c.setGoalHeading(360)); CHECK(!c.setGoalHeading(-1));
    entered(c); CHECK(!c.setGoalHeading(90));
    Input hill=sample(500); hill.slopeDeg=25;
    stopped(c.tick(hill)); CHECK(c.state()==RETREAT && c.reason()==SLOPE);
    hill.now=hill.observedAt=520;
    Output o=c.tick(hill); CHECK(o.left<0 && o.right<0);
    hill.now=hill.observedAt=850;
    stopped(c.tick(hill)); CHECK(c.state()==FAULT && c.reason()==SLOPE);
  }
  {
    Controller c(config()); Input in=sample(0); in.slopeDeg=30;
    CHECK(!c.start(in)); CHECK(c.reason()==SLOPE);
  }
  {
    Config bad; Controller c(bad);
    CHECK(!c.start(sample(0))); CHECK(c.reason() == NOT_CONFIGURED);
    stopped(c.tick(sample(100)));
  }
  for (Floor goal : {RED, BLUE}) testLegalDelivery(goal);
  {
    Controller c(config()); delivering(c);
    stopped(c.tick(sample(600, BLUE, true))); CHECK(c.state() == RETREAT);
    CHECK(c.deliveryAttempts() == 0);
  }
  // Black boundary interrupts EVERY moving phase, including a goal push.
  for (int stage = 0; stage < 5; ++stage) {
    Controller c(config()); entered(c);
    if (stage >= 1) c.tick(sample(500, WHITE, true));
    if (stage >= 2) c.tick(sample(520, WHITE, true));
    if (stage >= 3) { c.tick(sample(600, RED, true)); c.tick(sample(660, RED, true)); }
    if (stage >= 4) c.tick(sample(1010, RED, true));
    stopped(c.tick(sample(1030, BLACK, true))); CHECK(c.state() == RETREAT);
    CHECK(c.deliveryAttempts() == 0);
  }
  {
    Controller c(config()); delivering(c);
    stopped(c.tick(sample(600, WHITE, false))); CHECK(c.state() == SEARCH);
    CHECK(c.reason() == LOST_CUP);
  }
  {
    Controller c(config()); delivering(c);
    c.tick(sample(600, RED, true));
    c.tick(sample(660, RED, true));
    stopped(c.tick(sample(680, RED, false))); CHECK(c.state() == RETREAT);
    CHECK(c.deliveryAttempts() == 0);
  }
  {
    Controller c(config()); delivering(c);
    stopped(c.tick(sample(600, RED, true, -1, 90))); CHECK(c.state() == RETREAT);
  }
  {
    Controller c(config()); delivering(c);
    stopped(c.tick(sample(600, RED, true)));
    c.tick(sample(640, WHITE, true));
    stopped(c.tick(sample(660, RED, true))); CHECK(c.state() == DELIVER);
    stopped(c.tick(sample(720, RED, true))); CHECK(c.state() == PUSH);
    stopped(c.tick(sample(740, UNKNOWN, true))); CHECK(c.state() == FAULT);
    stopped(c.tick(sample(1400, WHITE, false))); CHECK(c.deliveryAttempts() == 0);
  }
  {
    Controller c(config()); delivering(c);
    c.tick(sample(600, RED, true)); c.tick(sample(660, RED, true));
    c.tick(sample(1010, WHITE, true)); CHECK(c.state() == RELEASE);
    c.tick(sample(1360, WHITE, true)); CHECK(c.deliveryAttempts() == 0);
  }
  {
    Controller c(config()); entered(c);
    c.tick(sample(500, WHITE, false, 300));
    CHECK(c.state() == SEARCH); // One noisy return must not trigger an approach.
    c.tick(sample(580, WHITE, false, 300)); CHECK(c.state() == APPROACH);
    c.tick(sample(600, WHITE, false, 40));
    c.tick(sample(1520, WHITE, false, 40)); CHECK(c.state() == RETREAT);
    CHECK(c.deliveryAttempts() == 0); // Distance alone never counts as a capture.
  }
  {
    Controller c(config()); entered(c);
    Input old = sample(700); old.observedAt = 500;
    stopped(c.tick(old)); CHECK(c.state() == FAULT && c.reason() == SENSOR);
  }
  {
    Controller c(config()); entered(c);
    Input bad = sample(500); bad.heading = std::numeric_limits<float>::quiet_NaN();
    stopped(c.tick(bad)); CHECK(c.state() == FAULT);
  }
  {
    Controller c(config()); entered(c);
    Input bad = sample(500); bad.sensorsOK = false;
    stopped(c.tick(bad)); CHECK(c.state() == FAULT);
  }
  {
    Controller c(config()); entered(c);
    stopped(c.tick(sample(500, UNKNOWN))); CHECK(c.state() == SEARCH);
    stopped(c.tick(sample(800, UNKNOWN))); CHECK(c.state() == FAULT);
  }
  {
    Controller c(config()); CHECK(c.start(sample(0, BLACK)));
    stopped(c.tick(sample(450, BLACK))); CHECK(c.state() == FAULT);
  }
  {
    Controller c(config()); entered(c);
    Input stop = sample(500); stop.stop = true;
    stopped(c.tick(stop)); CHECK(c.reason() == MANUAL_STOP);
    CHECK(!c.start(sample(600))); // Accidental button press cannot restart the match clock.
  }
  {
    Controller c(config()); entered(c);
    stopped(c.tick(sample(180000))); CHECK(c.reason() == TIME_UP);
    stopped(c.tick(sample(180020))); CHECK(!c.start(sample(180040)));
  }
  {
    uint32_t origin = UINT32_MAX - 200;
    Controller c(config()); entered(c, origin);
    CHECK(c.elapsed(origin + 500) == 500);
    stopped(c.tick(sample(origin + 180000))); CHECK(c.reason() == TIME_UP);
  }
  CHECK(fabsf(angleError(2, 358) - 4) < 0.001f);
  CHECK(fabsf(angleError(358, 2) + 4) < 0.001f);
  // Long deterministic adversarial run: output bounds and no forward black crossing.
  uint32_t rng = 12345;
  for (unsigned round = 0; round < 100; ++round) {
    Controller c(config()); entered(c);
    for (uint32_t ms = 500; ms < 6000; ms += 20) {
      rng = rng * 1664525u + 1013904223u;
      Floor floor = (rng % 17 == 0) ? BLACK : WHITE;
      Input in = sample(ms, floor, rng % 5 == 0, 50 + rng % 900, rng % 360);
      Output o = c.tick(in);
      CHECK(abs(o.left) <= 180 && abs(o.right) <= 180);
      if (floor == BLACK) CHECK(!(o.left > 0 && o.right > 0));
    }
  }
  printf("PASS: %u assertions; delivery, boundary, sensor, timing and adversarial scenarios\n", checks);
}
