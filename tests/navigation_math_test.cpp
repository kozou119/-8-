#include "CaptureEstimator.h"
#include "CompassMath.h"
#include <assert.h>
#include <stdio.h>
#include <limits>
int main() {
  CaptureEstimator estimate(35,75);
  assert(!estimate.update(50,100,100));
  assert(!estimate.update(50,100,140)); // Re-reading cache is not another observation.
  assert(!estimate.update(52,160,160));
  assert(estimate.update(49,220,220));
  assert(!estimate.update(-1,280,280)); // Loss stops delivery immediately.
  assert(!estimate.update(50,340,340));
  assert(!estimate.update(70,400,400)); // Unstable near values restart confirmation.
  assert(!estimate.update(70,460,460));
  assert(estimate.update(69,520,520));
  assert(!estimate.update(69,520,680)); // Stale sensor never retains a cup.
  CaptureEstimator close(0,20); assert(!close.update(10,0,0));
  CaptureEstimator wrap(35,75); uint32_t start=UINT32_MAX-90;
  assert(!wrap.update(50,start,start));
  assert(!wrap.update(50,start+60,start+60));
  assert(wrap.update(50,start+120,start+120));
  compassmath::Calibration cal; float h;
  assert(!cal.heading(1,1,1,0,h));
  for (int i=0;i<100;++i) {
    float rad=i*0.062831853f;
    cal.add(int16_t(300*cosf(rad)+100),int16_t(600*sinf(rad)-200));
  }
  assert(cal.valid());
  assert(cal.heading(400,-200,1,0,h) && fabsf(h)<1);
  assert(cal.heading(100,400,1,0,h) && fabsf(h-90)<1);
  assert(cal.heading(100,400,-1,0,h) && fabsf(h-270)<1);
  assert(cal.heading(400,-200,1,350,h) && fabsf(h-350)<1);
  assert(!cal.heading(100,-200,1,0,h)); // Zero field after calibration.
  assert(!cal.heading(32760,32760,1,0,h)); // Major magnetic distortion.
  assert(!cal.heading(100,400,0,0,h));
  float slope;
  assert(compassmath::slope(0,0,16384,slope) && fabsf(slope)<0.01f);
  assert(compassmath::slope(100,0,100,slope) && fabsf(slope-45)<0.01f);
  assert(!compassmath::slope(0,0,0,slope));
  assert(!compassmath::slope(std::numeric_limits<float>::quiet_NaN(),0,1,slope));
  puts("PASS: passive-arm capture estimate, cache/staleness/wrap, calibrated compass and slope");
}
