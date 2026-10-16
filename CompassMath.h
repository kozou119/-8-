#ifndef ROBOT_COMPASS_MATH_H
#define ROBOT_COMPASS_MATH_H
#include <math.h>
#include <stdint.h>
namespace compassmath {
struct Calibration {
  int16_t minX=32767, maxX=-32767, minY=32767, maxY=-32767;
  unsigned samples=0;
  void add(int16_t x, int16_t y) {
    if (x<minX) minX=x;
    if (x>maxX) maxX=x;
    if (y<minY) minY=y;
    if (y>maxY) maxY=y;
    if (samples<65535) ++samples;
  }
  bool valid() const {
    return samples>=50 && int32_t(maxX)-minX>=100 && int32_t(maxY)-minY>=100;
  }
  bool heading(int16_t x, int16_t y, float sign, float offset, float &result) const {
    if (!valid() || (sign!=1 && sign!=-1) || !isfinite(offset)) return false;
    float nx=(2.0f*x-minX-maxX)/(int32_t(maxX)-minX);
    float ny=(2.0f*y-minY-maxY)/(int32_t(maxY)-minY);
    float radius=nx*nx+ny*ny;
    if (radius<0.10f || radius>4.0f) return false; // Bad field/distortion, not an angle.
    result=fmodf(sign*atan2f(ny,nx)*57.2957795f+offset,360.0f);
    if (result<0) result+=360;
    return isfinite(result);
  }
};
inline bool slope(float x, float y, float z, float &degrees) {
  float norm=x*x+y*y+z*z;
  if (!isfinite(norm) || norm<1) return false;
  degrees=atan2f(sqrtf(x*x+y*y),fabsf(z))*57.2957795f;
  return isfinite(degrees);
}
}
#endif
