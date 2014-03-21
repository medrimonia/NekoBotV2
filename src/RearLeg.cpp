#include "RearLeg.hpp"

#include <dxl.h>

void RearLeg::setLatAngle(double a) {
  dxl_set_position(startIndex, a);
}

void RearLeg::setFemurAngle(double a) {
  a = inv ? -a : a;
  dxl_set_position(startIndex + 1, a);
}

void RearLeg::setTibiaAngle(double a) {
  a = inv ? -a : a;
  dxl_set_position(startIndex + 2, a);
}

void RearLeg::setFootAngle(double a) {
  a = inv ? -a : a;
  dxl_set_position(startIndex + 3, a);
}

void RearLeg::setAngles(double * angles)
{
  setLatAngle(angles[0]);
  setFemurAngle(angles[1]);
  setTibiaAngle(angles[2]);
  setFootAngle(angles[3]);
}

void RearLeg::enable()
{
  for (int i = startIndex; i <= startIndex + 4; i++) {
    dxl_enable(i);
  }
}

void RearLeg::disable()
{
  for (int i = startIndex; i <= startIndex + 4; i++) {
    dxl_disable(i);
  }
}
