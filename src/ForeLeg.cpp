#include "ForeLeg.hpp"

#include <dxl.h>

#include "InverseKinematics.hpp"

void ForeLeg::setLatAngle(double a) {
  a = inv ? a : -a;
  dxl_set_position(startIndex, a);
}

void ForeLeg::setHumerusAngle(double a) {
  a = inv ? -a : a;
  dxl_set_position(startIndex + 1, a);
}

void ForeLeg::setRadiusAngle(double a) {
  a = inv ? -a : a;
  dxl_set_position(startIndex + 2, a);
}

double ForeLeg::getLatAngle() {
  double mult =  inv ? 1 : -1;
  return mult * dxl_get_position(startIndex);
}

double ForeLeg::getHumerusAngle() {
  double mult =  inv ? -1 : 1;
  return mult * dxl_get_position(startIndex + 1);
}

double ForeLeg::getRadiusAngle() {
  double mult =  inv ? -1 : 1;
  return mult * dxl_get_position(startIndex + 2);
}

void ForeLeg::setAngles(double * angles)
{
  setLatAngle(angles[0]);
  setHumerusAngle(angles[1]);
  setRadiusAngle(angles[2]);
}

void ForeLeg::setFromIK(double x, double z)
{
  double previousValues[2];
  previousValues[0] = getHumerusAngle();
  previousValues[1] = getRadiusAngle();
  double wishedValues[2];
  if (computeForeLegIK(wishedValues, previousValues, x, z) == -1) {
    // If IK failed, do not set angles
    return;
  }
  setHumerusAngle(wishedValues[0]);
  setRadiusAngle(wishedValues[1]);
}

void ForeLeg::enable()
{
  for (int i = startIndex; i < startIndex + 3; i++) {
    dxl_enable(i);
  }
}

void ForeLeg::disable()
{
  for (int i = startIndex; i < startIndex + 3; i++) {
    dxl_disable(i);
  }
}
