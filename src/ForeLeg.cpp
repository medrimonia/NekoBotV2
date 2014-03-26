#include "ForeLeg.hpp"

#include <dxl.h>

#include "InverseKinematics.hpp"
#include "NekobotMotors.hpp"

void ForeLeg::init()
{
  //LAT motor disabled
  dxl_set_zero(startIndex + 1, 0.0);
  motSetMinMax(startIndex + 1,
               BACK2HUMERUS_MIN_ANGLE, BACK2HUMERUS_MAX_ANGLE, inv);
  dxl_set_zero(startIndex + 2, 0.0);
  motSetMinMax(startIndex + 2,
               HUMERUS2RADIUS_MIN_ANGLE, HUMERUS2RADIUS_MAX_ANGLE, inv);
}

void ForeLeg::setLatAngle(double time, double a) {
  a = inv ? a : -a;
  smoothSet(startIndex, oldLat, a, time, smoothStart, smoothingTime);
}

void ForeLeg::setHumerusAngle(double time, double a) {
  a = inv ? -a : a;
  smoothSet(startIndex + 1, oldHumerus, a, time, smoothStart, smoothingTime);
}

void ForeLeg::setRadiusAngle(double time, double a) {
  a = inv ? -a : a;
  smoothSet(startIndex + 2, oldRadius, a, time, smoothStart, smoothingTime);
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

void ForeLeg::setAngles(double time, double * angles)
{
  setLatAngle    (time, angles[0]);
  setHumerusAngle(time, angles[1]);
  setRadiusAngle (time, angles[2]);
}

void ForeLeg::setFromIK(double time, double x, double z, double robotPitch)
{
  double previousValues[2];
  previousValues[0] = getHumerusAngle();
  previousValues[1] = getRadiusAngle();
  double wishedValues[2];
  if (computeForeLegIK(wishedValues, previousValues, x, z, robotPitch) == -1) {
    // If IK failed, do not set angles
    return;
  }
  setHumerusAngle(time, wishedValues[0]);
  setRadiusAngle (time, wishedValues[1]);
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

void ForeLeg::startSmoothing(double time, double smoothLength)
{
  smoothStart = time;
  smoothingTime = smoothLength;
  oldLat     = dxl_get_position(startIndex    );
  oldHumerus = dxl_get_position(startIndex + 1);
  oldRadius  = dxl_get_position(startIndex + 2);
}
