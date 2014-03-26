#include "RearLeg.hpp"

#include <dxl.h>

#include "InverseKinematics.hpp"
#include "NekobotMotors.hpp"

void RearLeg::init()
{
  //LAT motor disabled
  dxl_set_zero(startIndex + 1, 0.0);
  motSetMinMax(startIndex + 1,
               BACK2FEMUR_MIN_ANGLE, BACK2FEMUR_MAX_ANGLE, inv);
  dxl_set_zero(startIndex + 2, 0.0);
  motSetMinMax(startIndex + 2,
               FEMUR2TIBIA_MIN_ANGLE, FEMUR2TIBIA_MAX_ANGLE, inv);
  dxl_set_zero(startIndex + 3, 0.0);
  motSetMinMax(startIndex + 3,
               TIBIA2FOOT_MIN_ANGLE, TIBIA2FOOT_MAX_ANGLE, inv);
}

void RearLeg::setLatAngle(double time, double a) {
  a = inv ? a : -a;
  smoothSet(startIndex, oldLat, a, time, smoothStart, smoothingTime);
}

void RearLeg::setFemurAngle(double time, double a) {
  a = inv ? -a : a;
  smoothSet(startIndex + 1, oldFemur, a, time, smoothStart, smoothingTime);
}

void RearLeg::setTibiaAngle(double time, double a) {
  a = inv ? -a : a;
  smoothSet(startIndex + 2, oldTibia, a, time, smoothStart, smoothingTime);
}

void RearLeg::setFootAngle(double time, double a) {
  a = inv ? -a : a;
  smoothSet(startIndex + 3, oldFoot, a, time, smoothStart, smoothingTime);
}

double RearLeg::getLatAngle() {
  double mult =  inv ? 1 : -1;
  return mult * dxl_get_position(startIndex);
}

double RearLeg::getFemurAngle() {
  double mult =  inv ? -1 : 1;
  return mult * dxl_get_position(startIndex + 1);
}

double RearLeg::getTibiaAngle() {
  double mult =  inv ? -1 : 1;
  return mult * dxl_get_position(startIndex + 2);
}

double RearLeg::getFootAngle() {
  double mult =  inv ? -1 : 1;
  return mult * dxl_get_position(startIndex + 3);
}

void RearLeg::setAngles(double time, double * angles)
{
  setLatAngle  (time, angles[0]);
  setFemurAngle(time, angles[1]);
  setTibiaAngle(time, angles[2]);
  setFootAngle (time, angles[3]);
}

void RearLeg::setFromIK(double time, double x, double z, double robotPitch)
{
  double previousValues[3];
  previousValues[0] = getFemurAngle();
  previousValues[1] = getTibiaAngle();
  previousValues[2] = getFootAngle();
  double wishedValues[3];
  if (computeRearLegIK(wishedValues, previousValues, x, z, robotPitch) == -1) {
    // If IK failed, do not set angles
    return;
  }
  setFemurAngle(time, wishedValues[0]);
  setTibiaAngle(time, wishedValues[1]);
  setFootAngle (time, wishedValues[2]);
}

void RearLeg::enable()
{
  for (int i = startIndex; i < startIndex + 4; i++) {
    dxl_enable(i);
  }
}

void RearLeg::disable()
{
  for (int i = startIndex; i < startIndex + 4; i++) {
    dxl_disable(i);
  }
}

void RearLeg::startSmoothing(double time, double smoothLength)
{
  smoothStart = time;
  smoothingTime = smoothLength;
  oldLat   = dxl_get_position(startIndex    );
  oldFemur = dxl_get_position(startIndex + 1);
  oldTibia = dxl_get_position(startIndex + 2);
  oldFoot  = dxl_get_position(startIndex + 3);
}
