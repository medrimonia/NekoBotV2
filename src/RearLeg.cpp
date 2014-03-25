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

void RearLeg::setLatAngle(double a) {
  a = inv ? a : -a;
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

void RearLeg::setAngles(double * angles)
{
  setLatAngle(angles[0]);
  setFemurAngle(angles[1]);
  setTibiaAngle(angles[2]);
  setFootAngle(angles[3]);
}

void RearLeg::setFromIK(double x, double z)
{
  double previousValues[3];
  previousValues[0] = getFemurAngle();
  previousValues[1] = getTibiaAngle();
  previousValues[2] = getFootAngle();
  double wishedValues[3];
  if (computeRearLegIK(wishedValues, previousValues, x, z) == -1) {
    // If IK failed, do not set angles
    return;
  }
  setFemurAngle(wishedValues[0]);
  setTibiaAngle(wishedValues[1]);
  setFootAngle (wishedValues[2]);
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
