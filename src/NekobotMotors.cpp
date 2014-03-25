#include "NekobotMotors.hpp"

#include <dxl.h>

RearLeg rightRearLeg ( 2, false);
RearLeg leftRearLeg(12, true);
ForeLeg rightForeLeg ( 6, false);
ForeLeg leftForeLeg(16, true);

void setPosture(double rearX, double rearZ, double foreX, double foreZ,
                double latAngle)
{
  leftRearLeg.setFromIK (rearX, rearZ );
  rightRearLeg.setFromIK(rearX, rearZ);
  leftForeLeg.setFromIK (foreX, foreZ );
  rightForeLeg.setFromIK(foreX, foreZ);
  leftRearLeg.setLatAngle (latAngle);
  rightRearLeg.setLatAngle(latAngle);
  leftForeLeg.setLatAngle (latAngle);
  rightForeLeg.setLatAngle(latAngle);
}

void initMotors()
{
  dxl_init();
  leftRearLeg.init();
  rightRearLeg.init();
  leftForeLeg.init();
  rightForeLeg.init();
  // If hacks are to be placed, it is know
}

void motSetMinMax(int id, double min, double max, bool inverted)
{
  double trueMin, trueMax;
  if (inverted) {
    trueMin = -max;
    trueMax = -min;
  }
  else {
    trueMin = min;
    trueMax = max;
  }
  dxl_set_min_max(id, trueMin, trueMax);
}

void enableMotors()
{
  leftRearLeg.enable();
  rightRearLeg.enable();
  leftForeLeg.enable();
  rightForeLeg.enable();
}

void disableMotors()
{
  leftRearLeg.disable();
  rightRearLeg.disable();
  leftForeLeg.disable();
  rightForeLeg.disable();
}

void setUniformLat(double latAngle)
{
  leftRearLeg.setLatAngle (latAngle);
  rightRearLeg.setLatAngle(latAngle);
  leftForeLeg.setLatAngle (latAngle);
  rightForeLeg.setLatAngle(latAngle);
}
