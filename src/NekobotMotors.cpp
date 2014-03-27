#include <cmath>

#include "NekobotMotors.hpp"

#include <dxl.h>

#include "Interpolation.hpp"
#include "InverseKinematics.hpp"

RearLeg rightRearLeg ( 2, false);
RearLeg leftRearLeg(12, true);
ForeLeg rightForeLeg ( 6, false);
ForeLeg leftForeLeg(16, true);

void startSmoothing(double time, double smoothingLength)
{
  rightRearLeg.startSmoothing(time, smoothingLength);
  leftRearLeg.startSmoothing(time, smoothingLength);
  rightForeLeg.startSmoothing(time, smoothingLength);
  leftForeLeg.startSmoothing(time, smoothingLength);
}

void smoothSet(int id, double src, double tar,
               double time, double tStart, double smoothLength)
{
  double dt = time - tStart;
  if (dt >= smoothLength) {
    dxl_set_position(id, tar);
  }
  else {
    double smoothedTar =  linearInterpolation(src, tar, dt, smoothLength);
    dxl_set_position(id, smoothedTar);
  }
}
/*
void setPosture(double time, double rearX, double rearZ, double foreX,
                double foreZ,
                double latAngle)
{
  leftRearLeg.setFromIK   (time, rearX, rearZ );
  rightRearLeg.setFromIK  (time, rearX, rearZ);
  leftForeLeg.setFromIK   (time, foreX, foreZ );
  rightForeLeg.setFromIK  (time, foreX, foreZ);
  leftRearLeg.setLatAngle (time, latAngle);
  rightRearLeg.setLatAngle(time, latAngle);
  leftForeLeg.setLatAngle (time, latAngle);
  rightForeLeg.setLatAngle(time, latAngle);
}
*/

void setAllFromIK(double time, double foreX, double rearX, double avgZ,
                  double robotPitch)
{
  double foreZ = computeForeZ(avgZ, robotPitch);
  double rearZ = computeRearZ(avgZ, robotPitch);
  leftRearLeg.setFromIK (time, rearX, rearZ, robotPitch);
  rightRearLeg.setFromIK(time, rearX, rearZ, robotPitch);
  leftForeLeg.setFromIK (time, foreX, foreZ, robotPitch);
  rightForeLeg.setFromIK(time, foreX, foreZ, robotPitch);
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

void setUniformLat(double time, double latAngle)
{
  leftRearLeg.setLatAngle (time, latAngle);
  rightRearLeg.setLatAngle(time, latAngle);
  leftForeLeg.setLatAngle (time, latAngle);
  rightForeLeg.setLatAngle(time, latAngle);
}

void setFromAngles(double time, double back2humerus, double humerus2radius,
                   double back2femur, double femur2tibia, double tibia2foot)
{
  leftForeLeg.setHumerusAngle (time, back2humerus);
  leftForeLeg.setRadiusAngle  (time, humerus2radius);
  rightForeLeg.setHumerusAngle(time, back2humerus);
  rightForeLeg.setRadiusAngle (time, humerus2radius);
  leftRearLeg.setFemurAngle   (time, back2femur);
  leftRearLeg.setTibiaAngle   (time, femur2tibia);
  leftRearLeg.setFootAngle    (time, tibia2foot);
  rightRearLeg.setFemurAngle  (time, back2femur);
  rightRearLeg.setTibiaAngle  (time, femur2tibia);
  rightRearLeg.setFootAngle   (time, tibia2foot);
}
