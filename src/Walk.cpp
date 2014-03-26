#include "Walk.hpp"

#include <terminal.h>

#include "Interpolation.hpp"
#include "InverseKinematics.hpp"
#include "NekobotMotors.hpp"

// Move Parameters
TERMINAL_PARAMETER_DOUBLE(forwardOrder , "ratio of forward Order",  0.0);
TERMINAL_PARAMETER_DOUBLE(rotationOrder, "ratio of rotation Order",  0.0);

// Duration parameters
TERMINAL_PARAMETER_DOUBLE(walkPeriod,   "Duration of a full step"   ,  2.0);
TERMINAL_PARAMETER_DOUBLE(stepDuration, "Foot out of ground (ratio)",  0.2);

// Amplitude parameters
TERMINAL_PARAMETER_DOUBLE(stepHeight, "Height of a step"              , 80.0);
TERMINAL_PARAMETER_DOUBLE(stepLength, "Length of a step"              ,100.0);
TERMINAL_PARAMETER_DOUBLE(stepLatAmp, "Amplitude of a step (rotation)",  5.0);

TERMINAL_PARAMETER_DOUBLE(walkLatOffset, "Walking lateral offset"  ,-10.0);

TERMINAL_PARAMETER_INT(walkType, "Way of walking (2 legs / 1 legs)", 1);

#define WALK_ONE_BY_ONE 0
#define WALK_TWO_BY_TWO 1

// For all the Pos function, time is a value in [0,1]

//TODO if not using leftStep and rightStep, remove amplitude
double movePosX(double staticX, double time, double amplitude)
{
  double xRatio = 0;
  bool inverted = false;
  // Antisymetric function
  if (time > 0.5) {
    inverted = true;
    time = 1 - time;
  }
  if (time < stepDuration / 2) {
    xRatio = sin(linearInterpolation(0, M_PI / 2, time, stepDuration / 2));
  }
  else {
    xRatio = linearInterpolation(1, 0,
                                 time - stepDuration / 2,
                                 0.5 - stepDuration / 2);
  }
  xRatio = inverted ? -xRatio : xRatio;
  return staticX + xRatio * amplitude / 2.0;
}

double movePosZ(double staticZ, double time, double amplitude)
{
  // Symetric function
  if (time > 0.5) {
    time = 1 - time;
  }
  double zRatio = 0.0;
  if (time < stepDuration / 2) {
    zRatio = cos(time / (stepDuration / 2) * M_PI / 2);
  }
  return staticZ - zRatio * amplitude / 2.0;
}

void move(double t,
          double staticForeX, double staticRearX,
          double staticAvgZ, double staticPitch)
{
  double walkingTime = fmod(t, walkPeriod) / walkPeriod;
  double staticRearZ = computeRearZ(staticAvgZ, staticPitch);
  double staticForeZ = computeForeZ(staticAvgZ, staticPitch);
  double time1, time2(0), time3, time4(0);
  time1 = walkingTime;
  time3 = fmod(walkingTime + 0.5, 1.0);
  switch(walkType) {
  case WALK_ONE_BY_ONE: {
    time2 = fmod(walkingTime + 0.25, 1.0);
    time4 = fmod(walkingTime + 0.75, 1.0);
    break;
  }
  case WALK_TWO_BY_TWO: {
    time2 = time1;
    time4 = time3;
    break;
  }
  }
  double leftRearX, rightRearX, leftForeX, rightForeX;
  double leftRearZ, rightRearZ, leftForeZ, rightForeZ;
  double leftStep, rightStep;
  leftStep  = stepLength * forwardOrder + stepLength * rotationOrder;
  rightStep = stepLength * forwardOrder - stepLength * rotationOrder;
  double totalOrder = abs(forwardOrder) + abs(rotationOrder);
  if (totalOrder > 1.0) {
    leftStep /= totalOrder;
    rightStep /= totalOrder;
  }
  // Computing positions
  leftRearX    = movePosX  (staticRearX   , time1, leftStep  );
  leftRearZ    = movePosZ  (staticRearZ   , time1, stepHeight);
  rightRearX   = movePosX  (staticRearX   , time3, rightStep );
  rightRearZ   = movePosZ  (staticRearZ   , time3, stepHeight);
  leftForeX    = movePosX  (staticForeX   , time4, leftStep  );
  leftForeZ    = movePosZ  (staticForeZ   , time4, stepHeight);
  rightForeX   = movePosX  (staticForeX   , time2, rightStep );
  rightForeZ   = movePosZ  (staticForeZ   , time2, stepHeight);
  // Applying positions
  leftRearLeg.setFromIK (t, leftRearX , leftRearZ , staticPitch);
  rightRearLeg.setFromIK(t, rightRearX, rightRearZ, staticPitch);
  leftForeLeg.setFromIK (t, leftForeX , leftForeZ , staticPitch);
  rightForeLeg.setFromIK(t, rightForeX, rightForeZ, staticPitch);
}
