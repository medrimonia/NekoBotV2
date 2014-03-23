#include "Walk.hpp"

#include <function.h>
#include <terminal.h>

#include "NekobotMotors.hpp"

// Move Parameters
TERMINAL_PARAMETER_DOUBLE(forwardOrder , "ratio of forward Order",  0.0);
TERMINAL_PARAMETER_DOUBLE(rotationOrder, "ratio of rotation Order",  0.0);

TERMINAL_PARAMETER_DOUBLE(walkPeriod, "Duration of a full step"       ,  1.0);
TERMINAL_PARAMETER_DOUBLE(stepHeight, "Height of a step"              , 25.0);
TERMINAL_PARAMETER_DOUBLE(stepLength, "Length of a step"              , 80.0);
TERMINAL_PARAMETER_DOUBLE(stepLatAmp, "Amplitude of a step (rotation)",  5.0);

TERMINAL_PARAMETER_DOUBLE(walkLatOffset, "Walking lateral offset"  ,-15.0);

Function walkingX;
Function walkingZ;

#define STEP_DURATION 0.2
void initWalking(){
  // X in the middle, foot in the air
  walkingX.addPoint(0, 0);
  walkingZ.addPoint(0, 1);
  // X forward, foot on ground
  walkingX.addPoint(STEP_DURATION / 2, 1);
  walkingZ.addPoint(STEP_DURATION / 2, 0);
  // X backward, foot on ground
  walkingX.addPoint(1 - STEP_DURATION / 2, -1);
  walkingZ.addPoint(1 - STEP_DURATION / 2, 0);
  // Back to start, in the middle, foot in the air
  walkingX.addPoint(1, 0);
  walkingZ.addPoint(1, 1);
}

//TODO if not using leftStep and rightStep, remove amplitude
double movePosX(double staticX, double time, double amplitude)
{
  return staticX + walkingX.getMod(time) * amplitude / 2.0;
}

double movePosZ(double staticZ, double time, double amplitude)
{
  return staticZ + walkingZ.getMod(time) * amplitude / 2.0;
}

double movePosLat(double staticLat, double time, double amplitude)
{
  double pos = staticLat;
  pos += walkingZ.getMod(time) * walkLatOffset;// step offset
  pos += walkingX.getMod(time) * amplitude / 2.0;// rotation
  return pos;
}

void move(double t,
          double staticRearX, double staticRearZ,
          double staticForeX, double staticForeZ,
          double staticLatAngle)
{
  double walkingTime = fmod(t, walkPeriod) / walkPeriod;
  double time1 = walkingTime;
  double time2 = walkingTime + 0.25;
  double time3 = walkingTime + 0.5;
  double time4 = walkingTime + 0.75;
  double leftRearX, rightRearX, leftForeX, rightForeX;
  double leftRearZ, rightRearZ, leftForeZ, rightForeZ;
  double leftRearLat, rightRearLat, leftForeLat, rightForeLat;
  double leftStep, rightStep, latStep;
  leftStep  = stepLength * forwardOrder;
  rightStep = stepLength * forwardOrder;
  latStep   = stepLatAmp * rotationOrder;
  // Computing positions
  leftRearX    = movePosX  (staticRearX   , time1, leftStep  );
  leftRearZ    = movePosZ  (staticRearZ   , time1, stepHeight);
  leftRearLat  = movePosLat(staticLatAngle, time1, latStep   );
  rightRearX   = movePosX  (staticRearX   , time3, rightStep );
  rightRearZ   = movePosZ  (staticRearZ   , time3, stepHeight);
  rightRearLat = movePosLat(staticLatAngle, time3, latStep   );
  leftForeX    = movePosX  (staticForeX   , time4, leftStep  );
  leftForeZ    = movePosZ  (staticForeZ   , time4, stepHeight);
  leftForeLat  = movePosLat(staticLatAngle, time4, latStep   );
  rightForeX   = movePosX  (staticForeX   , time2, rightStep );
  rightForeZ   = movePosZ  (staticForeZ   , time2, stepHeight);
  rightForeLat = movePosLat(staticLatAngle, time2, latStep   );
  // Applying positions
  leftRearLeg.setFromIK(leftRearX, leftRearZ);
  rightRearLeg.setFromIK(rightRearX, rightRearZ);
  leftForeLeg.setFromIK(leftForeX, leftForeZ);
  rightForeLeg.setFromIK(rightForeX, rightForeZ);
  leftRearLeg.setLatAngle(leftRearLat);
  rightRearLeg.setLatAngle(rightRearLat);
  leftForeLeg.setLatAngle(leftForeLat);
  rightForeLeg.setLatAngle(rightForeLat);
}
