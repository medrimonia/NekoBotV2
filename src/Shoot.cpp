#include "Shoot.hpp"

#include <terminal.h>

#include "Interpolation.hpp"
#include "InverseKinematics.hpp"
#include "NekobotMotors.hpp"

// STATIC POSITION
//IK shoot
TERMINAL_PARAMETER_DOUBLE(shootRearX, "",   22.0);
TERMINAL_PARAMETER_DOUBLE(shootForeX, "",  -20.0);
TERMINAL_PARAMETER_DOUBLE(shootAvgZ , "",  140.0);
TERMINAL_PARAMETER_DOUBLE(shootPitch, "",  -10.0);

TERMINAL_PARAMETER_DOUBLE(shootLatAngle, "Shoot lat angle of legs", 12.0);


// SHOOT PREPARATION
TERMINAL_PARAMETER_DOUBLE(shootPreparationTime, "Time to prepare shoot", 0.2);
TERMINAL_PARAMETER_DOUBLE(shootPreparationDX, "delta X prepare shoot", -60.0);
TERMINAL_PARAMETER_DOUBLE(shootPreparationDZ, "delta Z prepare shoot", -40.0);

// SHOOT MOVE (From a prepared posture)
TERMINAL_PARAMETER_DOUBLE(shootTime, "Time to shoot", 0.2);
TERMINAL_PARAMETER_DOUBLE(shootFinalDX, "delta X at end of shoot",  160.0);
TERMINAL_PARAMETER_DOUBLE(shootFinalDZ, "delta Z at end of shoot", - 15.0);


void prepareShoot(double time, double elapsedTime, int shootingSide)
{
  double shootForeZ = computeForeZ(shootAvgZ, shootPitch);
  double shootRearZ = computeRearZ(shootAvgZ, shootPitch);
  double leftForeFootX = shootForeX;
  double leftForeFootZ = shootForeZ;
  double rightForeFootX = shootForeX;
  double rightForeFootZ = shootForeZ;
  if (shootingSide < 0) {
    leftForeFootX += shootPreparationDX;
    leftForeFootZ += shootPreparationDZ;
  }
  if (shootingSide > 0) {
    rightForeFootX += shootPreparationDX;
    rightForeFootZ += shootPreparationDZ;
  }
  leftRearLeg.setFromIK (time, shootRearX, shootRearZ);
  rightRearLeg.setFromIK(time, shootRearX, shootRearZ);
  leftForeLeg.setFromIK (time, leftForeFootX, leftForeFootZ);
  rightForeLeg.setFromIK(time, rightForeFootX, rightForeFootZ);
  setUniformLat(time, shootLatAngle);
}

void performShoot(double time, double elapsedTime, int shootingSide)
{
  double shootForeZ = computeForeZ(shootAvgZ, shootPitch);
  double shootRearZ = computeRearZ(shootAvgZ, shootPitch);
  double leftForeFootX = shootForeX;
  double leftForeFootZ = shootForeZ;
  double rightForeFootX = shootForeX;
  double rightForeFootZ = shootForeZ;
  double shootSrcX = shootForeX + shootPreparationDX;
  double shootSrcZ = shootForeZ + shootPreparationDZ;
  double shootTarX = shootForeX + shootFinalDX;
  double shootTarZ = shootForeZ + shootFinalDZ;
  double shootFootX = linearInterpolation(shootSrcX, shootTarX,
                                          elapsedTime, shootTime);
  double shootFootZ = linearInterpolation(shootSrcZ, shootTarZ,
                                          elapsedTime, shootTime);
  if (shootingSide < 0) {
    leftForeFootX = shootFootX;
    leftForeFootZ = shootFootZ;
  }
  if (shootingSide > 0) {
    rightForeFootX = shootFootX;
    rightForeFootZ = shootFootZ;
  }
  leftRearLeg.setFromIK (time, shootRearX, shootRearZ);
  rightRearLeg.setFromIK(time, shootRearX, shootRearZ);
  leftForeLeg.setFromIK (time, leftForeFootX, leftForeFootZ);
  rightForeLeg.setFromIK(time, rightForeFootX, rightForeFootZ);
}

int shoot(double time, double shootStart, int shootingSide)
{
  double elapsedTime = time - shootStart;
  if (elapsedTime < shootPreparationTime) {
    prepareShoot(time, elapsedTime, shootingSide);
    return 0;
  }
  elapsedTime -= shootPreparationTime;
  if (elapsedTime > shootTime) return 1;
  performShoot(time, elapsedTime, shootingSide);
  return 0;
}
