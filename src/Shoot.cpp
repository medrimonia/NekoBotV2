#include "Shoot.hpp"

#include <terminal.h>

#include "Interpolation.hpp"
#include "NekobotMotors.hpp"

// STATIC POSITION
//rear IK shoot
TERMINAL_PARAMETER_DOUBLE(shootRearX, "Shoot X of rear foots",   22.0);
TERMINAL_PARAMETER_DOUBLE(shootRearZ, "Shoot Z of rear foots",  170.0);
//fore IK shoot
TERMINAL_PARAMETER_DOUBLE(shootForeX, "Shoot X of fore foots",  -20.0);
TERMINAL_PARAMETER_DOUBLE(shootForeZ, "Shoot Z of fore foots",  170.0);

TERMINAL_PARAMETER_DOUBLE(shootLatAngle, "Shoot lat angle of legs", 12.0);


// SHOOT PREPARATION
TERMINAL_PARAMETER_DOUBLE(shootPreparationTime, "Time to prepare shoot", 0.2);
TERMINAL_PARAMETER_DOUBLE(shootPreparationDX, "delta X prepare shoot", -60.0);
TERMINAL_PARAMETER_DOUBLE(shootPreparationDZ, "delta Z prepare shoot", -40.0);

// SHOOT MOVE (From a prepared posture)
TERMINAL_PARAMETER_DOUBLE(shootTime, "Time to shoot", 0.2);
TERMINAL_PARAMETER_DOUBLE(shootFinalDX, "delta X at end of shoot",  160.0);
TERMINAL_PARAMETER_DOUBLE(shootFinalDZ, "delta Z at end of shoot", - 15.0);


void prepareShoot(double elapsedTime, int shootingSide)
{
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
  leftRearLeg.setFromIK(shootRearX, shootRearZ);
  rightRearLeg.setFromIK(shootRearX, shootRearZ);
  leftForeLeg.setFromIK(leftForeFootX, leftForeFootZ);
  rightForeLeg.setFromIK(rightForeFootX, rightForeFootZ);
  setUniformLat(shootLatAngle);
}

void performShoot(double elapsedTime, int shootingSide)
{
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
  leftRearLeg.setFromIK(shootRearX, shootRearZ);
  rightRearLeg.setFromIK(shootRearX, shootRearZ);
  leftForeLeg.setFromIK(leftForeFootX, leftForeFootZ);
  rightForeLeg.setFromIK(rightForeFootX, rightForeFootZ);
  setUniformLat(shootLatAngle);
}

int shoot(double time, double shootStart, int shootingSide)
{
  double elapsedTime = time - shootStart;
  if (elapsedTime < shootPreparationTime) {
    prepareShoot(elapsedTime, shootingSide);
    return 0;
  }
  elapsedTime -= shootPreparationTime;
  if (elapsedTime > shootTime) return 1;
  performShoot(elapsedTime, shootingSide);
  return 0;
}
