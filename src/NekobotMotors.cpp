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
  dxl_set_zero(2, 0.00);
  dxl_set_min_max(2, -77.64, 53.61);
  dxl_set_zero(3, 0.00);
  dxl_set_min_max(3, -88.77, 111.62);
  dxl_set_zero(4, 0.00);
  dxl_set_min_max(4, -127.44, 29.00);
  dxl_set_zero(5, 0.00);
  dxl_set_min_max(5, -107.52, 107.52);
  dxl_set_zero(6, 0.00);
  dxl_set_min_max(6, -100.20, 32.52);
  dxl_set_zero(7, 0.00);
  dxl_set_min_max(7, -139.45, 25.20);
  dxl_set_zero(8, 0.00);
  dxl_set_min_max(8, -101.66, 114.26);
  dxl_set_zero(12, 0.00);
  dxl_set_min_max(12, -57.13, 91.41);
  dxl_set_zero(13, 0.00);
  dxl_set_min_max(13, -106.64, 86.43);
  dxl_set_zero(14, 0.00);
  dxl_set_min_max(14, -27.54, 131.25);
  dxl_set_zero(15, 0.00);
  dxl_set_min_max(15, -103.12, 109.28);
  dxl_set_zero(16, 0.00);
  dxl_set_min_max(16, -46.88, 91.41);
  dxl_set_zero(17, 0.00);
  dxl_set_min_max(17, -113.38, 102.54);
  dxl_set_zero(18, 0.00);
  dxl_set_min_max(18, -113.38, 114.84);
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
