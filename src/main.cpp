#include <cstdlib>
#include <wirish/wirish.h>
#include <servos.h>
#include <terminal.h>
#include <main.h>
#include <dxl.h>

#include "Interpolation.hpp"
#include "ForeLeg.hpp"
#include "RearLeg.hpp"

TERMINAL_PARAMETER_DOUBLE(t, "Temps", 0.0);

TERMINAL_PARAMETER_DOUBLE(period, "Period length", 5.0);

TERMINAL_PARAMETER_BOOL(freeMove, "Disable", true);

//rear IK init
TERMINAL_PARAMETER_DOUBLE(initRearX, "Init X of rear foots",   22.0);
TERMINAL_PARAMETER_DOUBLE(initRearZ, "Init Z of rear foots",  190.0);
//fore IK default
TERMINAL_PARAMETER_DOUBLE(initForeX, "Init X of rear foots",    0.0);
TERMINAL_PARAMETER_DOUBLE(initForeZ, "Init Z of rear foots",  190.0);


RearLeg rightRearLeg ( 2, false);
RearLeg leftRearLeg(12, true);
ForeLeg rightForeLeg ( 6, false);
ForeLeg leftForeLeg(16, true);

/**
 * Vous pouvez écrire du code qui sera exécuté à 
 * l'initialisation ici
 */
void setup()
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

/**
 * Foncton appellée à 50hz, c'est ici que vous pouvez mettre
 * à jour les angles moteurs etc.
 */
void tick()
{
  t += 0.02; // 20ms
  if (freeMove) {
    leftRearLeg.disable();
    rightRearLeg.disable();
    leftForeLeg.disable();
    dxl_disable(6);
    dxl_disable(8);
    //rightForeLeg.disable();
    return;    
  }
  // Enabling motors
  leftRearLeg.enable();
  rightRearLeg.enable();
  leftForeLeg.enable();
  dxl_enable(6);
  //dxl_enable(7);
  dxl_enable(8);
  //rightForeLeg.enable();
  // Setting Lat Angles
  leftRearLeg.setLatAngle(0);
  rightRearLeg.setLatAngle(0);
  leftForeLeg.setLatAngle(0);
  // Setting Legs angles from IK
  leftRearLeg.setFromIK(initRearX, initRearZ);
  rightRearLeg.setFromIK(initRearX, initRearZ);
  leftForeLeg.setFromIK(initForeX, initForeZ);
  //rightForeLeg.setFromIK(initForeX, initForeZ);
  //double zeros[3] = {0, 0, 0};
  //leftForeLeg.setAngles(zeros);
  //rightForeLeg.setAngles(zeros);
}

/**
 * Si vous souhaitez écrire ici du code, cette fonction sera
 * apellée en boucle
 */
void loop()
{
}
