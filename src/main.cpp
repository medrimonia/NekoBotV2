#include <cstdlib>
#include <wirish/wirish.h>
#include <servos.h>
#include <terminal.h>
#include <main.h>
#include <dxl.h>

#include "Interpolation.hpp"
#include "RearLeg.hpp"

TERMINAL_PARAMETER_DOUBLE(t, "Temps", 0.0);

TERMINAL_PARAMETER_DOUBLE(period, "Period length", 5.0);

TERMINAL_PARAMETER_BOOL(freeMove, "Disable", true);

// Init Pos
TERMINAL_PARAMETER_DOUBLE(initRearAngle1, "Init angle of rear 1",   0.0);
TERMINAL_PARAMETER_DOUBLE(initRearAngle2, "Init angle of rear 2",  45.0);
TERMINAL_PARAMETER_DOUBLE(initRearAngle3, "Init angle of rear 3", -80.0);
TERMINAL_PARAMETER_DOUBLE(initRearAngle4, "Init angle of rear 4",  61.0);
TERMINAL_PARAMETER_DOUBLE(initForeAngle1, "Init angle of fore 1",   0.0);
TERMINAL_PARAMETER_DOUBLE(initForeAngle2, "Init angle of fore 2", -55.0);
TERMINAL_PARAMETER_DOUBLE(initForeAngle3, "Init angle of fore 3",  70.0);

TERMINAL_PARAMETER_DOUBLE(bottomRearAngle1, "Bottom angle of rear 1",   0.0);
TERMINAL_PARAMETER_DOUBLE(bottomRearAngle2, "Bottom angle of rear 2",  45.0);
TERMINAL_PARAMETER_DOUBLE(bottomRearAngle3, "Bottom angle of rear 3", -110.0);
TERMINAL_PARAMETER_DOUBLE(bottomRearAngle4, "Bottom angle of rear 4",  80.0);

RearLeg leftRearLeg ( 2, false);
RearLeg rightRearLeg(12, true);

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
  dxl_set_zero(12, 0.00);
  dxl_set_min_max(12, -57.13, 91.41);
  dxl_set_zero(13, 0.00);
  dxl_set_min_max(13, -106.64, 86.43);
  dxl_set_zero(14, 0.00);
  dxl_set_min_max(14, -27.54, 131.25);
  dxl_set_zero(15, 0.00);
  dxl_set_min_max(15, -103.12, 109.28);
}

void getInitRearLeg(double * dst)
{
  dst[0] = initRearAngle1;
  dst[1] = initRearAngle2;
  dst[2] = initRearAngle3;
  dst[3] = initRearAngle4;
}

void getBottomRearLeg(double * dst)
{
  dst[0] = initRearAngle1;
  dst[1] = initRearAngle2;
  dst[2] = initRearAngle3;
  dst[3] = initRearAngle4;
}

/**
 * Foncton appellée à 50hz, c'est ici que vous pouvez mettre
 * à jour les angles moteurs etc.
 */
void tick()
{
  t += 0.02; // 20ms
  if (freeMove) {
    for (int i = 2; i <= 5; i++) {
      leftRearLeg.disable();
      rightRearLeg.disable();
    }
    return;    
  }
  for (int i = 2; i <= 5; i++) {
    leftRearLeg.enable();
    rightRearLeg.enable();
  }
  double src[4], tar[4], curr[4];
  getInitRearLeg(src);
  getBottomRearLeg(tar);
  linearInterpolation(curr, src, tar, 4, fmod(t, period), period);
  leftRearLeg.setAngles(src);
  rightRearLeg.setAngles(src);
}

/**
 * Si vous souhaitez écrire ici du code, cette fonction sera
 * apellée en boucle
 */
void loop()
{
}
