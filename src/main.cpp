#include <cstdlib>
#include <wirish/wirish.h>
#include <servos.h>
#include <terminal.h>
#include <main.h>
#include <dxl.h>
#include <function.h>

#include "ATR.hpp"
#include "Interpolation.hpp"
#include "NekobotMotors.hpp"
#include "Shoot.hpp"
#include "Walk.hpp"

TERMINAL_PARAMETER_DOUBLE(t, "Temps", 0.0);

TERMINAL_PARAMETER_DOUBLE(period, "Period length", 5.0);

//rear IK init
TERMINAL_PARAMETER_DOUBLE(initRearX, "Init X of rear foots",    0.0);
TERMINAL_PARAMETER_DOUBLE(initRearZ, "Init Z of rear foots",  150.0);
//fore IK default
TERMINAL_PARAMETER_DOUBLE(initForeX, "Init X of fore foots",  -20.0);
TERMINAL_PARAMETER_DOUBLE(initForeZ, "Init Z of fore foots",  150.0);
//Using Pitch + avgZ
TERMINAL_PARAMETER_DOUBLE(initPitch,"Init pitch of robot",  0.0);
TERMINAL_PARAMETER_DOUBLE(initAvgZ, "Init avg Z",  150.0);


TERMINAL_PARAMETER_DOUBLE(initLatAngle, "Init lat angle of legs", 10.0);
TERMINAL_PARAMETER_DOUBLE(oscPeriod, "Init lat angle of legs", 1.0);
TERMINAL_PARAMETER_DOUBLE(oscAmp, "Init lat angle of legs", 0.0);

// States handling
#define FREE_MOVE_STATE     0
#define INIT_STATE          1
#define WALK_STATE          2
#define SHOOT_STATE         3
#define ATR_STATE           4
#define BACK_FROM_ATR_STATE 5

TERMINAL_PARAMETER_INT(state    , "Robot current state"   , 0);
TERMINAL_PARAMETER_INT(prevState, "Robot previous state"  , 0);
TERMINAL_PARAMETER_DOUBLE(lastStateChange, "Last state change"  , 0.0);
TERMINAL_PARAMETER_DOUBLE(stateSmoothing,
                          "Smooth time after state change [s]", 1.0);

// -1 : Shooting from left
// +1 : Shooting from right
TERMINAL_PARAMETER_INT(shootingSide, "Robot shooting side", 1);

/**
 * Vous pouvez écrire du code qui sera exécuté à 
 * l'initialisation ici
 */
void setup()
{
  initMotors();
}

/**
 * Fonction appellée à 50hz, c'est ici que vous pouvez mettre
 * à jour les angles moteurs etc.
 */
void tick()
{
  t += 0.02; // 20ms

  if (prevState != state) lastStateChange = t;

  if (state == 0) {
    //if (prevState != 0)
    prevState = 0;
    return;
  }
  if (state != prevState) {
    startSmoothing(t, stateSmoothing);
  }
  switch(state) {
  case INIT_STATE: {
    //setPosture(initRearX, initRearZ, initForeX,initForeZ, initLatAngle);
    double dPitch = sin(t * M_PI * 2.0 / oscPeriod) * oscAmp;
    setAllFromIK(t, initForeX, initRearX, initAvgZ, initPitch + dPitch);
    break;
  }
  case WALK_STATE: {
    move(t, initForeX, initRearX, initAvgZ, initPitch);
    break;
  }
  case SHOOT_STATE: {
    if (shoot(t, lastStateChange, shootingSide) == 1) {
      state = INIT_STATE;
    }
    break;
  }
  case ATR_STATE: {
    performATR(t, lastStateChange);
    break;
  }
  case BACK_FROM_ATR_STATE: {
    backFromATR(t, lastStateChange);
    break;
  }
  }
  prevState = state;
}

/**
 * Si vous souhaitez écrire ici du code, cette fonction sera
 * apellée en boucle
 */
void loop()
{
}
