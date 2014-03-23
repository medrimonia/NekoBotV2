#include <cstdlib>
#include <wirish/wirish.h>
#include <servos.h>
#include <terminal.h>
#include <main.h>
#include <dxl.h>
#include <function.h>

#include "Interpolation.hpp"
#include "NekobotMotors.hpp"
#include "Shoot.hpp"
#include "Walk.hpp"

TERMINAL_PARAMETER_DOUBLE(t, "Temps", 0.0);

TERMINAL_PARAMETER_DOUBLE(period, "Period length", 5.0);

//rear IK init
TERMINAL_PARAMETER_DOUBLE(initRearX, "Init X of rear foots",   22.0);
TERMINAL_PARAMETER_DOUBLE(initRearZ, "Init Z of rear foots",  170.0);
//fore IK default
TERMINAL_PARAMETER_DOUBLE(initForeX, "Init X of fore foots",  -20.0);
TERMINAL_PARAMETER_DOUBLE(initForeZ, "Init Z of fore foots",  170.0);

TERMINAL_PARAMETER_DOUBLE(initLatAngle, "Init lat angle of legs", 12.0);

// States handling
#define FREE_MOVE_STATE 0
#define INIT_STATE      1
#define WALK_STATE      2
#define SHOOT_STATE     3

TERMINAL_PARAMETER_INT(state    , "Robot current state"   , 0);
TERMINAL_PARAMETER_INT(prevState, "Robot previous state"  , 0);
TERMINAL_PARAMETER_DOUBLE(lastStateChange, "Last state change"  , 0.0);

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
  initWalking();
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
    if (prevState != 0) disableMotors();
    prevState = 0;
    return;
  }
  if (prevState == 0) enableMotors();
  switch(state) {
  case INIT_STATE: {
    setPosture(initRearX, initRearZ, initForeX,initForeZ, initLatAngle);
    break;
  }
  case WALK_STATE: {
    move(t, initRearX, initRearZ, initForeX, initForeZ, initLatAngle);
    break;
  }
  case SHOOT_STATE: {
    if (shoot(t, lastStateChange, shootingSide) == 1) {
      state = INIT_STATE;
    }
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
