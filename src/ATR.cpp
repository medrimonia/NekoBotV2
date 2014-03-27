#include "ATR.hpp"

#include <terminal.h>

#include "Interpolation.hpp"
#include "NekobotMotors.hpp"

// FINAL
TERMINAL_PARAMETER_DOUBLE(atrFinalBack2Humerus  , "",  -56.0);
TERMINAL_PARAMETER_DOUBLE(atrFinalHumerus2Radius, "",  150.0);
TERMINAL_PARAMETER_DOUBLE(atrFinalBack2Femur    , "",  -90.0);
TERMINAL_PARAMETER_DOUBLE(atrFinalFemur2Tibia   , "",    0.0);
TERMINAL_PARAMETER_DOUBLE(atrFinalTibia2Foot    , "",  -20.0);

// STEP3
TERMINAL_PARAMETER_DOUBLE(atrS3Back2Humerus  , "",  -53.0);
TERMINAL_PARAMETER_DOUBLE(atrS3Humerus2Radius, "",  150.0);
TERMINAL_PARAMETER_DOUBLE(atrS3Back2Femur    , "",  -70.0);
TERMINAL_PARAMETER_DOUBLE(atrS3Femur2Tibia   , "",   0.0);
TERMINAL_PARAMETER_DOUBLE(atrS3Tibia2Foot    , "",  -20.0);

// STEP 2
TERMINAL_PARAMETER_DOUBLE(atrS2Back2Humerus  , "",  -40.0);
TERMINAL_PARAMETER_DOUBLE(atrS2Humerus2Radius, "",  150.0);
TERMINAL_PARAMETER_DOUBLE(atrS2Back2Femur    , "",    0.0);
TERMINAL_PARAMETER_DOUBLE(atrS2Femur2Tibia   , "",    0.0);
TERMINAL_PARAMETER_DOUBLE(atrS2Tibia2Foot    , "",  -20.0);

// STEP 1
TERMINAL_PARAMETER_DOUBLE(atrPrepBack2Humerus  , "", - 50.0);
TERMINAL_PARAMETER_DOUBLE(atrPrepHumerus2Radius, "",  150.0);
TERMINAL_PARAMETER_DOUBLE(atrPrepBack2Femur    , "",   70.0);
TERMINAL_PARAMETER_DOUBLE(atrPrepFemur2Tibia   , "", - 20.0);
TERMINAL_PARAMETER_DOUBLE(atrPrepTibia2Foot    , "",    0.0);

// INIT
TERMINAL_PARAMETER_DOUBLE(atrInitForeX, "Initial IK posture", - 20.0);
TERMINAL_PARAMETER_DOUBLE(atrInitRearX, "Initial IK posture",    0.0);
TERMINAL_PARAMETER_DOUBLE(atrInitAvgZ , "Initial IK posture",  130.0);
TERMINAL_PARAMETER_DOUBLE(atrInitPitch, "Initial IK posture", - 20.0);

// BACK 1
TERMINAL_PARAMETER_DOUBLE(atrB1Back2Humerus  , "",  -45.0);
TERMINAL_PARAMETER_DOUBLE(atrB1Humerus2Radius, "",  150.0);
TERMINAL_PARAMETER_DOUBLE(atrB1Back2Femur    , "",  -50.0);
TERMINAL_PARAMETER_DOUBLE(atrB1Femur2Tibia   , "",    0.0);
TERMINAL_PARAMETER_DOUBLE(atrB1Tibia2Foot    , "",  -20.0);

// BACK 2
TERMINAL_PARAMETER_DOUBLE(atrB2Back2Humerus  , "",  -40.0);
TERMINAL_PARAMETER_DOUBLE(atrB2Humerus2Radius, "",  150.0);
TERMINAL_PARAMETER_DOUBLE(atrB2Back2Femur    , "",    0.0);
TERMINAL_PARAMETER_DOUBLE(atrB2Femur2Tibia   , "",    0.0);
TERMINAL_PARAMETER_DOUBLE(atrB2Tibia2Foot    , "",  -20.0);

// BACK 3
TERMINAL_PARAMETER_DOUBLE(atrB3Back2Humerus  , "",  -50.0);
TERMINAL_PARAMETER_DOUBLE(atrB3Humerus2Radius, "",  150.0);
TERMINAL_PARAMETER_DOUBLE(atrB3Back2Femur    , "",   40.0);
TERMINAL_PARAMETER_DOUBLE(atrB3Femur2Tibia   , "",  -50.0);
TERMINAL_PARAMETER_DOUBLE(atrB3Tibia2Foot    , "",  -20.0);

TERMINAL_PARAMETER_INT(atrState, "Current Part of ATR", 0);

#define ATR_STATE_INIT         0
#define ATR_STATE_PREPARATION  1
#define ATR_STATE_STEP2        2
#define ATR_STATE_STEP3        3
#define ATR_STATE_FINAL        4


TERMINAL_PARAMETER_DOUBLE(atrInitTime,      "Time to init"        ,  1.0 );
TERMINAL_PARAMETER_DOUBLE(atrPrepTime,      "From init to prep"   ,  0.25);
TERMINAL_PARAMETER_DOUBLE(atrStabilizeTime, "On preparation state",  1.0 );
TERMINAL_PARAMETER_DOUBLE(atrS2Time,        "From prep to S2"     ,  1.5 );
TERMINAL_PARAMETER_DOUBLE(atrS3Time,        "From S2 to S3"       ,  1.5 );
TERMINAL_PARAMETER_DOUBLE(atrFinalTime,     "From S3 to final"    ,  1.0 );
TERMINAL_PARAMETER_DOUBLE(atrB1Time,        "From Final to B1"    ,  2.5 );
TERMINAL_PARAMETER_DOUBLE(atrB2Time,        "From B1 to B2"       ,  2.5 );
TERMINAL_PARAMETER_DOUBLE(atrB3Time,        "From B1 to B2"       ,  2.5 );

void initATR(double time)
{
  atrState = ATR_STATE_INIT;
  setAllFromIK(time, atrInitForeX, atrInitRearX, atrInitAvgZ, atrInitPitch);
}

void setPrepATR(double time)
{
  if (atrState != ATR_STATE_PREPARATION) {
    atrState = ATR_STATE_PREPARATION;
    startSmoothing(time, atrPrepTime);
  }
  setFromAngles(time, atrPrepBack2Humerus, atrPrepHumerus2Radius,
                atrPrepBack2Femur, atrPrepFemur2Tibia, atrPrepTibia2Foot);
}

#define REGISTER_ATR_POS(idx, prefix, time)              \
{                                                        \
  atrBack2Humerus  [idx] =  prefix ## Back2Humerus;      \
  atrHumerus2Radius[idx] =  prefix ## Humerus2Radius;    \
  atrBack2Femur    [idx] =  prefix ## Back2Femur;        \
  atrFemur2Tibia   [idx] =  prefix ## Femur2Tibia;       \
  atrTibia2Foot    [idx] =  prefix ## Tibia2Foot;        \
  timing [idx] = time;                                   \
}

#define CALC_ATR_POS(time, suffix)                                \
  cubicInterpolation(time, timing, atr ## suffix, 4, 0.0, 0.0)

void performATR(double time, double startTime)
{
  double elapsedTime = time - startTime;
  if (elapsedTime < atrInitTime) {
    initATR(time);
    return;
  }
  elapsedTime -= atrInitTime;
  if (elapsedTime < atrPrepTime + atrStabilizeTime) {
    setPrepATR(time);
    return;
  }
  elapsedTime -= atrPrepTime + atrStabilizeTime;
  double atrBack2Humerus  [4];
  double atrHumerus2Radius[4];
  double atrBack2Femur    [4];
  double atrFemur2Tibia   [4];
  double atrTibia2Foot    [4];
  double timing           [4];
  REGISTER_ATR_POS(0, atrPrep ,  0);
  REGISTER_ATR_POS(1, atrS2   ,  atrS2Time);
  REGISTER_ATR_POS(2, atrS3   ,  atrS2Time + atrS3Time);
  REGISTER_ATR_POS(3, atrFinal,  atrS2Time + atrS3Time + atrFinalTime);
  setFromAngles(time,
                CALC_ATR_POS(elapsedTime, Back2Humerus  ),
                CALC_ATR_POS(elapsedTime, Humerus2Radius),
                CALC_ATR_POS(elapsedTime, Back2Femur    ),
                CALC_ATR_POS(elapsedTime, Femur2Tibia   ),
                CALC_ATR_POS(elapsedTime, Tibia2Foot    ));
}

void backFromATR(double time, double startTime)
{
  double elapsedTime = time - startTime;
  double totalTime = atrB1Time + atrB2Time + atrB3Time;
  if (elapsedTime > totalTime) {
    elapsedTime = totalTime;
  }
  double atrBack2Humerus  [4];
  double atrHumerus2Radius[4];
  double atrBack2Femur    [4];
  double atrFemur2Tibia   [4];
  double atrTibia2Foot    [4];
  double timing           [4];
  REGISTER_ATR_POS(0, atrFinal ,  0);
  REGISTER_ATR_POS(1, atrB1   ,  atrB1Time);
  REGISTER_ATR_POS(2, atrB2   ,  atrB1Time + atrB2Time);
  REGISTER_ATR_POS(2, atrB3   ,  atrB1Time + atrB2Time +atrB3Time);
  setFromAngles(time,
                CALC_ATR_POS(elapsedTime, Back2Humerus  ),
                CALC_ATR_POS(elapsedTime, Humerus2Radius),
                CALC_ATR_POS(elapsedTime, Back2Femur    ),
                CALC_ATR_POS(elapsedTime, Femur2Tibia   ),
                CALC_ATR_POS(elapsedTime, Tibia2Foot    ));
  
}
