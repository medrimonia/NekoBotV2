#ifndef NEKOBOT_MOTORS_HPP
#define NEKOBOT_MOTORS_HPP

#include "ForeLeg.hpp"
#include "RearLeg.hpp"

#define TIBIA2FOOT_MAX_ANGLE  135.0
#define TIBIA2FOOT_MIN_ANGLE -105.0

#define FEMUR2TIBIA_MAX_ANGLE    0.0
#define FEMUR2TIBIA_MIN_ANGLE -135.0 //TODO update to -145 with new tibia

#define BACK2FEMUR_MAX_ANGLE  100.0
#define BACK2FEMUR_MIN_ANGLE - 95.0

#define BACK2HUMERUS_MAX_ANGLE  105.0
#define BACK2HUMERUS_MIN_ANGLE -100.0

#define HUMERUS2RADIUS_MAX_ANGLE  115.0 //TODO update with a new radius
#define HUMERUS2RADIUS_MIN_ANGLE    0.0

extern RearLeg rightRearLeg;
extern RearLeg leftRearLeg;
extern ForeLeg rightForeLeg;
extern ForeLeg leftForeLeg;

void initMotors();

void enableMotors();

void disableMotors();

void motSetMinMax(int id, double min, double max, bool inverted = false);

void setPosture(double rearX, double rearZ, double foreX, double foreZ,
                double latAngle);

void setUniformLat(double latAngle);


#endif//NEKOBOT_MOTORS_HPP
