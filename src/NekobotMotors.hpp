#ifndef NEKOBOT_MOTORS_HPP
#define NEKOBOT_MOTORS_HPP

#include "ForeLeg.hpp"
#include "RearLeg.hpp"

extern RearLeg rightRearLeg;
extern RearLeg leftRearLeg;
extern ForeLeg rightForeLeg;
extern ForeLeg leftForeLeg;

void initMotors();

void enableMotors();

void disableMotors();

void setPosture(double rearX, double rearZ, double foreX, double foreZ,
                double latAngle);

void setUniformLat(double latAngle);


#endif//NEKOBOT_MOTORS_HPP
