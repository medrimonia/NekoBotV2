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


#endif//NEKOBOT_MOTORS_HPP
