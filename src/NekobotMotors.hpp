#ifndef NEKOBOT_MOTORS_HPP
#define NEKOBOT_MOTORS_HPP

#include "ForeLeg.hpp"
#include "RearLeg.hpp"

#define TIBIA2FOOT_MAX_ANGLE  135.0
#define TIBIA2FOOT_MIN_ANGLE -105.0

#define FEMUR2TIBIA_MAX_ANGLE    0.0
#define FEMUR2TIBIA_MIN_ANGLE -145.0

#define BACK2FEMUR_MAX_ANGLE  100.0
#define BACK2FEMUR_MIN_ANGLE - 95.0

#define BACK2HUMERUS_MAX_ANGLE  105.0
#define BACK2HUMERUS_MIN_ANGLE -100.0

#define HUMERUS2RADIUS_MAX_ANGLE  150.0
#define HUMERUS2RADIUS_MIN_ANGLE    0.0

extern RearLeg rightRearLeg;
extern RearLeg leftRearLeg;
extern ForeLeg rightForeLeg;
extern ForeLeg leftForeLeg;

void initMotors();

void enableMotors();

void disableMotors();

void startSmoothing(double time, double smoothingLength);

void smoothSet(int id, double src, double tar,
               double time, double tStart, double smoothLength);

void motSetMinMax(int id, double min, double max, bool inverted = false);

void setPosture(double time, double rearX, double rearZ,
                double foreX, double foreZ, double latAngle);

void setUniformLat(double time, double latAngle);

void setFromAngles(double time, double back2humerus, double humerus2radius,
                   double back2femur, double femur2tibia, double tibia2foot);

void setAllFromIK(double time, double foreX, double rearX, double avgZ,
                  double robotPitch);
#endif//NEKOBOT_MOTORS_HPP
