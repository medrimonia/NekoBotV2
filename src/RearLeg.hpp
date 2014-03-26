#ifndef REAR_LEG_HPP
#define REAR_LEG_HPP

class RearLeg {
private:
  int startIndex;
  bool inv;

  double smoothStart;
  double smoothingTime;
  double oldLat, oldFemur, oldTibia, oldFoot;

  // Convert value from and to internal values
  double externalVal(double intVal);
  double internalVal(double extVal);
public:
  RearLeg(int firstId, bool inverted) : startIndex(firstId), inv(inverted) {
    smoothStart = 0;
    smoothingTime = 1;
    oldLat = 0;
    oldFemur = 0;
    oldTibia = 0;
    oldFoot = 0;
  }

  // set zero and associated values
  void init();

  /** Lateral Angle
   * + -> leg outside
   * - -> leg inside
   */
  void setLatAngle(double time, double a);
  void setFemurAngle(double time, double a);
  void setTibiaAngle(double time, double a);
  void setFootAngle(double time, double a);
  double getLatAngle();
  double getFemurAngle();
  double getTibiaAngle();
  double getFootAngle();
  /* Use an array of size 4 :
   * [ lat, femur, tibia, foot]
   */
  void setAngles(double time, double * a);

  /** Compute an inverse kinematic for the rear leg according to the specified
   * z (height from top of Femur)
   *    and
   * x (horizontal distance from top of Femur)
   */
  void setFromIK(double time, double x, double z, double robotPitch = 0);

  void enable();
  void disable();

  void startSmoothing(double time, double smoothLength);
};

#endif//REAR_LEG_HPP
