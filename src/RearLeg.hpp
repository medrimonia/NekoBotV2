#ifndef REAR_LEG_HPP
#define REAR_LEG_HPP

class RearLeg {
private:
  int startIndex;
  bool inv;
public:
  RearLeg(int firstId, bool inverted) : startIndex(firstId), inv(inverted) {}

  /** Lateral Angle
   * + -> leg outside
   * - -> leg inside
   */
  void setLatAngle(double a);
  void setFemurAngle(double a);
  void setTibiaAngle(double a);
  void setFootAngle(double a);
  double getLatAngle();
  double getFemurAngle();
  double getTibiaAngle();
  double getFootAngle();
  /* Use an array of size 4 :
   * [ lat, femur, tibia, foot]
   */
  void setAngles(double * a);

  /** Compute an inverse kinematic for the rear leg according to the specified
   * z (height from top of Femur)
   *    and
   * x (horizontal distance from top of Femur)
   */
  void setFromIK(double x, double z);

  void enable();
  void disable();
};

#endif//REAR_LEG_HPP
