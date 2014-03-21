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
  /* Use an array of size 4 :
   * [ lat, femur, tibia, foot]
   */
  void setAngles(double * a);

  void enable();
  void disable();
};

#endif//REAR_LEG_HPP
