#ifndef FORE_LEG_HPP
#define FORE_LEG_HPP

class ForeLeg {
private:
  int startIndex;
  bool inv;
public:
  ForeLeg(int firstId, bool inverted) : startIndex(firstId), inv(inverted) {}

  /** Lateral Angle
   * + -> leg outside
   * - -> leg inside
   */
  void setLatAngle(double a);
  void setHumerusAngle(double a);
  void setRadiusAngle(double a);
  double getLatAngle();
  double getHumerusAngle();
  double getRadiusAngle();
  /* Use an array of size 3 :
   * [ lat, humerus, radius]
   */
  void setAngles(double * a);

  /** Compute an inverse kinematic for the fore leg according to the specified
   * z (height from top of Femur)
   *    and
   * x (horizontal distance from top of Femur)
   */
  void setFromIK(double x, double z);

  void enable();
  void disable();
};

#endif//FORE_LEG_HPP
