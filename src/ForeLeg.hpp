#ifndef FORE_LEG_HPP
#define FORE_LEG_HPP

class ForeLeg {
private:
  int startIndex;
  bool inv;

  double smoothStart;
  double smoothingTime;
  double oldLat, oldHumerus, oldRadius;

public:
  ForeLeg(int firstId, bool inverted) : startIndex(firstId), inv(inverted) {
    smoothStart = 0;
    smoothingTime = 1;
    oldLat = 0;
    oldHumerus = 0;
    oldRadius = 0;
  }

  // Init motors with default min, max and zero values
  void init();

  /** Lateral Angle
   * + -> leg outside
   * - -> leg inside
   */
  void setLatAngle    (double time, double a);
  void setHumerusAngle(double time, double a);
  void setRadiusAngle (double time, double a);
  double getLatAngle();
  double getHumerusAngle();
  double getRadiusAngle();
  /* Use an array of size 3 :
   * [ lat, humerus, radius]
   */
  void setAngles(double time, double * a);

  /** Compute an inverse kinematic for the fore leg according to the specified
   * z (height from top of Femur)
   *    and
   * x (horizontal distance from top of Femur)
   */
  void setFromIK(double time, double x, double z, double robotPitch = 0);

  void enable();
  void disable();

  void startSmoothing(double time, double smoothLength);
};

#endif//FORE_LEG_HPP
