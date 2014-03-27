#include "Interpolation.hpp"

#include <cmath>
#include <cstdio>

//#define TEST_INTERPOLATION

double linearInterpolation(double src, double tar,
                           double elapsedTime, double transitionTime)
{
  double ratio = elapsedTime / transitionTime;
  ratio = ratio > 1 ? 1 : ratio;
  return ratio * tar + (1 - ratio) * src;
}

void linearInterpolation(double * dst,
                         double * src,
                         double * tar,
                         int n,
                         double elapsedTime,
                         double transitionTime)
{
  for (int i = 0; i < n; i++) {
    dst[i] = linearInterpolation(src[i], tar[i], elapsedTime, transitionTime);
  }
}

void cubicInterpolation(double y0, double y1, double dy0, double dy1,
                        double * a)
{
  a[0] =  2 * y0 - 2 * y1 +     dy0 + dy1;
  a[1] = -3 * y0 + 3 * y1 - 2 * dy0 - dy1;
  a[2] = dy0;
  a[3] = y0;
}

void cubicInterpolation(double x0, double y0, double x1, double y1,
                        double dy0, double dy1, double * a)
{
  double dx = x1 - x0;
  double offset = x0;
  cubicInterpolation(y0, y1, dy0, dy1, a);
  normalizeCubicFunction(a, dx, offset);
}

void normalizeCubicFunction(double * a, double timeUnit, double offset)
{
  //Dilate time
  for (int i = 0; i < 3; i++) {
    a[i] /= pow(timeUnit, 3 - i);
  }
  //Time offset
  double offset2 = offset * offset;
  double offset3 = offset * offset2;
  double result[4];
  result[0] = a[0];
  result[1] = (a[1] - 3 * a[0] * offset);
  result[2] = (a[2] - 2 * a[1] * offset + 3 * a[0] * offset2);
  result[3] = (a[3] - a[2] * offset + a[1] * offset2 - a[0] * offset3);
  for (int i = 0; i < 4; i ++) {
    a[i] = result[i];
  }
}

void displayCubicFunction(const char * header, double * a)
{
  printf("%s : %.2lf x^3 + %.2lf x^2 + %.2lf x + %.2lf\n",
         header, a[0], a[1], a[2], a[3]); 
}

void cubicInterpolation(double * x, double * y, double * a)
{
  double dy0 = (y[2] - y[0]) / (x[2] - x[0]);
  double dy1 = (y[3] - y[1]) / (x[3] - x[1]);
  cubicInterpolation(x[1], y[1], x[2], y[2], dy0, dy1, a);
}


//TODO add acyclic
double cubicInterpolation(double time, double * x, double * y, int nbPoints,
                          double initDy, double endDy)
{
  int index = 0;
  // Placing in a point such as x[index - 1] < time and x[index] >= time
  while (index < nbPoints) {
    if (x[index] >= time)
      break;
    index++;
  }
  //Special case, x > end | x < start
  if (index == 0) return y[0];
  if (index == nbPoints) return y[nbPoints -1];
  double dy0 = initDy;
  double dy1 = endDy;
  // Point available at start
  if (index >= 2) {
    dy0 = (y[index] - y[index - 2]) / (x[index] - x[index - 2]);
  }
  if (index < nbPoints - 1) {
    dy1 = (y[index + 1] - y[index - 1]) / (x[index + 1] - x[index - 1]);
  }
  double a[4];
  cubicInterpolation(x[index-1], y[index-1],x[index], y[index], dy0, dy1, a);
  return evalCubicFunction(a, time);  
}

double evalCubicFunction(double * a, double x)
{
  double x2 = x * x;
  double x3 = x * x2;
  return a[0] * x3 + a[1] * x2 + a[2] * x + a[3];
}


#ifdef TEST_INTERPOLATION
int main(int argc, char ** argv) {
  double a[4];
  //cubicInterpolation(-1, 0, 0, 3, a);// x^3 - 1
  //normalizeCubicFunction(a, 1.0, 0);
  //displayCubicFunction("x^3 - 1", a);
  double y[4] = { 0, 0, 5, 5};
  double x[4] = {-1, 2.5, 3.5, 4};
  cubicInterpolation(x, y, a);
  for (double t = 2.5; t <= 3.5; t += 0.02) {
    printf("%lf\n", evalCubicFunction(a, t));
  }
}
#endif
