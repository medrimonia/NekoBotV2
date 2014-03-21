#include "Interpolation.hpp"

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
