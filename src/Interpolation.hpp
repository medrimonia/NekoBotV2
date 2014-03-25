#ifndef INTERPOLATION_HPP
#define INTERPOLATION_HPP


double linearInterpolation(double src, double tar,
                           double elapsedTime, double transitionTime = 1.0);

/**
 * save the linear interpolation from src to tar in dst array
 * n values are used
 */
void linearInterpolation(double * dst,
                         double * src,
                         double * tar,
                         int n,
                         double elapsedTime,
                         double transitionTime = 1.0);

#endif//INTERPOLATION_HPP
