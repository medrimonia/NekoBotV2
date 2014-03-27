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

/**
 * Compute the coefficient of the cubic function that passes through
 * x and y points, 4 points should be entered, 4 coefficients are written
 * in a.
 * f(x) = a[0] * x^3 + a[1] * x^2 + a[2] * x + a[3]
 */
void cubicInterpolation(double * x, double * y, double * a);

#endif//INTERPOLATION_HPP
