

#include "JQMisc.h"

/**
 * Returns the angle alpha based on the length of all sides
 */
double triAngleFromSSS(double a, double b, double c)
{
    return acos((a * a - b * b - c * c) / (-2.0 * b * c));
}

/**
 * Returns the angle gamma for beta=90
 */
double triAngleFromSS90(double a, double b)
{
    double alpha = asin((1 / b) * a);
    return M_PI - alpha - M_PI / 2.0;
}

