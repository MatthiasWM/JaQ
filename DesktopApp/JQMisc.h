
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <stdint.h>

inline double deg2rad(double deg) { return deg / 180.0 * M_PI; }
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
double triAngleFromSSS(double a, double b, double c);
double triAngleFromSS90(double a, double b);


