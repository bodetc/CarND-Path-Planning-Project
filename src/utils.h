//
// Created by Cédric Bodet on 27.10.17.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>
#include <cmath>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

#endif //PATH_PLANNING_UTILS_H
