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

inline double distance(double x1, double y1, double x2, double y2) { return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)); }

inline double distance(std::vector<double> a, std::vector<double> b) {
  if(a.size()!=2 || b.size()!=2) {
    throw std::length_error("Must supply exactly two arguments!");
  }
  return sqrt((b[0]-a[0])*(b[0]-a[0])+(b[1]-a[1])*(b[1]-a[1])); 
}

/**
 * A function that returns a value between 0 and 1 for x in the
 * range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
 *
 * Useful for cost functions.
 */
inline double logistic(double x) {
  return 2. / (1. + exp(-x)) - 1.;
}

#endif //PATH_PLANNING_UTILS_H
