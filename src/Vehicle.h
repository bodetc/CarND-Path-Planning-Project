//
// Created by Cédric Bodet on 05.11.17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

/**
 * Non-ego vehicles move w/ constant acceleration
 */
struct Vehicle {
  const double s;
  const double d;
  const double speed;

public:
  Vehicle(double s, double d, double speed) : s(s), d(d), speed(speed) {}

  double s_at(double t) const { return s + speed * t; }
};

#endif //PATH_PLANNING_VEHICLE_H