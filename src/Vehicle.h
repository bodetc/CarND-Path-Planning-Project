//
// Created by Cédric Bodet on 05.11.17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

/**
 * Non-ego vehicles move are considered to move with constant speed along the road
 */
struct Vehicle {
  const double s;
  const double d;
  const double speed;

public:
  Vehicle(double s, double d, double speed) : s(s), d(d), speed(speed) {}

  // Position along the road a give time in the future
  double s_at(double t) const { return s + speed * t; }
};

#endif //PATH_PLANNING_VEHICLE_H