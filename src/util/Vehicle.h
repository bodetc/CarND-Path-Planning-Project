//
// Created by Cédric Bodet on 05.11.17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <vector>
#include "State.h"

/**
 * Non-ego vehicles move w/ constant acceleration
 */
class Vehicle {
private:
  State start;

public:
  Vehicle(const State startState) : start(startState) {}
  ~Vehicle() = default;

  State stateAt(double T) const;
};

#endif //PATH_PLANNING_VEHICLE_H
