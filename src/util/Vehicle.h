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
  const State start;

public:
  explicit Vehicle(const State& startState);
  ~Vehicle() = default;

  const State stateAt(double T) const;
};

#endif //PATH_PLANNING_VEHICLE_H
