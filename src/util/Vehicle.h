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
  Vehicle(const Vehicle& other) = default;
  explicit Vehicle(const State& startState);
  ~Vehicle() = default;

  Vehicle& operator=(const Vehicle& other) = default;

  const State stateAt(double T) const { return start.stateAt(T); }
  const State& startState() const { return start; }
};

#endif //PATH_PLANNING_VEHICLE_H
