//
// Created by Cédric Bodet on 05.11.17.
//

#include "Vehicle.h"

Vehicle::Vehicle(const State& startState) : start(startState) {
  start.t = 0;
}
