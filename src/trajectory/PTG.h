//
// Created by Cédric Bodet on 04.11.17.
//

#ifndef PATH_PLANNING_PTG_H
#define PATH_PLANNING_PTG_H


#include "CostCalculator.h"
#include "../util/Vehicle.h"

class PTG {
private:
  CostCalculator costCalculator;

public:
  Trajectory operator()(State start_state, int target_vehicle, const State& delta, double T, const std::vector<Vehicle>& predictions);
};


#endif //PATH_PLANNING_PTG_H
