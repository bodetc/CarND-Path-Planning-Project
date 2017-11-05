//
// Created by Cédric Bodet on 04.11.17.
//

#ifndef PATH_PLANNING_PTG_H
#define PATH_PLANNING_PTG_H


#include <random>
#include "CostCalculator.h"
#include "../util/Vehicle.h"

class PTG {
private:
  std::default_random_engine generator;
  std::normal_distribution<double> distribution;

  CostCalculator costCalculator;

  const State perturbGoal(const State &goal);

public:
  Trajectory operator()(const State& start_state, const Vehicle& target, const State& delta, double T, const std::vector<Vehicle>& predictions);
};


#endif //PATH_PLANNING_PTG_H
