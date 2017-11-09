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

  CostCalculator cost_calculator;

  const State perturb_goal(const State &goal);

public:
  const Trajectory getTrajectory(const State& start_state, const Vehicle& target, double T, const std::vector<Vehicle>& predictions);
};


#endif //PATH_PLANNING_PTG_H
