//
// Created by Cédric Bodet on 05.11.17.
//

#include "CostCalculator.h"
#include "../definitions.h"

class TimeCost : public AbstractCostFunction {
  double calculate_cost(double t, const PolynomialTrajectory &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) {
    return 1.;
  }
};

CostCalculator::CostCalculator() {
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = TIME_DIFF_COST, .cost_function = new TimeCost()});
}

CostCalculator::~CostCalculator() {
  // free the allocated memory
  for (auto &wcf : weighted_cost_functions) {
    delete wcf.cost_function;
  }
  // empty the container
  weighted_cost_functions.clear();
}

double
CostCalculator::calculate_cost(double t, const PolynomialTrajectory &ego, const Vehicle &target, const State &delta,
                               double T, const std::vector<Vehicle> &predictions) {
  double totalCost = 0;
  for (auto &wcf : weighted_cost_functions) {
    totalCost += wcf.weight * wcf.cost_function->calculate_cost(t, ego, target, delta, T, predictions);
  }
  return totalCost;
}
