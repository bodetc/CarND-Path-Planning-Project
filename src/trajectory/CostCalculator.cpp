//
// Created by Cédric Bodet on 05.11.17.
//

#include "CostCalculator.h"
#include "../definitions.h"

class TimeCost : public AbstractCostFunction {
  double calculateCost(double t, const PolynomialTrajectory& ego, const Vehicle& target, const State& delta, double T, const std::vector<Vehicle>& predictions) {
    return 1.;
  }
};

CostCalculator::CostCalculator() {
  weightedCostFunctions.push_back(WeightedCostFunction { .weight = TIME_DIFF_COST, .costFunction = new TimeCost()});
}

CostCalculator::~CostCalculator() {
  // free the allocated memory
  for (auto &wcf : weightedCostFunctions) {
    delete wcf.costFunction;
  }
  // empty the container
  weightedCostFunctions.clear();
}

double CostCalculator::calculateCost(double t, const PolynomialTrajectory& ego, const Vehicle& target, const State& delta, double T, const std::vector<Vehicle>& predictions) {
  double totalCost = 0;
  for (auto &wcf : weightedCostFunctions) {
    totalCost += wcf.weight * wcf.costFunction->calculateCost(t, ego, target, delta, T, predictions);
  }
  return totalCost;
}
