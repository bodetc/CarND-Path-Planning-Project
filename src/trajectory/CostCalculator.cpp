//
// Created by Cédric Bodet on 05.11.17.
//

#include "CostCalculator.h"
#include "../definitions.h"

class TimeCost : public AbstractCostFunction {
  double calculateCost(const PolynomialTrajectory& ego, int target_vehicle, const State& delta, double T, const std::vector<Vehicle>& predictions) override {
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

double CostCalculator::calculateCost(const PolynomialTrajectory& ego, int target_vehicle, const State& delta, double T, const std::vector<Vehicle>& predictions) override {
  double totalCost = 0;
  for (auto &wcf : weightedCostFunctions) {
    totalCost += wcf.weight * wcf.costFunction->calculateCost(ego, target_vehicle, delta, T, predictions);
  }
  return totalCost;
}
