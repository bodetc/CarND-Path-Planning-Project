//
// Created by Cédric Bodet on 05.11.17.
//

#include "CostCalculator.h"
#include "../definitions.h"

class TimeCost : public AbstractCostFunction {
  double operator()(const Trajectory& ego, double T) override {
    return 1.;
  }
};

CostCalculator::CostCalculator() {
  weights.push_back(TIME_DIFF_COST);
  costFunctions.push_back(new TimeCost());
}

CostCalculator::~CostCalculator() {
  // free the allocated memory
  for (auto &costFunction : costFunctions) {
    delete costFunction;
  }
  // empty the container
  costFunctions.clear();
}

double CostCalculator::operator()(const Trajectory &ego, double T) {
  if(costFunctions.size()!=weights.size()) {
    throw std::length_error("Cost functions and weights must have the same size!");
  }
  double totalCost = 0;
  for(int i = 0; i<costFunctions.size(); i++) {
    auto costFunction = costFunctions[i];
    totalCost += weights[i] * (*costFunction)(ego, T);
  }
  return totalCost;
}
