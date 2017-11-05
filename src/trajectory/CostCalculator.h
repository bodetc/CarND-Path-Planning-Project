//
// Created by Cédric Bodet on 05.11.17.
//

#ifndef PATH_PLANNING_COSTCALCULATOR_H
#define PATH_PLANNING_COSTCALCULATOR_H

#include "../util/State.h"
#include "../util/Vehicle.h"
#include "../util/Trajectory.h"
#include "../util/PolynomialTrajectory.h"

class AbstractCostFunction {
public:
  virtual ~AbstractCostFunction() = default;
  virtual double calculateCost(const PolynomialTrajectory& ego, int target_vehicle, const State& delta, double T, const std::vector<Vehicle>& predictions) = 0;
};

struct WeightedCostFunction {
  double weight;
  AbstractCostFunction* costFunction;
};

class CostCalculator : AbstractCostFunction {
private:
  std::vector<WeightedCostFunction> weightedCostFunctions;

public:
  CostCalculator();
  ~CostCalculator() override;

  double calculateCost(const PolynomialTrajectory& ego, int target_vehicle, const State& delta, double T, const std::vector<Vehicle>& predictions) override;
};


#endif //PATH_PLANNING_COSTCALCULATOR_H
