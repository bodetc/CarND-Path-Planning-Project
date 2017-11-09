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

  virtual double
  calculate_cost(const std::vector<State> &ego, const Vehicle &target, double T,
                 const std::vector<Vehicle> &predictions) = 0;
};

struct WeightedCostFunction {
  double weight;
  AbstractCostFunction *cost_function;
};

class CostCalculator : AbstractCostFunction {
private:
  std::vector<WeightedCostFunction> weighted_cost_functions;

public:
  CostCalculator();

  ~CostCalculator() override;

  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, double T,
                        const std::vector<Vehicle> &predictions) override;
};


#endif //PATH_PLANNING_COSTCALCULATOR_H
