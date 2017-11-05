//
// Created by Cédric Bodet on 05.11.17.
//

#ifndef PATH_PLANNING_COSTCALCULATOR_H
#define PATH_PLANNING_COSTCALCULATOR_H

#include "../util/Trajectory.h"

class AbstractCostFunction {
public:
  virtual ~AbstractCostFunction() = default;
  virtual double operator()(const Trajectory& ego, double T) = 0;
};


class CostCalculator {
private:
  std::vector<AbstractCostFunction*> costFunctions;
  std::vector<double> weights;

public:
  CostCalculator();
  ~CostCalculator();

  double operator()(const Trajectory& ego, double T);
};


#endif //PATH_PLANNING_COSTCALCULATOR_H
