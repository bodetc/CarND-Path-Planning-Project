//
// Created by Cédric Bodet on 05.11.17.
//

#include "CostCalculator.h"
#include "../definitions.h"
#include "../utils.h"

/**
 * Penalizes trajectories that span a duration which is longer or shorter than the duration requested.
 */
class TimeCost : public AbstractCostFunction {
  double calculate_cost(const PolynomialTrajectory &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    return logistic(fabs(ego.get_destination_time() - T) / T);
  }
};

/**
 * Penalizes trajectories whose s coordinate (and derivatives) differ from the goal.
 */
class SDiffCost : public AbstractCostFunction {
  double calculate_cost(const PolynomialTrajectory &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    double t = ego.get_destination_time();
    State destination = ego.get_destination();
    State goal = target.stateAt(t) + delta;

    double cost = 0.;
    cost += logistic(fabs(goal.s - destination.s) / PTG_SIGMA_S);
    cost += logistic(fabs(goal.s_dot - destination.s_dot) / PTG_SIGMA_S_DOT);
    cost += logistic(fabs(goal.s_ddot - destination.s_ddot) / PTG_SIGMA_S);
    return cost;
  }
};

/**
 * Penalizes trajectories whose d coordinate (and derivatives) differ from the goal.
 */
class DDiffCost : public AbstractCostFunction {
  double calculate_cost(const PolynomialTrajectory &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    double t = ego.get_destination_time();
    State destination = ego.get_destination();
    State goal = target.stateAt(t) + delta;

    double cost = 0.;
    cost += logistic(fabs(goal.d - destination.d) / PTG_SIGMA_D);
    cost += logistic(fabs(goal.d_dot - destination.d_dot) / PTG_SIGMA_D_DOT);
    cost += logistic(fabs(goal.d_ddot - destination.d_ddot) / PTG_SIGMA_D);
    return cost;
  }
};

double nearestApproach(const PolynomialTrajectory &ego, const Vehicle &prediction) {
  double closest = std::numeric_limits<double>::max();
  double t = 0;
  while (t <= ego.get_destination_time()) {
    double d = ego.stateAt(t).distance(prediction.stateAt(t));
    if (d < closest) {
      closest = d;
    }
    t += TIMESTEP;
  }
  return closest;
}


double nearestApproach(const PolynomialTrajectory &ego, const std::vector<Vehicle> &predictions) {
  double closest = std::numeric_limits<double>::max();
  for (auto prediction : predictions) {
    double d = nearestApproach(ego, prediction);
    if (d < closest) {
      closest = d;
    }
  }
  return closest;
}

class CollisiontCost : public AbstractCostFunction {
  double calculate_cost(const PolynomialTrajectory &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    double closest = nearestApproach(ego, predictions);
    return closest < VEHICLE_RADIUS ? 1. : 0.;
  }
};

CostCalculator::CostCalculator() {
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = TIME_DIFF_COST, .cost_function = new TimeCost()});
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = S_DIFF_COST, .cost_function = new SDiffCost()});
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = D_DIFF_COST, .cost_function = new DDiffCost()});
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = COLLISION_COST, .cost_function = new CollisiontCost()});
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
CostCalculator::calculate_cost(const PolynomialTrajectory &ego, const Vehicle &target, const State &delta, double T,
                               const std::vector<Vehicle> &predictions) {
  double totalCost = 0;
  for (auto &wcf : weighted_cost_functions) {
    totalCost += wcf.weight * wcf.cost_function->calculate_cost(ego, target, delta, T, predictions);
  }
  return totalCost;
}
