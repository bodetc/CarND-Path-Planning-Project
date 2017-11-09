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
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    double t = ego[ego.size() - 1].t;
    return logistic(fabs(t - T) / T);
  }
};

/**
 * Penalizes trajectories whose s coordinate (and derivatives) differ from the goal.
 */
class SDiffCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    const State &destination = ego[ego.size() - 1];
    double t = destination.t;
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
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    const State &destination = ego[ego.size() - 1];
    double t = destination.t;
    State goal = target.stateAt(t) + delta;

    double cost = 0.;
    cost += logistic(fabs(goal.d - destination.d) / PTG_SIGMA_D);
    cost += logistic(fabs(goal.d_dot - destination.d_dot) / PTG_SIGMA_D_DOT);
    cost += logistic(fabs(goal.d_ddot - destination.d_ddot) / PTG_SIGMA_D);
    return cost;
  }
};

double nearestApproach(const std::vector<State> &ego, const Vehicle &prediction) {
  double closest = std::numeric_limits<double>::max();
  for (auto state : ego) {
    double d = state.distance(prediction.stateAt(state.t));
    if (d < closest) {
      closest = d;
    }
  }
  return closest;
}


double nearestApproach(const std::vector<State> &ego, const std::vector<Vehicle> &predictions) {
  double closest = std::numeric_limits<double>::max();
  for (auto prediction : predictions) {
    double d = nearestApproach(ego, prediction);
    if (d < closest) {
      closest = d;
    }
  }
  return closest;
}

class CollisionCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    double closest = nearestApproach(ego, predictions);
    return closest < VEHICLE_RADIUS ? 1. : 0.;
  }
};

class BufferCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    double closest = nearestApproach(ego, predictions);
    return logistic(2 * VEHICLE_RADIUS / closest);
  }
};

class StaysOnRoadCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    for (auto state : ego) {
      if (state.d < 0. || state.d > 12.) {
        return 1;
      }
    }
    return 0.;
  }
};

class SpeedLimitCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    for (auto state : ego) {
      if (state.s > MAX_SPEED) {
        return 1;
      }
    }
    return 0.;
  }
};

class EfficiencyCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    const State &destination = ego[ego.size() - 1];
    const double t = destination.t;
    const double s = destination.s;
    const double s_0 = ego[0].s;
    const double speed = (s - s_0) / t;
    return logistic(2*(TARGET_SPEED-speed)/speed);
  }
};

class TotalAccelerationCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    double acc = 0;
    for (auto state : ego) {
      acc += (fabs(state.s_dot) + fabs(state.d_dot)) / TIMESTEP; // Taxidistance is quicker to compute
    }
    return logistic(acc / EXPECTED_ACC_IN_ONE_SEC);
  }
};

class MaxAccelerationCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    for (auto state : ego) {
      if (fabs(state.s_dot) > MAX_ACCEL || fabs(state.d_dot) > MAX_ACCEL) {
        return 1;
      }
    }
    return 0;
  }
};

class TotalJerkCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    double jerk = 0;
    for (auto state : ego) {
      jerk += (fabs(state.s_ddot) + fabs(state.d_ddot)) / TIMESTEP; // Taxidistance is quicker to compute
    }
    return logistic(jerk / EXPECTED_JERK_IN_ONE_SEC);
  }
};

class MaxJerkCost : public AbstractCostFunction {
  double calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                        const std::vector<Vehicle> &predictions) override {
    for (auto state : ego) {
      if (fabs(state.s_ddot) > MAX_JERK || fabs(state.d_ddot) > MAX_JERK) {
        return 1;
      }
    }
    return 0;
  }
};


CostCalculator::CostCalculator() {
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = TIME_DIFF_COST, .cost_function = new TimeCost()});
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = S_DIFF_COST, .cost_function = new SDiffCost()});
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = D_DIFF_COST, .cost_function = new DDiffCost()});
  weighted_cost_functions.push_back(
      WeightedCostFunction {.weight = COLLISION_COST, .cost_function = new CollisionCost()});
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = BUFFER_COST, .cost_function = new BufferCost()});
  weighted_cost_functions.push_back(
      WeightedCostFunction {.weight = STAYS_ON_ROAD_COST, .cost_function = new StaysOnRoadCost()});
  weighted_cost_functions.push_back(
      WeightedCostFunction {.weight = SPEED_LIMIT_COST, .cost_function = new SpeedLimitCost()});
  weighted_cost_functions.push_back(
      WeightedCostFunction {.weight = EFFICIENCY_COST, .cost_function = new EfficiencyCost()});
  weighted_cost_functions.push_back(
      WeightedCostFunction {.weight = TOTAL_ACCELERATION_COST, .cost_function = new TotalAccelerationCost()});
  weighted_cost_functions.push_back(
      WeightedCostFunction {.weight = MAX_ACCELERATION_COST, .cost_function = new MaxAccelerationCost()});
  weighted_cost_functions.push_back(
      WeightedCostFunction {.weight = TOTAL_JERK_COST, .cost_function = new TotalJerkCost()});
  weighted_cost_functions.push_back(WeightedCostFunction {.weight = MAX_JERK_COST, .cost_function = new MaxJerkCost()});
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
CostCalculator::calculate_cost(const std::vector<State> &ego, const Vehicle &target, const State &delta, double T,
                               const std::vector<Vehicle> &predictions) {
  double totalCost = 0;
  for (auto &wcf : weighted_cost_functions) {
    totalCost += wcf.weight * wcf.cost_function->calculate_cost(ego, target, delta, T, predictions);
  }
  return totalCost;
}
