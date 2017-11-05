//
// Created by Cédric Bodet on 04.11.17.
//

#include <random>
#include "PTG.h"
#include "../definitions.h"
#include "PolynomialSolver.h"

using namespace std;

Trajectory PTG::operator()(const State& start_state, const Vehicle& target, const State &delta, double T,
                           const vector<Vehicle> &predictions) {
  vector<State> allGoals((2 * PTG_N_STEPS + 1) * (PTG_N_SAMPLES + 1));

  for (int i = -PTG_N_STEPS; i <= PTG_N_STEPS; i++) {
    double t = T + i * PTG_N_STEPS;
    State goal = target.stateAt(t) + delta;

    allGoals.push_back(goal);

    for (int j = 0; j < PTG_N_SAMPLES; j++) {
      allGoals.push_back(perturbGoal(goal));
    }
  }

  PolynomialTrajectory bestTrajectory;
  double bestCost = std::numeric_limits<double>::max();
  for(auto goal : allGoals) {
    PolynomialTrajectory trajectory = PolynomialSolver::solveJMT(start_state, goal);
    double cost = costCalculator.calculateCost(goal.t, trajectory, target, delta, T, predictions);
    if(cost < bestCost) {
      bestTrajectory=trajectory;
      bestCost = cost;
    }
  }

  return bestTrajectory.toTrajectory();
}

const State PTG::perturbGoal(const State &other) {
  return State {
      .s = other.s + PTG_SIGMA_S * distribution(generator),
      .s_dot = other.s_dot + PTG_SIGMA_S_DOT * distribution(generator),
      .s_ddot = other.s_ddot + PTG_SIGMA_S_DDOT * distribution(generator),
      .d = other.d + PTG_SIGMA_D * distribution(generator),
      .d_dot = other.d_dot + PTG_SIGMA_D_DOT * distribution(generator),
      .d_ddot = other.d_ddot + PTG_SIGMA_D_DDOT * distribution(generator),
      .t = other.t
  };
}
