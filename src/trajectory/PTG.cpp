//
// Created by Cédric Bodet on 04.11.17.
//

#include <random>
#include "PTG.h"
#include "../definitions.h"
#include "PolynomialSolver.h"

using namespace std;

const Trajectory PTG::operator()(const State& start_state, const Vehicle& target, const State &delta, double T,
                           const vector<Vehicle> &predictions) {
  vector<State> all_goals;
  all_goals.reserve(((2 * PTG_N_STEPS + 1) * (PTG_N_SAMPLES + 1)));

  for (int i = -PTG_N_STEPS; i <= PTG_N_STEPS; i++) {
    double t = T + i * PTG_N_STEPS * PTG_TIMESTEP;
    State goal = target.stateAt(t) + delta;
    goal.t = t;

    all_goals.push_back(goal);

    for (int j = 0; j < PTG_N_SAMPLES; j++) {
      all_goals.push_back(perturb_goal(goal));
    }
  }

  PolynomialTrajectory best_trajectory;
  double best_cost = std::numeric_limits<double>::max();
  for(auto goal : all_goals) {
    PolynomialTrajectory trajectory = PolynomialSolver::solve_JMT(start_state, goal);
    double cost = cost_calculator.calculate_cost(goal.t, trajectory, target, delta, T, predictions);
    if(cost < best_cost) {
      best_trajectory=trajectory;
      best_cost = cost;
    }
  }

  return best_trajectory.toTrajectory();
}

const State PTG::perturb_goal(const State &other) {
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
