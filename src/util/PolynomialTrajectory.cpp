//
// Created by Cédric Bodet on 05.11.17.
//

#include "PolynomialTrajectory.h"
#include "../definitions.h"

PolynomialTrajectory::PolynomialTrajectory(Polynomial s_, Polynomial d_, double t_) :
    s(std::move(s_)),
    s_dot(s.derivative()),
    s_ddot(s_dot.derivative()),
    d(std::move(d_)),
    d_dot(d.derivative()),
    d_ddot(d_dot.derivative()),
    t_max(t_) {};

const State PolynomialTrajectory::stateAt(double t) const {
  return State {
      .s = s.evaluate(t),
      .s_dot = s_dot.evaluate(t),
      .s_ddot = s_ddot.evaluate(t),
      .d = d.evaluate(t),
      .d_dot = d_dot.evaluate(t),
      .d_ddot = d_ddot.evaluate(t),
      .t = t,
  };
}

const Trajectory PolynomialTrajectory::toTrajectory() const {
  Trajectory trajectory;
  for (int i = 1; i <= N_STEPS; i++) {
    double s_ = s.evaluate(i * TIMESTEP);
    double d_ = d.evaluate(i * TIMESTEP);
    trajectory.push_back(s_, d_);
  }
  return trajectory;
}

const std::vector<State> PolynomialTrajectory::toStates() const {
  std::vector<State> states;
  for (double t = 0; t <= t_max; t += TIMESTEP) {
    states.push_back(stateAt(t));
  }
  return states;
}
