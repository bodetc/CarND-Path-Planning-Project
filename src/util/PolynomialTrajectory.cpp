//
// Created by Cédric Bodet on 05.11.17.
//

#include "PolynomialTrajectory.h"
#include "../definitions.h"

State PolynomialTrajectory::stateAt(double t) {
  return State {
      .s = s.evaluate(t),
      .s_dot = s_dot.evaluate(t),
      .s_ddot = s_ddot.evaluate(t),
      .d = d.evaluate(t),
      .d_dot = d_dot.evaluate(t),
      .d_ddot = d_ddot.evaluate(t),
  };
}

Trajectory PolynomialTrajectory::toTrajectory() {
  Trajectory trajectory;
  for (int i = 1; i <= N_STEPS; i++) {
    double s_ = s.evaluate(i * TIMESTEP);
    double d_ = d.evaluate(i * TIMESTEP);
    trajectory.push_back(s_, d_);
  }
  return trajectory;
}
