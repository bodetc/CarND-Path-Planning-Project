//
// Created by Cédric Bodet on 05.11.17.
//

#ifndef PATH_PLANNING_POLYNOMIALTRAJECTORY_H
#define PATH_PLANNING_POLYNOMIALTRAJECTORY_H


#include <utility>

#include "Polynomial.h"
#include "State.h"
#include "Trajectory.h"

class PolynomialTrajectory {
private:
  Polynomial s;
  Polynomial s_dot;
  Polynomial s_ddot;
  Polynomial d;
  Polynomial d_dot;
  Polynomial d_ddot;
  double t_max;

public:
  PolynomialTrajectory() = default;

  PolynomialTrajectory(Polynomial s_, Polynomial d_, double t_max);

  PolynomialTrajectory(const PolynomialTrajectory &other) = default;

  PolynomialTrajectory &operator=(const PolynomialTrajectory &other) = default;

  const State stateAt(double t) const;

  const Trajectory toTrajectory() const;

  const std::vector<State> toStates() const;

  double get_t_max() const { return t_max; }
};


#endif //PATH_PLANNING_POLYNOMIALTRAJECTORY_H
