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

public:
  PolynomialTrajectory() = default;
  PolynomialTrajectory(Polynomial s_, Polynomial d_);

  PolynomialTrajectory(const PolynomialTrajectory& other) = default;
  PolynomialTrajectory& operator=(const PolynomialTrajectory& other) = default;

  const State stateAt(double t) const;
  const Trajectory toTrajectory() const;
};


#endif //PATH_PLANNING_POLYNOMIALTRAJECTORY_H
