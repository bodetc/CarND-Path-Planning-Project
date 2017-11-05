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
  PolynomialTrajectory(Polynomial s_, Polynomial d_) :
      s(std::move(s_)),
      s_dot(s.derivative()),
      s_ddot(s_dot.derivative()),
      d(std::move(d_)),
      d_dot(d.derivative()),
      d_ddot(d_dot.derivative()) {};

  State stateAt(double t);

  Trajectory toTrajectory();
};


#endif //PATH_PLANNING_POLYNOMIALTRAJECTORY_H
