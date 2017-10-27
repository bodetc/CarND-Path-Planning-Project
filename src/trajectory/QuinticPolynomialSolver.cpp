//
// Created by Cédric Bodet on 27.10.17.
//

#include "QuinticPolynomialSolver.h"

#include "../Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::Matrix3d;
using Eigen::Vector3d;

/*
Calculate the Jerk Minimizing Trajectory that connects the initial state
to the final state in time T.

INPUTS

start - the vehicles start location given as a length three array
    corresponding to initial values of [s, s_dot, s_double_dot]

end   - the desired end state for vehicle. Like "start" this is a
    length three array.

T     - The duration, in seconds, over which this maneuver should occur.

OUTPUT
an array of length 6, each value corresponding to a coefficent in the polynomial
s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

EXAMPLE

> JMT( [0, 10, 0], [10, 10, 0], 1)
[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
*/
Polynomial
QuinticPolynomialSolver::solveJMT(std::vector<double> start, std::vector<double> end, double T) {
  if (start.size() != 3 || end.size() != 3) {
    throw std::length_error("start and end vectors must have size 3!");
  }

  Matrix3d A;
  A << T * T * T, T * T * T * T, T * T * T * T * T,
      3. * T * T, 4. * T * T * T, 5. * T * T * T * T,
      6. * T, 12. * T * T, 20. * T * T * T;

  double b1 = end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T);
  double b2 = end[1] - (start[1] + start[2] * T);
  double b3 = end[2] - (start[2]);

  Vector3d b;
  b << b1, b2, b3;

  Vector3d x = A.inverse() * b;

  vector<double> alphas = {start[0], start[1], .5 * start[2]};
  for (int i = 0; i < 3; i++) {
    alphas.push_back(x[i]);
  }

  return Polynomial(alphas);
}
