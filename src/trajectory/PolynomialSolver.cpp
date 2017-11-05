//
// Created by Cédric Bodet on 28.10.17.
//

#include "PolynomialSolver.h"

#include "../Eigen-3.3/Eigen/Dense"
#include "../util/PolynomialTrajectory.h"
#include "../definitions.h"

using namespace std;
using Eigen::Matrix3d;
using Eigen::Vector3d;

Polynomial PolynomialSolver::solveJMT(const std::vector<double> &start, const std::vector<double> &end, double T) {
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

PolynomialTrajectory PolynomialSolver::solveJMT(const State &start_state, const State &end_state) {
  double t = end_state.t;

  std::vector<double> start_s {start_state.s, start_state.s_dot, start_state.s_ddot};
  std::vector<double> start_d {start_state.d, start_state.d_dot, start_state.d_ddot};

  std::vector<double> end_s {end_state.s, end_state.s_dot, end_state.s_ddot};
  std::vector<double> end_d {end_state.d, end_state.d_dot, end_state.d_ddot};

  Polynomial s_polynomial = PolynomialSolver::solveJMT(start_s, end_s, t);
  Polynomial d_polynomial = PolynomialSolver::solveJMT(start_d, end_d, t);

  return PolynomialTrajectory(s_polynomial, d_polynomial);
}
