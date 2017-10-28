//
// Created by Cédric Bodet on 28.10.17.
//

#include "PolynomialSolver.h"

#include "../Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::Matrix3d;
using Eigen::Vector3d;

Polynomial PolynomialSolver::solveJMT(std::vector<double> start, std::vector<double> end, double T) {
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
