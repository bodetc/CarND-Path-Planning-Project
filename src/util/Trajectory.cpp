//
// Created by Cédric Bodet on 27.10.17.
//

#include "Trajectory.h"

Trajectory::Trajectory(const std::vector<double> &a, const std::vector<double> &b) {
  if (a.size() != b.size()) {
    throw std::length_error("Both vector must have same length!");
  }
  a_vals = a;
  b_vals = b;
}

void Trajectory::push_back(double a, double b) {
  a_vals.push_back(a);
  b_vals.push_back(b);
}

void Trajectory::push_back(const std::vector<double> &values) {
  if (values.size() != 2) {
    throw std::length_error("Must supply exactly two arguments!");
  }
  push_back(values[0], values[1]);
}
