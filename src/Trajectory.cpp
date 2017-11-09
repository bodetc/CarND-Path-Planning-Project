//
// Created by Cédric Bodet on 27.10.17.
//

#include "Trajectory.h"

Trajectory::Trajectory(const std::vector<double> &x, const std::vector<double> &y) : x_vals(x), y_vals(y) {}

void Trajectory::push_back(double x, double y) {
  x_vals.push_back(x);
  y_vals.push_back(y);
}
