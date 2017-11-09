//
// Created by Cédric Bodet on 27.10.17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H


#include <vector>

class Trajectory {
private:
  std::vector<double> x_vals;
  std::vector<double> y_vals;

public:
  Trajectory(const std::vector<double> &x, const std::vector<double> &y);

  const std::vector<double> &getX() { return x_vals; }

  const std::vector<double> &getY() { return y_vals; }

  void push_back(double x, double y);
};


#endif //PATH_PLANNING_TRAJECTORY_H
