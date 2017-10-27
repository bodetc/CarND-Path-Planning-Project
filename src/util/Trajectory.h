//
// Created by Cédric Bodet on 27.10.17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H


#include <vector>

class Trajectory {
private:
  std::vector<double> a_vals;
  std::vector<double> b_vals;

public:
  Trajectory(const std::vector<double> &a, const std::vector<double> &b);

  Trajectory() = default;

  const std::vector<double> &getX() const { return a_vals; }

  const std::vector<double> &getY() const { return b_vals; }

  size_t size() const { return a_vals.size(); }

  std::vector<double> at(int i) const { return {a_vals[i], b_vals[i]}; }

  void push_back(double a, double b);

  void push_back(const std::vector<double> &values);
};


#endif //PATH_PLANNING_TRAJECTORY_H
