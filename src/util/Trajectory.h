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
  Trajectory() = default;
  Trajectory(const std::vector<double> &a, const std::vector<double> &b);

  const std::vector<double> &getX() const { return a_vals; }
  const std::vector<double> &getY() const { return b_vals; }

  size_t size() const { return a_vals.size(); }

  const std::vector<double> at(unsigned long i) const { return {a_vals[i], b_vals[i]}; }

  void push_back(double a, double b);
  void push_back(const std::vector<double> &values);

  void push_all_back(Trajectory trajectory);
};


#endif //PATH_PLANNING_TRAJECTORY_H
