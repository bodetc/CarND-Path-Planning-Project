//
// Created by Cédric Bodet on 28.10.17.
//

#ifndef PATH_PLANNING_POLYNOMIALTRAJECTORY_H
#define PATH_PLANNING_POLYNOMIALTRAJECTORY_H


#include <vector>

class Polynomial {
private:
  std::vector<double> alphas;

public:
  explicit Polynomial(const std::vector<double> &alphas);

  double evaluate(double t) const;
};


#endif //PATH_PLANNING_POLYNOMIALTRAJECTORY_H