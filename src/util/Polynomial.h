//
// Created by Cédric Bodet on 28.10.17.
//

#ifndef PATH_PLANNING_POLYNOMIAL_H
#define PATH_PLANNING_POLYNOMIAL_H


#include <vector>

class Polynomial {
private:
  std::vector<double> alphas;

public:
  explicit Polynomial() = default;
  explicit Polynomial(const std::vector<double> &alphas) : alphas(alphas) {}

  double evaluate(double t) const;
  Polynomial derivative() const;
};


#endif //PATH_PLANNING_POLYNOMIAL_H