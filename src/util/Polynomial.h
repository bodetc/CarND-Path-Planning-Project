//
// Created by Cédric Bodet on 28.10.17.
//

#ifndef PATH_PLANNING_POLYNOMIAL_H
#define PATH_PLANNING_POLYNOMIAL_H


#include <utility>
#include <vector>

class Polynomial {
private:
  std::vector<double> alphas;

public:
  explicit Polynomial() = default;
  explicit Polynomial(std::vector<double> alphas);

  double evaluate(double t) const;

  const Polynomial derivative() const;
};


#endif //PATH_PLANNING_POLYNOMIAL_H