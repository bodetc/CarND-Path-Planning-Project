//
// Created by Cédric Bodet on 27.10.17.
//

#include "Polynomial.h"

Polynomial::Polynomial(std::vector<double> alphas) : alphas(std::move(alphas)) {}

double Polynomial::evaluate(double t) const {
  double value = 0.;
  double pow = 1.;
  for (int i = 0; i < alphas.size(); i++) {
    value += alphas[i] * pow;
    pow *= t;
  }
  return value;
}

const Polynomial Polynomial::derivative() const {
  std::vector<double> d_alphas;
  for (int degree = 1; degree < alphas.size(); degree++) {
    d_alphas.push_back(degree * alphas[degree]);
  }
  return Polynomial(d_alphas);
}
