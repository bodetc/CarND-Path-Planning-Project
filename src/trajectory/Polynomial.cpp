//
// Created by Cédric Bodet on 27.10.17.
//

#include "Polynomial.h"

Polynomial::Polynomial(const std::vector<double> &alphas) : alphas(alphas) {}

double Polynomial::evaluate(double t) const {
  double value = 0.;
  double pow = 1.;
  for(int i = 0; i<alphas.size(); i++) {
    value+=alphas[i]*pow;
    pow*=t;
  }
  return value;
}
