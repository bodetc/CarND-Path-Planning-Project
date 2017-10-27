//
// Created by Cédric Bodet on 27.10.17.
//

#include "Controller.h"
#include "../utils.h"
#include "../trajectory/QuinticPolynomialSolver.h"
#include "../definitions.h"

using namespace std;

Controller::Controller(const Map &map) : map(map) {}

Trajectory Controller::computeTrajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw,
                                         double car_speed) {
  int n = 50;
  double T = n * TIMESTEP;
  double targetDistance = TARGET_SPEED * T;

  vector<double> start = {car_s, car_speed, 0.};
  vector<double> end = {car_s + targetDistance, TARGET_SPEED, 0.};

  Polynomial polynomial = QuinticPolynomialSolver::solveJMT(start, end, T);

  Trajectory sdTrajectory;
  for (int i = 0; i < 50; i++) {
    double dist_inc = 0.45;
    sdTrajectory.push_back(car_s + i * dist_inc, 6.);
  }

  Trajectory xyTrajectory = map.getXY(sdTrajectory);
  return xyTrajectory;
}
