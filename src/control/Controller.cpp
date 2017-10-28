//
// Created by Cédric Bodet on 27.10.17.
//

#include <iostream>
#include "Controller.h"
#include "../definitions.h"
#include "../trajectory/Polynomial.h"
#include "../trajectory/PolynomialSolver.h"

using namespace std;


Controller::Controller(const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s) :
    map(maps_x, maps_y, maps_s) {}


Trajectory Controller::computeTrajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw,
                                         double car_speed) {
  return keepLane(car_s, car_d, car_speed);
}

Trajectory Controller::keepLane(double car_s, double car_d, double car_speed) {
  double distance = TARGET_SPEED * HORIZON;
  vector<double> start = {0, car_speed, 0};
  vector<double> end = {distance, TARGET_SPEED, 0};

  Polynomial polynomial = PolynomialSolver::solveJMT(start, end, HORIZON);

  Trajectory trajectory;
  for (int i = 1; i <= N_STEPS; i++) {
    double s = car_s + polynomial.evaluate(i * TIMESTEP);
    double d = 6.;
    trajectory.push_back(map.getXY(s, d));
  }
  return trajectory;
}
