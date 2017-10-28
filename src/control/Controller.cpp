//
// Created by Cédric Bodet on 27.10.17.
//

#include "Controller.h"
#include "../utils.h"

using namespace std;


Controller::Controller(const std::vector<double> &maps_x, const std::vector<double> &maps_y,
                       const std::vector<double> &maps_s) : map(maps_x, maps_y, maps_s) {}


Trajectory Controller::computeTrajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw,
                                         double car_speed) {

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double dist_inc = 0.1;
  for (int i = 0; i < 50; i++) {
    next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
    next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
  }

  return Trajectory(next_x_vals, next_y_vals);
}
