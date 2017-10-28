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

  Trajectory sdTrajectory;
  Trajectory xyTrajectory;
  double dist_inc = 0.4;
  for (int i = 0; i < 50; i++) {
    sdTrajectory.push_back(i*dist_inc, 6.);
    xyTrajectory.push_back(map.getXY(car_s+i*dist_inc, 6.));
  }

  return xyTrajectory;
}
