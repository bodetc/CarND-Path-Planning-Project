//
// Created by Cédric Bodet on 27.10.17.
//

#include "Controller.h"
#include "../utils.h"

using namespace std;

Controller::Controller(const Map &map) : map(map) {}

Trajectory Controller::computeTrajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw,
                                         double car_speed) {
  double dist_inc = 0.45;
  Trajectory sdTrajectory;
  for (int i = 0; i < 50; i++) {
    sdTrajectory.push_back(car_s + i * dist_inc, 6.);
  }

  return map.getXY(sdTrajectory);
}
