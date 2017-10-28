//
// Created by Cédric Bodet on 27.10.17.
//

#ifndef PATH_PLANNING_CONTROLLER_H
#define PATH_PLANNING_CONTROLLER_H

#include "../util/Trajectory.h"
#include "../util/Map.h"

class Controller {
  const Map map;

public:
  Controller(const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s);

  Trajectory computeTrajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);

private:
  Trajectory keepLane(double car_s, double car_d, double car_speed);
};


#endif //PATH_PLANNING_CONTROLLER_H
