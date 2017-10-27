//
// Created by Cédric Bodet on 27.10.17.
//

#ifndef PATH_PLANNING_CONTROLLER_H
#define PATH_PLANNING_CONTROLLER_H

#include "Trajectory.h"
#include "Map.h"

class Controller {
  const Map& map;

public:
  explicit Controller(const Map& map);

  Trajectory
  computeTrajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);
};


#endif //PATH_PLANNING_CONTROLLER_H
