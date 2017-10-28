//
// Created by Cédric Bodet on 27.10.17.
//

#ifndef PATH_PLANNING_CONTROLLER_H
#define PATH_PLANNING_CONTROLLER_H

#include "../util/Trajectory.h"
#include "../util/Map.h"

class Controller {
  const Map map;

  double ego_speed;
  double ego_a;
  double ego_s;
  double ego_d;

  Trajectory previous_trajectory;

public:
  Controller(const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s);

  Trajectory computeTrajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
    const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y);

private:
  Trajectory updateStateForLag(const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y);

  Trajectory keepLane();
};


#endif //PATH_PLANNING_CONTROLLER_H
