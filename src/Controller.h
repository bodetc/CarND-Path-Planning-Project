//
// Created by Cédric Bodet on 27.10.17.
//

#ifndef PATH_PLANNING_CONTROLLER_H
#define PATH_PLANNING_CONTROLLER_H

#include "Trajectory.h"
#include "Map.h"
#include "SensorFusion.h"

class Controller {
  const Map map;

  double ref_vel = 0;

  int current_lane = 1;
  int target_lane = 1;

  inline bool is_passing() { return current_lane != target_lane; }

public:
  Controller(const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s);

  const Trajectory
  compute_trajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                     const std::vector<double> &previous_path_x,
                     const std::vector<double> &previous_path_y,
                     double end_path_s, double end_path_d,
                     const SensorFusion &sensor_fusion);
};


#endif //PATH_PLANNING_CONTROLLER_H
