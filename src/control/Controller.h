//
// Created by Cédric Bodet on 27.10.17.
//

#ifndef PATH_PLANNING_CONTROLLER_H
#define PATH_PLANNING_CONTROLLER_H

#include "../util/Trajectory.h"
#include "../util/Map.h"
#include "../util/State.h"
#include "../trajectory/PTG.h"

class Controller {
  const Map map;
  PTG ptg;

  Trajectory previous_trajectory;

public:
  Controller(const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s);

  const Trajectory compute_trajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw,
                                      double car_speed,
                                      const std::vector<double> &previous_path_x,
                                      const std::vector<double> &previous_path_y,
                                      const std::vector<Vehicle>& predictions);

private:
  const Trajectory update_state_for_lag(State &start_state,
                                        const std::vector<double> &previous_path_x,
                                        const std::vector<double> &previous_path_y);

  const State get_state_from_trajectory(const Trajectory &previous_trajectory, unsigned long current);

  const Trajectory keep_lane(State start_state, const std::vector<Vehicle>& predictions);
};


#endif //PATH_PLANNING_CONTROLLER_H
