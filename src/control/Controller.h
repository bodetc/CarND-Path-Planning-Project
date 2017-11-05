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
  const PTG ptg;

  Trajectory previous_trajectory;

public:
  Controller(const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s);

  Trajectory computeTrajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
    const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y);

private:
  Trajectory updateStateForLag(State& start_state, const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y);
  State updateState(const Trajectory& previous_trajectory, unsigned long current);

  Trajectory keepLane(const State& start_state);
};


#endif //PATH_PLANNING_CONTROLLER_H
