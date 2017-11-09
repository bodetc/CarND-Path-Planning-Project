//
// Created by Cédric Bodet on 27.10.17.
//

#include "Controller.h"

using namespace std;

Controller::Controller(const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s) :
    map(maps_x, maps_y, maps_s) {}

const Trajectory
Controller::compute_trajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                               const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
                               double end_path_s, double end_path_d,
                               const SensorFusion &sensor_fusion) {

  // Take care to the data coming from the previous path
  int prev_size = previous_path_x.size();
  double lag = (double) prev_size * TIMESTEP;
  if (prev_size > 0) {
    car_s = end_path_s;
  }

  // Adapt velocity to any car in the lane
  bool too_close = sensor_fusion.is_too_close(lag, lane, car_s);
  if (too_close) {
    ref_vel -= SPEED_STEP;
  } else if (ref_vel < TARGET_SPEED) {
    ref_vel += SPEED_STEP;
  }

  return map.complete_trajectory(lane, ref_vel, car_x, car_y, car_s, car_yaw, previous_path_x, previous_path_y);
}


