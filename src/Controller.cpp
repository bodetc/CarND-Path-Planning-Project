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
  unsigned long prev_size = previous_path_x.size();
  double lag = (double) prev_size * TIMESTEP;
  double current_car_s = car_s;
  double lag_car_s = prev_size > 0 ? end_path_s : car_s;

  // Adapt velocity to any car in the lane
  bool too_close = sensor_fusion.is_too_close(lag, lane, current_car_s, lag_car_s);
  if (too_close) {
    ref_vel -= SPEED_STEP;
  } else if (ref_vel < TARGET_SPEED) {
    ref_vel += SPEED_STEP;
  }

  // Check for passing opportunity
  if (too_close && ref_vel < PASSING_SPEED) {
    // Left check
    if (lane > 0 && sensor_fusion.is_lane_empty(lag, lane - 1, lag_car_s)) {
      cout << "Left lane change";
      lane--;
    } else if (lane < N_LANES && sensor_fusion.is_lane_empty(lag, lane + 1, lag_car_s)) {
      cout << "Right lane change";
      lane++;
    }
  }

  return map.complete_trajectory(lane, ref_vel, car_x, car_y, lag_car_s, car_yaw, previous_path_x, previous_path_y);
}


