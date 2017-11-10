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

  // Adapt velocity to any car in the current lane, or the target lane if different
  bool too_close = sensor_fusion.is_too_close(lag, current_lane, current_car_s, lag_car_s)
                   || (is_passing() && sensor_fusion.is_too_close(lag, target_lane, current_car_s, lag_car_s));

  if (too_close) {
    ref_vel -= SPEED_STEP;
  } else if (ref_vel < TARGET_SPEED) {
    ref_vel += SPEED_STEP;
  }

  // If currently passing
  if (is_passing()) {
    // Check if lane change is over
    if (Map::is_in_lane(end_path_d, target_lane)) {
      current_lane = target_lane;
      if (DEBUG) cout << "Change lane completed! lane " << current_lane << endl;
    }
  } else {
    // Check for passing only if car in front and speed below some threshold
    if (too_close && ref_vel < PASSING_SPEED) {
      if (DEBUG) cout << "Check for passing opportunity... ";

      // Left check
      if (current_lane > 0 && sensor_fusion.is_lane_empty(lag, current_lane - 1, lag_car_s)) {
        target_lane = current_lane - 1;
        if (DEBUG) cout << "Left lane change to lane " << target_lane;
      } else if (current_lane < N_LANES-1 && sensor_fusion.is_lane_empty(lag, current_lane + 1, lag_car_s)) {
        target_lane = current_lane + 1;
        if (DEBUG) cout << "Right lane change to lane " << target_lane;
      }
      if (DEBUG) cout << endl;
    }
  }

  return map.complete_trajectory(target_lane, ref_vel, car_x, car_y, lag_car_s, car_yaw, previous_path_x,
                                 previous_path_y);
}


