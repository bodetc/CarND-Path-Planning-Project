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

  /**
   * Taking previous path into account
   */
  unsigned long prev_size = previous_path_x.size();
  double lag = (double) prev_size * TIMESTEP; // Time in the future we will write our points
  double current_car_s = car_s; // Current position
  double lag_car_s = prev_size > 0 ? end_path_s : car_s; // Position at the end of the previous path

  /**
   * Velocity check
   */

  // Check if any car is present in the current or target lane (if different)
  bool too_close = sensor_fusion.is_too_close(lag, current_lane, current_car_s, lag_car_s)
                   || (is_changing_lane() && sensor_fusion.is_too_close(lag, target_lane, current_car_s, lag_car_s));
  if (too_close) {
    ref_vel -= SPEED_STEP;
  } else if (ref_vel < TARGET_SPEED) {
    ref_vel += SPEED_STEP;
  }

  /**
   * Lane change logic
   */

  if (is_changing_lane()) {
    //  If currently passing, check if lane change is over
    if (Map::is_in_lane(end_path_d, target_lane)) {
      current_lane = target_lane;
      if (DEBUG) cout << "Change lane completed! lane " << current_lane << endl;
    }
  } else {
    // Check for passing only if there is a car in front and the speed is below some threshold
    if (too_close && ref_vel < PASSING_SPEED) {
      if (DEBUG) cout << "Check for passing opportunity... ";

      // Check first left then right, and change target lane if lane is free
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

  // Append trajectory using the new reference velocity and target lane
  return map.complete_trajectory(target_lane, ref_vel, car_x, car_y, lag_car_s, car_yaw, previous_path_x,
                                 previous_path_y);
}


