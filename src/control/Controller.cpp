//
// Created by Cédric Bodet on 27.10.17.
//

#include <iostream>
#include "Controller.h"
#include "../definitions.h"
#include "../utils.h"
#include "../trajectory/PolynomialSolver.h"
#include "../behaviour/TargetFinder.h"

using namespace std;

Controller::Controller(const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s) :
    map(maps_x, maps_y, maps_s) {}


const Trajectory
Controller::compute_trajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                               const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
                               const std::vector<Vehicle> &predictions) {
  State start_state{
      .s = car_s,
      .s_dot = car_speed,
      .s_ddot = 0,
      .d = car_d,
      .d_dot = 0,
      .d_ddot = 0,
      .t = 0
  };

  Trajectory lag_trajectory = update_state_for_lag(start_state, previous_path_x, previous_path_y);

  cout << "Ego=(" << start_state.s << ", " << start_state.d << ")" << endl;

  Trajectory new_trajectory = keep_lane(start_state, predictions);

  previous_trajectory = lag_trajectory;
  previous_trajectory.push_all_back(new_trajectory);
  return map.getXYspline(previous_trajectory);
}

const Trajectory
Controller::update_state_for_lag(State &start_state,
                                 const std::vector<double> &previous_path_x,
                                 const std::vector<double> &previous_path_y) {
  Trajectory trajectory;

  // Do nothing if not enough steps are present
  if (previous_path_x.size() >= N_LAG && previous_path_x.size() >= N_LAG) {
    // Calculate number of steps elapsed
    unsigned long elapsed = previous_trajectory.size() - previous_path_x.size();

    cout << "Elapsed: " << elapsed << endl;

    // Use some steps for the previous list to allow for lag
    for (int i = 0; i < N_LAG; i++) {
      trajectory.push_back(previous_trajectory.at(elapsed + i));
    }

    // Now update the car state
    unsigned long current = elapsed + N_LAG - 1;
    start_state = get_state_from_trajectory(previous_trajectory, current);
  }

  return trajectory;
}

const State Controller::get_state_from_trajectory(const Trajectory &previous_trajectory, unsigned long current) {
  auto current_pos = previous_trajectory.at(current);
  auto previous_pos = previous_trajectory.at(current - 1);
  auto ante_pos = previous_trajectory.at(current - 2);

  double current_s = current_pos[0];
  double current_d = current_pos[1];

  double current_s_dot = (current_pos[0] - previous_pos[0]) / TIMESTEP;
  double current_d_dot = (current_pos[1] - previous_pos[1]) / TIMESTEP;

  double previous_s_dot = (previous_pos[0] - ante_pos[0]) / TIMESTEP;
  double previous_d_dot = (previous_pos[1] - ante_pos[1]) / TIMESTEP;

  return State {
      // Current position
      .s = current_s,
      .d = current_d,

      // Current speed
      .s_dot = current_s_dot,
      .d_dot = current_d_dot,

      // Current acceleration
      .s_ddot = (current_s_dot - previous_s_dot) / TIMESTEP,
      .d_ddot = (current_d_dot - previous_d_dot) / TIMESTEP,
  };
}

const Trajectory Controller::keep_lane(const State &ego, const std::vector<Vehicle> &predictions) {
  double T = HORIZON;
  int target_lane = 1;
  double max_distance = TARGET_SPEED * HORIZON;

  State delta{.s=-FOLLOW_DISTANCE};

  Vehicle target = TargetFinder::getTarget(ego, target_lane, max_distance, predictions);
  State goal = target.stateAt(T)+delta;
  cout << "Goal=(" << goal.s << "," << goal.d << ")" << endl;
  return ptg.getTrajectory(ego, target, delta, T, predictions);
}
