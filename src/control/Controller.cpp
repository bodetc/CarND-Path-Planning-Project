//
// Created by Cédric Bodet on 27.10.17.
//

#include <iostream>
#include "Controller.h"
#include "../definitions.h"
#include "../utils.h"
#include "../trajectory/Polynomial.h"
#include "../trajectory/PolynomialSolver.h"

using namespace std;


Controller::Controller(const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s) :
    map(maps_x, maps_y, maps_s) {}


Trajectory
Controller::computeTrajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                              const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y) {
  ego_speed = car_speed;
  ego_a = 0.;
  ego_s = car_s;
  ego_d = car_d;

  Trajectory lagTrajectory = updateStateForLag(previous_path_x, previous_path_y);

  cout << "Ego=(" << ego_s << ", " << ego_d << ")" << endl;

  Trajectory newTrajectory = keepLane();

  previous_trajectory = lagTrajectory;
  previous_trajectory.push_all_back(newTrajectory);
  return map.getXYspline(previous_trajectory);
}

Trajectory Controller::updateStateForLag(const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y) {
  Trajectory trajectory;

  // Do nothing if not enough steps are present
  if (previous_path_x.size() >= N_LAG && previous_path_x.size() >= N_LAG) {
    // Calculate number of steps elapsed
    unsigned long elapsed = previous_trajectory.size() - previous_path_x.size();
    unsigned long current = elapsed+N_LAG-1;

    cout << "Elapsed: " << elapsed << endl;

    // Use some steps for the previous list to allow for lag
    for (int i = 0; i < N_LAG; i++) {
      trajectory.push_back(previous_trajectory.at(elapsed+i));
    }

    // Now update the car parameters

    // "Current" Frenet coordinates
    auto ego_sd = previous_trajectory.at(current);
    ego_s = ego_sd[0];
    ego_d = ego_sd[1];

    // "Current" and previous positions
    auto previous_sd = previous_trajectory.at(current-1);
    auto ante_sd = previous_trajectory.at(current-2);

    // "Current" and previous speed
    ego_speed = distance(ego_sd, previous_sd) / TIMESTEP;
    double previous_speed = distance(previous_sd, ante_sd) / TIMESTEP;

    // "Current" acceleration
    ego_a = (ego_speed - previous_speed) / TIMESTEP;
  }

  return trajectory;
}

Trajectory Controller::keepLane() {
  double distance = TARGET_SPEED * HORIZON;
  vector<double> start = {0, ego_speed, ego_a};
  vector<double> end = {distance, TARGET_SPEED, 0};

  Polynomial polynomial = PolynomialSolver::solveJMT(start, end, HORIZON);

  Trajectory trajectory;
  for (int i = 1; i <= N_STEPS; i++) {
    double s = ego_s + polynomial.evaluate(i * TIMESTEP);
    double d = 6.;
    trajectory.push_back(s, d);
  }
  return trajectory;
}
