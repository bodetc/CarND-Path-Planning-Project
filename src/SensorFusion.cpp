//
// Created by Cédric Bodet on 09.11.17.
//

#include "SensorFusion.h"
#include "utils.h"
#include "Map.h"

using namespace std;

SensorFusion::SensorFusion(const nlohmann::json &sensor_fusion) {
  for (auto vehicle : sensor_fusion) {
    double s = vehicle[5];
    double d = vehicle[6];

    double vx = vehicle[3];
    double vy = vehicle[4];
    double speed = norm(vx, vy); // Assume forward velocity only

    observations.emplace_back(s, d, speed);
  }
}

// Returns true if there is a car in the current lane that is too close to ego
bool SensorFusion::is_too_close(double lag, int lane, double current_car_s, double lag_car_s) const {
  // Find speed to use based on the car in front
  for (const auto &vehicle : observations) {
    // Car is in my lane and currently in front of me
    if (Map::is_in_lane(vehicle.d, lane) && vehicle.s > current_car_s) {
      // Car will be too close at time t=lag
      if ((vehicle.s_at(lag) - lag_car_s) < FORWARD_DISTANCE) {
        if (VERBOSE) cout << "too close: " << vehicle.s_at(lag) - lag_car_s << " lane " << lane << endl;
        return true;
      }
    }
  }
  return false;
}

// Returns true if there is no one in the given lane around ego, and that the lane is safe to change to
bool SensorFusion::is_lane_empty(double lag, int lane, double car_s) const {
  // Find speed to use based on the car in front
  for (const auto &vehicle : observations) {
    // Car is in my lane
    if (Map::is_in_lane(vehicle.d, lane)) {
      // Relative position of the car a the end of the previous trajectory
      double distance = vehicle.s_at(lag) - car_s;

      // check that the car is not in the gap
      if (distance > -LANE_CHECK_BACKWARDS_DISTANCE
          && distance < LANE_CHECK_FORWARD_DISTANCE) {
        return false;
      }
    }
  }
  return true;
}
