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

bool SensorFusion::is_too_close(double lag, int lane, double car_s) const {
  // Find speed to use based on the car in front
  for (const auto &vehicle : observations) {
    // Car is in my lane and in front of me
    if (Map::is_in_lane(vehicle.d, lane) && vehicle.s > car_s) {
      // Car will be too close at time t=lag
      if ((vehicle.s_at(lag) - car_s) < FORWARD_DISTANCE) {
        if (DEBUG) cout << "too close: " << vehicle.s_at(lag) - car_s << " lane " << lane << endl;
        return true;
      }
    }
  }
  return false;
}

bool SensorFusion::is_car_in_range(double lag, int lane, double car_s, double range, bool forward) const {
  // Find speed to use based on the car in front
  for (const auto &vehicle : observations) {
    // Car is in my lane
    if (Map::is_in_lane(vehicle.d, lane)) {
      // Position of the car a the end of the previous trajectory
      double s = vehicle.s_at(lag);

      //check S value greater than mine and s gap
      double distance = s - car_s;

      if (forward ? distance > 0 : distance <= 0 // Car is forward or back?
                                   && fabs(distance) < range // S gap smaller than expected?
          ) {
        return true;
      }
    }
  }
  return false;
}
