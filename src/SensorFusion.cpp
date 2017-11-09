//
// Created by Cédric Bodet on 09.11.17.
//

#include "SensorFusion.h"
#include "utils.h"
#include "Map.h"

using namespace std;

SensorFusion::SensorFusion(const nlohmann::json &sensor_fusion) : sensor_fusion(sensor_fusion) {}

bool SensorFusion::is_too_close(double lag, int lane, double car_s) const {
  // Find speed to use based on the car in front
  for (const auto &vehicle : sensor_fusion) {

    float d = vehicle[6];

    // Car is in my lane
    if (Map::is_in_lane(d, lane)) {
      double vx = vehicle[3];
      double vy = vehicle[4];
      double check_speed_front = norm(vx, vy);
      double check_car_s = vehicle[5];

      // If using previous points, can project S value out
      check_car_s += (lag * check_speed_front);

      //check S value greater than mine and s gap
      if ((check_car_s >= car_s) && (check_car_s - car_s) < FORWARD_DISTANCE) {
        if (DEBUG) cout << "too close" << (check_car_s - car_s) << " lane" << lane << endl;
        return true;
      }
    }
  }
  return false;
}
