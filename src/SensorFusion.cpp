//
// Created by Cédric Bodet on 09.11.17.
//

#include "SensorFusion.h"

using namespace std;

SensorFusion::SensorFusion(const nlohmann::json &sensor_fusion) : sensor_fusion(sensor_fusion) {}

bool SensorFusion::isTooClose(double lag, int lane, double car_s) const {
  // Find speed to use based on the car in front
  for (int i = 0; i < sensor_fusion.size(); i++) {

    float d = sensor_fusion[i][6];

    // Car is in my lane
    if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
      int id = sensor_fusion[i][0];
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed_front = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += (lag * check_speed_front); // If using previous points, can project S value out
      //check S value greater than mine and s gap
      if ((check_car_s >= car_s) && (check_car_s - car_s) < 30) {
        cout << "too close" << (check_car_s - car_s) << " lane" << lane << endl;
        return true;
      }
    }
  }
  return false;
}
