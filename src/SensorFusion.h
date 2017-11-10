//
// Created by Cédric Bodet on 09.11.17.
//

#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H

#include "json.hpp"
#include "Vehicle.h"

class SensorFusion {
private:
  std::vector<Vehicle> observations;

public:
  explicit SensorFusion(const nlohmann::json &sensor_fusion);

  // Returns true if there is no one in the given lane around ego, and that the lane is safe to change to
  bool is_lane_empty(double lag, int lane, double car_s) const;

  // Returns true if there is a car in the current lane that is too close to ego
  bool is_too_close(double lag, int lane, double current_car_s, double lag_car_s) const;

};


#endif //PATH_PLANNING_SENSORFUSION_H
