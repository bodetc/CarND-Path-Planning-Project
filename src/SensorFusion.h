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
  SensorFusion(const nlohmann::json &sensor_fusion);

  bool is_car_in_range(double lag, int lane, double car_s, double range, bool forward) const;

  // Returns true if there is a car in the current lane that is too close to ego
  bool is_too_close(double lag, int lane, double car_s) const;

};


#endif //PATH_PLANNING_SENSORFUSION_H
