//
// Created by Cédric Bodet on 09.11.17.
//

#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H

#include "json.hpp"

class SensorFusion {
private:
  const nlohmann::json &sensor_fusion;

public:
  SensorFusion(const nlohmann::json &sensor_fusion);

  // Returns true if there is a car in the current lane that is too close to ego
  bool is_too_close(double lag, int lane, double car_s) const;
};


#endif //PATH_PLANNING_SENSORFUSION_H
