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

  bool isTooClose(double lag, int lane, double car_s) const;
};


#endif //PATH_PLANNING_SENSORFUSION_H
