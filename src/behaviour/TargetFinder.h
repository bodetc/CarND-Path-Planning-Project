//
// Created by Cédric Bodet on 08.11.17.
//

#ifndef PATH_PLANNING_TARGETFINDER_H
#define PATH_PLANNING_TARGETFINDER_H

#include "../util/Vehicle.h"

class TargetFinder {
public:
  static const Vehicle getTarget(const State& ego, int lane_delta, double max_distance, const std::vector<Vehicle> &predictions);
};


#endif //PATH_PLANNING_TARGETFINDER_H
