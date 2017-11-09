//
// Created by Cédric Bodet on 08.11.17.
//

#ifndef PATH_PLANNING_TARGETFINDER_H
#define PATH_PLANNING_TARGETFINDER_H

#include "../util/Vehicle.h"

class TargetFinder {
  static int getInLane(const State &ego, int target_lane, double max_distance, const std::vector<Vehicle> &predictions, bool forward);

public:
  static const Vehicle getTarget(const State& ego, int lane_delta, double max_distance, const std::vector<Vehicle> &predictions);

  static int getNextInLane(const State &ego, int target_lane, double max_distance, const std::vector<Vehicle> &predictions);
  static int getPreviousInLane(const State &ego, int target_lane, double max_distance, const std::vector<Vehicle> &predictions);
};


#endif //PATH_PLANNING_TARGETFINDER_H
