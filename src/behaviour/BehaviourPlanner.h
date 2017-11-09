//
// Created by Cédric Bodet on 09.11.17.
//

#ifndef PATH_PLANNING_BEHAVIOUR_H
#define PATH_PLANNING_BEHAVIOUR_H


#include "../util/Vehicle.h"

class BehaviourPlanner {
private:
  int last_lane_check = -50;
  int target_lane = 1;

  void updateTargetLane(const State& ego, const std::vector<Vehicle> &predictions);
  bool checkRightLane(const State& ego, const std::vector<Vehicle> &predictions);
  bool checkLeftLane(const State& ego, const std::vector<Vehicle> &predictions);
  bool shouldOvertake(const State& ego, const std::vector<Vehicle> &predictions);

public:
  const Vehicle update(const State& ego, const std::vector<Vehicle> &predictions);
};


#endif //PATH_PLANNING_BEHAVIOUR_H
